/*
 * Driver for the Symmetrical SPI communication between independent
 * CPUs, which uses the SPI bus + 2 GPIO handshaking lines to implement
 * full duplex and fully symmetrical communication between parties.
 *
 * Copyright (c) 2020 Robert Bosch GmbH
 * Artem Gulyaev <Artem.Gulyaev@de.bosch.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// SPDX-License-Identifier: GPL-2.0

#include "symspi.h"

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/printk.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>


// DEV STACK
//
// VERIFICATION:
//
//      * On transport failure, report to the consumer
//        layer and by default restart data package transmission,
//        or do as consumenr layer orders.
//
//      * Introduce usage of SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC timeout
//        in waiting for their flag and if timed out, then do error
//        recovery.
//
// WIP:
//
//      * To add minimal time interval for repeated error reports
//        (to avoid flooding in logs). This will require some error
//        histogramm tracking to provide information about how many
//        times this error happened while output was inhibited.
//
// BACKLOG:
//
//      * Make a single goto_error routine to make similar actions
//        on errors (like checking for nested error state, etc.)
//
//      * [Max] better practice to use 'struct device* dev' to
//        point to device. Consider migration to conventional device
//        addressing and inheritance.
//
//      * We might fall in data races with timeout timer launching, when
//        the timer is about to be started upon entering the WAITING state
//        and at this point interrupt comes and discards the WAITING state
//        and deletes the timer, and then we start the timer from resumed
//        thread.
//                    TH                            INT
//              STATE -> WAITING
//                                              STATE -> XFER
//                                              del_timer_sync()
//               add_timer()
//
//        This will lead to ticking timer while it is already should be
//        disabled.
//
//        Pobably we will have to switch to mutexes finally, which is
//        not that good.
//
//      * Error recovery is to be launched only when we are not
//        ALREADY in ERROR state. See __symspi_other_side_wait_timeout()
//        comments.
//
//      * Add error (our/their) handling tests.
//
//      * Add the test/debug method to force the SymSPI fall into error
//        mode (simulating other side error).
//
//      * Check that in any sequence step the current state is checked
//        for being ERROR, and if so, standard workflow is aborted.
//
//      * Add close_request checks to all entry points of SymSPI
//
//      * To cover the iccom test case 22, which closes immediately
//        after posting the message. We need to forbid the closing
//        while we are not in idle state. Stopped in symspi_close.
//
//      * adjust code style according to the Linux kernel code style
//
//      * kmem_cache_free check if needed (review) sockaddr_nl
//
// DEV STACK END

/* -- HARD CONFIGURATION DEFINED AT BUILD TIME (related to target HW) -- */

// if defined all assertions are enabled
//#define SYMSPI_DEBUG

#define SYMSPI_LOG_PREFIX "SymSPI: "

// The maximum single xfer size in bytes
// (is limited by the SPI hardware FIFO buffer)
//
// NOTE: usually it is the same for all SPI controllers
//          on chip and for both Master and Slave modes
#define SYMSPI_XFER_SIZE_MAX_BYTES 64

// Which TTL level will be interpreted as ACTIVE flag state
#define SYMSPI_MASTER_FLAG_ACTIVE_VALUE 1
#define SYMSPI_SLAVE_FLAG_ACTIVE_VALUE 1

// the minimal delay between drop of our flag and its raise
// (is introduced cause other side may not detect very fast
// drop-change of our flag, that is why we may need to slow
// down a bit)
//
// It looks like other side might ignore our flag raise
// if not enough time passed since its drop.
//
// Can be set via kernel config.
#define SYMSPI_OUR_FLAG_INACTIVE_STATE_MIN_TIME_USEC 750
// The required precision of inactive interval in percents
#define SYMSPI_OUR_FLAG_INACTIVE_STATE_MIN_TIME_VARIANCE_PERCENT 10

// The timeout of waiting for other side to raise their flag
// (after that timeout SymSPI should go to error state and
// attempt to recover from error)
//
// NOTE: don't set the timeout lower than 3 jiffies (usually 30ms),
//      cause the relative error will be too high (for example, if
//      timer is set at the end of current jiffie, it will be timed
//      out immediately on the next timer tick, which might happen
//      say in 25 microseconds, instead of 10 milliseconds,
//      triggering false positive timeout indication, in practice
//      difference of 0.2ms instead of 10ms set was observed)
//
// NOTE: one more recomendation: set this value to high enough value
//  	which would reasonably can be considered as "not-normal/
//  	suspicious" delay which might indicate a hang or some lock,
//  	so the other side can be "woken up"/purged using the SymSPI
//  	error indication flag line oscillation. Too low value will
//  	trigger unnecessary error recovery procedures which will slow
//  	us down without a reason to do so.
#ifndef SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC
#define SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC 60
#endif

// The duration of the silence which immediately follows
// the error recovery procedure (this is actually the time
// between the error indication to the other side and resuming
// normal workflow)
#define SYMSPI_ERROR_RECOVERY_SILENCE_TIME_MS 10
// The required precision of silence time waiting
#define SYMSPI_ERROR_RECOVERY_SILENCE_TIME_VARIANCE_PERCENT 5

// The timeout to wait for hardware xfer to be finished
// on device closing (in milliseconds)
#define SYMSPI_CLOSE_HW_WAIT_TIMEOUT_MSEC 500


// Selects the workqueue to use to run operations ordered
// from interrupt context.
// Three options are available now:
// * "SYMSPI_WQ_SYSTEM": see system_wq in workqueue.h.
// * "SYMSPI_WQ_SYSTEM_HIGHPRI": see system_highpri_wq in
//   workqueue.h.
// * "SYMSPI_WQ_PRIVATE": use privately constructed high priority
//   workqueue.
//
// NOTE: the selection of the workqueue depends on the
//      generic considerations on SymSPI functioning
//      within the overall system context. Say if SymSPI serves
//      as a connection for an optional device, or device which
//      can easily wait for some sec (in the worst case) to react
//      then "SYMSPI_WQ_SYSTEM" workqeue is a nice option to select.
//      On the other hand if no delays are allowed in handling SymSPI
//      communication (say, to communicate to hardware watchdog)
//      then "SYMSPI_WQ_SYSTEM_HIGHPRI" or "SYMSPI_WQ_PRIVATE" is
//      surely more preferrable.
//
// Can be set via kernel config.
#ifndef SYMSPI_WORKQUEUE_MODE
#define SYMSPI_WORKQUEUE_MODE SYMSPI_WQ_PRIVATE
#endif

#define SYMSPI_WQ_SYSTEM 0
#define SYMSPI_WQ_SYSTEM_HIGHPRI 1
#define SYMSPI_WQ_PRIVATE 2

// Comparator
#define SYMSPI_WQ_MODE_MATCH(x)		\
	SYMSPI_WORKQUEUE_MODE == SYMSPI_WQ_##x

#ifndef SYMSPI_WORKQUEUE_MODE
#error SYMSPI_WORKQUEUE_MODE must be defined to \
		one of [SYMSPI_WQ_SYSTEM, SYMSPI_WQ_SYSTEM_HIGHPRI, \
		SYMSPI_WQ_PRIVATE].
#endif

// Define this to work as SPI master (for now
// SPI master is the only option).
//
// Can be set via kernel config.
#define SYMSPI_SPI_MASTER true

// Defines the log verbosity level for SymSPI
// 0: total silence
// 1: only error messages
// 2: + warnings
// 3: (DEFAULT) + key info messages (info level 0)
// 4: + optional info messages (info level 1)
// 5: + debug check points (info level 2 == debug level 1)
// 6: + trace level, print everything, will flood
//      if communication is actively used.
#define SYMSPI_VERBOSITY 3

// The minimal time which must pass between repeated error is reported
// to avoid logs flooding.
// 0: no minimal interval
// >0: minimal time interval in mseconds
#define SYMSPI_MIN_ERR_REPORT_INTERVAL_MSEC 10000
// The rate (number of msec) at which the error rate decays double.
// The error rate is relevant in interpretation of the errors,
// cause occasional errors usually don't have very high significance,
// while high error rate usually indicates a real fault.
// Surely: > 0
#define SYMSPI_ERR_RATE_DECAY_RATE_MSEC_PER_HALF 2000
// Minimal decay rate, even if error events are sequential
#define SYMSPI_ERR_RATE_DECAY_RATE_MIN 3

/* --------------------- UTILITIES SECTION ----------------------------- */

#define macro_val_str(s) stringify(s)
#define stringify(s) #s

#if SYMSPI_VERBOSITY >= 1
#define symspi_err(fmt, ...)						\
	pr_err(SYMSPI_LOG_PREFIX"%s: "fmt"\n", __func__, ##__VA_ARGS__)
#define symspi_err_raw(fmt, ...)					\
	pr_err(SYMSPI_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#else
#define symspi_err(fmt, ...)
#define symspi_err_raw(fmt, ...)
#endif

#if SYMSPI_VERBOSITY >= 2
#define symspi_warning(fmt, ...)					\
	pr_warning(SYMSPI_LOG_PREFIX"%s: "fmt"\n", __func__		\
		   , ##__VA_ARGS__)
#define symspi_warning_raw(fmt, ...)					\
	pr_warning(SYMSPI_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#else
#define symspi_warning(fmt, ...)
#define symspi_warning_raw(fmt, ...)
#endif

#if SYMSPI_VERBOSITY >= 3
#define symspi_info_helper(fmt, ...)					\
	pr_info(SYMSPI_LOG_PREFIX"%s: "fmt"\n", __func__, ##__VA_ARGS__)
#define symspi_info_raw_helper(fmt, ...)				\
	pr_info(SYMSPI_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#define symspi_info_helper_0(fmt, ...)					\
	symspi_info_helper(fmt, ##__VA_ARGS__)
#define symspi_info_raw_helper_0(fmt, ...)				\
	symspi_info_raw_helper(fmt, ##__VA_ARGS__)
#else
#define symspi_info_helper(fmt, ...)
#define symspi_info_raw_helper(fmt, ...)
#define symspi_info_helper_0(fmt, ...)
#define symspi_info_raw_helper_0(fmt, ...)
#endif

#if SYMSPI_VERBOSITY >= 4
#define symspi_info_helper_1(fmt, ...)					\
	symspi_info_helper(fmt, ##__VA_ARGS__)
#define symspi_info_raw_helper_1(fmt, ...)				\
	symspi_info_raw_helper(fmt, ##__VA_ARGS__)
#else
#define symspi_info_helper_1(fmt, ...)
#define symspi_info_raw_helper_1(fmt, ...)
#endif

#if SYMSPI_VERBOSITY >= 5
#define symspi_info_helper_2(fmt, ...)					\
	symspi_info_helper(fmt, ##__VA_ARGS__)
#define symspi_info_raw_helper_2(fmt, ...)				\
	symspi_info_raw_helper(fmt, ##__VA_ARGS__)
#else
#define symspi_info_helper_2(fmt, ...)
#define symspi_info_raw_helper_2(fmt, ...)
#endif

#if SYMSPI_VERBOSITY >= 6
#define symspi_trace(fmt, ...)					\
	pr_info(SYMSPI_LOG_PREFIX"%s: "fmt"\n", __func__		\
		   , ##__VA_ARGS__)
#define symspi_trace_raw(fmt, ...)					\
	pr_info(SYMSPI_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#else
#define symspi_trace(fmt, ...)
#define symspi_trace_raw(fmt, ...)
#endif


// information messages levels
#define SYMSPI_LOG_INFO_KEY_LEVEL 0
#define SYMSPI_LOG_INFO_OPT_LEVEL 1
#define SYMSPI_LOG_INFO_DBG_LEVEL 2

#define symspi_info_helper__(level, fmt, ...)				\
	symspi_info_helper_##level(fmt, ##__VA_ARGS__)
#define symspi_info_raw_helper__(level, fmt, ...)			\
	symspi_info_raw_helper_##level(fmt, ##__VA_ARGS__)

#define symspi_info(level, fmt, ...)					\
	symspi_info_helper__(level, fmt, ##__VA_ARGS__)
#define symspi_info_raw(level, fmt, ...)				\
	symspi_info_raw_helper__(level, fmt, ##__VA_ARGS__)

#define SYMSPI_CHECK_WORK(work, error_action)				\
	if (!work) {							\
		symspi_err("no work provided");				\
		error_action;						\
	}

#define SYMSPI_GET_DEVICE_FROM_WORK(symspi_ptr, work_ptr, work_member)	\
	do {								\
		const size_t work_offset = offsetof(			\
			struct symspi_dev_private, work_member);	\
		symspi_ptr = ((struct symspi_dev_private *)		\
			     (((void *)work) - work_offset))->symspi;	\
	} while(0)

#define SYMSPI_CHECK_NOT_CLOSING(msg, closing_action)			\
	if (__symspi_is_closing(symspi)) {				\
		symspi_info_raw(SYMSPI_LOG_INFO_DBG_LEVEL		\
				, "SymSPI is closing; at %s; "msg	\
				, __func__);				\
		closing_action;						\
	}
#define SYMSPI_SWITCH_STRICT(initial, destination)			\
	symspi_switch_strict((void*)symspi				\
			     , SYMSPI_STATE_##initial			\
			     , SYMSPI_STATE_##destination)

#define SYMSPI_CHECK_DEVICE(msg, error_action)				\
	if (IS_ERR_OR_NULL(symspi)) {					\
		symspi_err("no device; "msg);				\
		error_action;						\
	}

#define SYMSPI_CHECK_PRIVATE(msg, error_action)				\
	if (!symspi->p) {						\
		symspi_err("no private part of device; "msg);		\
		error_action;						\
	}								\
	if (symspi->p->magic != SYMSPI_PRIVATE_MAGIC) {			\
		symspi_err("private part has broken magic; "msg);	\
		error_action;						\
	}

#define SYMSPI_CHECK_DEVICE_AND_PRIVATE(msg, error_action)		\
	SYMSPI_CHECK_DEVICE(msg, error_action)				\
	SYMSPI_CHECK_PRIVATE(msg, error_action)

#define SYMSPI_CHECK_STATE(expected_state, error_action)		\
	if (symspi->p->state != expected_state) {			\
		symspi_err("called not in %d state but in %d state."	\
			   , expected_state, symspi->p->state);		\
		error_action;						\
	}

#define SYMSPI_CHECK_PTR(ptr, error_action)				\
	if (IS_ERR_OR_NULL(ptr)) {					\
		symspi_err("%s: pointer "# ptr" is invalid;\n"		\
				, __func__);					\
		error_action;						\
	}

#define __symspi_error_handle(err_no, sub_error_no)			\
	__symspi_error_handle_(symspi, err_no, sub_error_no, __func__);

// Init level section
#define SYMSPI_INIT_LEVEL_PRIVATE_ALLOCATED 1
#define SYMSPI_INIT_LEVEL_XFER_CREATED 2
#define SYMSPI_INIT_LEVEL_WORKQUEUE_INIT 3
#define SYMSPI_INIT_LEVEL_GPIO_IRQS 4
#define SYMSPI_INIT_LEVEL_FULL 5

#define __SYMSPI_INIT_LEVEL(level)							\
	symspi->p->init_level = SYMSPI_INIT_LEVEL_##level;		\
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL					\
			    , "current init level: "#level);

/* -------------- BUILD TIME CONSTANTS SECTION ------------------------- */

// Should be > 0
#define SYMSPI_INITIAL_XFER_ID 1

// Symspi states

// TODO: to switch states to enum, will be more readable

// Cold and dark: SPI is not enabled, no ISR registered, etc...
#define SYMSPI_STATE_COLD 0
// Idle: waiting for the xfer request (from any Side)
#define SYMSPI_STATE_IDLE 1
// Data prepare: preparing our data to send
#define SYMSPI_STATE_XFER_PREPARE 2
// Waiting previous: waiting for the other side to finish with previous xfer
// (their flag was not yet dropped since previous xfer)
#define SYMSPI_STATE_WAITING_PREV 3
// Waiting ready: waiting for the other side to start xfer
// (waiting for their flag to be set)
//
// NOTE: this state is used **only** when SYMSPI_HARDWARE_SPI_RDY is
//          undefined and we are SPI Master
#define SYMSPI_STATE_WAITING_RDY 4
// Xfer: hardware controlled xfer
#define SYMSPI_STATE_XFER 5
// Xfer is done and we are processing its results
#define SYMSPI_STATE_POSTPROCESSING 6
// Error: error (was) detected, recovery is planned (about to be planned)
#define SYMSPI_STATE_ERROR 7


// Internal struct magic number for additional initialization verification
// (see @symspi_dev_private description)
#define SYMSPI_PRIVATE_MAGIC 0x0E31553B

// the root directory in proc file system, which contains
// SymSPI information for user space
#define SYMSPI_PROC_ROOT_NAME "symspi"
// the name of the character device to readout SymSPI info
#define SYMSPI_INFO_FILE_NAME "info"
#define SYMSPI_PROC_R_PERMISSIONS 0444

/* ------------------------ GLOBAL VARIABLES ----------------------------*/

static struct symspi_dev *symspi_global_device_ptr = NULL;

/* ----------------- FORWARD DECLARATIONS SECTION -----------------------*/

static void __symspi_error_report_init(struct symspi_dev *symspi);
static inline void __symspi_restart_timeout_timer(struct symspi_dev *symspi);
static inline void __symspi_stop_timeout_timer(struct symspi_dev *symspi);
static inline void __symspi_stop_timeout_timer_sync(struct symspi_dev *symspi);
static void __symspi_other_side_wait_timeout(struct timer_list *t);
static inline int __symspi_init_workqueue(
		const struct symspi_dev *const symspi);
static inline void __symspi_close_workqueue(
		const struct symspi_dev *const symspi);
static inline void __symspi_schedule_work(
		const struct symspi_dev *const symspi
		, struct work_struct *work);
static inline void __symspi_cancel_work_sync(
		const struct symspi_dev *const symspi
		, struct work_struct *work);
static inline bool __symspi_is_closing(struct symspi_dev *symspi);
static int symspi_idle_to_xfer_prepare_sequence(struct full_duplex_xfer *xfer
		, struct symspi_dev* symspi
		, bool force_size_change);
static void symspi_xfer_init_empty(struct full_duplex_xfer *xfer);
static int symspi_xfer_init_copy(struct full_duplex_xfer *target
		, struct full_duplex_xfer *source);
static void symspi_xfer_free(struct full_duplex_xfer *target);
static int symspi_verify_consumer_input(struct symspi_dev *symspi
		, struct full_duplex_xfer *xfer, bool check_xfer);
static int symspi_init_gpio_irqs(struct symspi_dev *symspi);
static void symspi_close_qpio_irqs(struct symspi_dev *symspi);
static int symspi_xfer_prepare_to_waiting_prev_sequence(
		struct symspi_dev *symspi);
inline static void symspi_do_update_native_spi_xfer_data(
		struct symspi_dev *symspi);
static void symspi_do_resize_xfer(struct full_duplex_xfer *xfer
		, const size_t new_size_bytes);
static inline bool symspi_is_current_xfer_ok(struct symspi_dev *symspi);
static bool regions_overlap(void *r_1, size_t size_1, void *r_2
		, size_t size_2);
inline static int symspi_replace_xfer(struct __kernel symspi_dev *symspi
		, struct __kernel full_duplex_xfer *new_xfer
		, bool force_size_change);
static int symspi_update_xfer_sequence(struct symspi_dev __kernel *symspi
		, struct __kernel full_duplex_xfer *xfer
		, int original_state
		, bool force_size_change);
inline static bool symspi_is_their_request(
		struct symspi_dev __kernel *symspi);
inline static char *symspi_get_state_ptr(void *symspi_dev);
inline static char symspi_get_state(void *symspi_dev);
inline static bool symspi_switch_strict(void *symspi_dev_ptr
		, char expected_state
		, char dst_state);
inline static char symspi_switch_state_val_forced(void *symspi_dev_ptr
		, char dst_state);
static void symspi_do_xfer_work_wrapper(struct work_struct *work);
static int symspi_do_xfer(struct symspi_dev *symspi);
static void symspi_recovery_sequence_wrapper(struct work_struct *work);
static int symspi_recovery_sequence(struct symspi_dev *symspi);
static void symspi_wait_flag_silence_period(void);
static void symspi_our_flag_set(struct symspi_dev *symspi);
static void symspi_our_flag_drop(struct symspi_dev *symspi);
inline static bool symspi_their_flag_is_set(struct symspi_dev *symspi);
static int symspi_try_leave_waiting_prev_sequence(struct symspi_dev *symspi);
static int symspi_try_leave_waiting_rdy_sequence( struct symspi_dev *symspi);
static inline int symspi_get_next_xfer_id(struct symspi_dev *symspi);
static void symspi_inc_current_xfer_counter(struct symspi_dev *symspi);
static void symspi_postprocessing_sequence(struct work_struct *work);
static int symspi_try_to_error_sequence(struct symspi_dev *symspi
					, int internal_error);
static int symspi_to_idle_sequence(struct symspi_dev *symspi
				   , const int original_state
				   , bool start_next_xfer
				   , const int internal_error);
static inline int __symspi_procfs_init(struct symspi_dev *symspi);
static inline void __symspi_procfs_close(struct symspi_dev *symspi);
static inline int __symspi_info_init(struct symspi_dev *symspi);
static void __symspi_info_close(struct symspi_dev *symspi);
static ssize_t __symspi_info_read(struct file *file
		, char __user *ubuf, size_t count, loff_t *ppos);
static void symspi_spi_xfer_done_callback(void *context);
static irqreturn_t symspi_their_flag_isr(int irq, void *symspi_device);
static void symspi_their_flag_drop_isr_sequence(struct symspi_dev *symspi);
static void symspi_their_flag_set_isr_sequence(struct symspi_dev *symspi);

#ifdef SYMSPI_DEBUG
static void symspi_xfer_printout(struct full_duplex_xfer *xfer);
static void symspi_printout(struct symspi_dev *symspi);
#endif

/* ------------------------ GLOBAL CONSTANTS ----------------------------*/

static const char SYMSPI_DRIVER_NAME[] = "symspi";

static const char SYMSPI_ERROR_S_LOGICAL[] = "";
static const char SYMSPI_ERROR_S_XFER_SIZE_MISMATCH[] = "";
static const char SYMSPI_ERROR_S_XFER_SIZE_ZERO[] = "";
static const char SYMSPI_ERROR_S_NO_MEMORY[] = "";
static const char SYMSPI_ERROR_S_OTHER_SIDE[]
		= "More than one falling edge"
		  " of 'their' flag after last xfer start.";
static const char SYMSPI_ERROR_S_STATE[] = "";
static const char SYMSPI_ERROR_S_OVERLAP[] = "";
static const char SYMSPI_ERROR_S_SPI[] = "The SPI layer resulted an error."
					 " See subsystem error code: ";
static const char SYMSPI_ERROR_S_NO_SPI[] = "";
static const char SYMSPI_ERROR_S_NO_GPIO[] = "";
static const char SYMSPI_ERROR_S_NO_XFER[] = "";
static const char SYMSPI_ERROR_S_IRQ_ACQUISITION[] = "";
static const char SYMSPI_ERROR_S_ISR_SETUP[] = "";
static const char SYMSPI_ERROR_S_WAIT_OTHER_SIDE[]
		= "Timeout waiting for other side reaction.";
#if SYMSPI_WQ_MODE_MATCH(PRIVATE)
static const char SYMSPI_ERROR_S_WORKQUEUE_INIT[]
		= "Failed to create own workqueue.";
#endif

/* --------------------- DATA STRUCTS SECTION ---------------------------*/

// Keeps the error history record
// @err_num keeps the error number which the record belongs to
// @total_count the total count of the error happened since last
//      SymSPI start
// @in_curr_report_interval_count the number of errors of @err_num
//      type happened within current report interval.
// @last_report_time_msec the msecs time when the error type was
//      last reported. If new error comes earlier than
//      @last_report_time_msec + SYMSPI_MIN_ERR_REPORT_INTERVAL_MSEC
//      then it is only put into statistics but not reported (will be
//      reported as new error of this type occured after silence time)
// @last_occurence_time_msec the ms time when the error occured
// 	last time (independent of reporting times)
// @exp_avg_interval_msec the exponentially weightened interval between
//      errors (in mseconds).
// @err_msg the error message to be sent to kernel message buffer
// @last_reported is set to true, when the last error was reported to
//      user.
// @err_per_sec_threshold sets the error rate starting from which the
// 	error is reported as error (not as warning or info). This
// 	will be used to identify the real issues like full stall of
// 	communication among occasional errors which always might happen
// 	on HW communication line.
struct symspi_error_rec {
	unsigned char err_num;
	unsigned int total_count;
	unsigned int unreported_count;
	unsigned long last_report_time_msec;
	unsigned long last_occurence_time_msec;
	unsigned long exp_avg_interval_msec;
	const char *err_msg;
	bool last_reported;
	unsigned int err_per_sec_threshold;
};

// Tracks SymSPI statistics.
// NOTE: due to performance reasons we can not introduce
// 		any locking on the statistics, thus values are expected
// 		to provide only big-picture information without pretending
// 		to have exact precision.
// @other_side_indicated_errors how many errors were indicated
// 		by the other side
// @xfers_done_ok how many raw SPI xfers were successfully finished
// @their_flag_edges how many edges of the other side flag was
//  	detected since startup
struct symspi_info {
	unsigned long long other_side_indicated_errors;
	unsigned long long other_side_no_reaction_errors;
	unsigned long long xfers_done_ok;
	unsigned long long their_flag_edges;
};

// Opaque struct which is allocated and managed by SymSPI internally.
//
// OWNERSHIP:
//      SymSPI always.
//
// @symspi keeps pointer to the wrapping symspi device structure
//      (inited by SymSPI at creation time)
// @current_xfer The data to work with upon next entering
//      SYMSPI_STATE_XFER_PREPARE state.
//      OWNERSHIP: our module
// @spi_xfer the underlying SPI device xfer data.
//      OWNERSHIP: our module
// @spi_msg the underlying SPI device message data.
//      OWNERSHIP: our module
// @next_xfer_id keeps the next xfer id.
//      should start from some positive number (@SYMSPI_INITIAL_XFER_ID).
//      If wraps starts @SYMSPI_INITIAL_XFER_ID.
// @work_queue pointer to personal SymSPI dedicated work-queue to handle
//      communication jobs. It is used mainly for stability consderations
//      cause publicitly available work-queue can potentially be blocked
//      by other running/pending tasks. So to stay on a safe side we
//      will allocate our own single-threaded workqueue for our purposes.
//      NOTE: used only when SYMSPI_WORKQUEUE_MODE equals SYMSPI_WQ_PRIVATE
// @xfer_work launches the xfer out of interrupt context.
//      Not used for now, but this might be changed due to performance
//      investigations.
// @postprocessing_work work triggers the xfer postprocessing procedures
//      (provides RX and TX data to consumer and calls consumer data
//      processing callback)
// @state the current state of SymSPI state machine.
// @their_flag_drop_counter stores number of registered (while
//      having GPIO ISR active) since last zeroing drops (from ACTIVE
//      to INACTIVE state) of other side flag.
// NOTE: their_flag_drop_counter values:
//      0   - other side didn't yet done with the previous xfer
//      1   - other side done with the previous xfer
//      >1  - other side indicates failure
// @spi_master_mode defines if we are in master mode.
//      NOTE: allows to avoid macro definitions usage over the code
//          to make it more readable
//      NOTE: ultimate intention is to make it possible to change
//          the slave and master mode at runtime (at least
//          for performance measurements)
// @hardware_spi_rdy defines if we must use SPI_RDY hardware support.
//      SPI Master side supports the SPI_RDY signal for automatic
//      hardware xfer triggering (avoiding neccessity in corresponding
//      interrupt handling).
//      In this case we never go to WAITING_RDY state, we go to XFER
//      state directly, and hardware manages all SPI_RDY sequence.
//      The value of this field is provided by spi_device->mode flag
//      (see struct spi_device for more info). In turn spi_device->mode
//      should have SPI_READY bit set only if SPI hardware supports this
//      mode.
// @their_flag_irq_number the IRQ number associated with their flag
//      set/drop interrupt. If < 0, then ISQ is not used by us
//      (latter happens only if their flag IRQ request fails and
//      we are not able to start).
// @delayed_xfer_request is set to true, when default data xfer was
//      ordered while we were not in IDLE state. Upon the xfer done,
//      start_immediately callback flag is ored with
//      delayed_xfer_request to decide if to initiate the next xfer
//      immediately.
// @close_request if set to true (this can happen only once
//      in a lifecycle (all flow from COLD to COLD state)). Triggered
//      and checked atomically.
// @final_leave_xfer_completion this completion is triggered when
//      the device switches from XFER state to any other while
//      the close_request is already issued, this completion
//      indicates that the device is ready to be closed (triggered
//      only once)
// @last_error keeps the last error code (positive), keeps its
//      value while recovery is not completed.
// @wait_timeout_timer the other side wait timeout timer,
//      it is enabled every time when we start waiting for the
//      other side action, and is disabled every time we finish
//      the waiting.
// @magic {always SYMSPI_PRIVATE_MAGIC after struct was initialized}
//      this field is used for verification that private structure
//      of symspi was actually initialized.
// @errors_history the errors history record (used to avoid flood in
//      kernel message log in case of burst of errors, say due to
//      high noise on the other side flag line; and also to track
//      overall error statistics)
// @init_level contains the SYMSPI_INIT_LEVEL_* value which tells the
//      close routine from which point do we need a cleanup started.
// @proc_root the root SymSPI directory in the proc file system
//      this directory is now aiming to provide SymSPI runtime
//      information but later might be used to set some SymSPI
//      parameters dynamically.
// @info_file the file in proc fs which provides the SymSPI
//      info to user space.
// @info_ops defines info file operations to call upon request
//      from user space
// @info tracks representation of current status of SymSPI
//      from performance POV (error statistics, data statistics)
//      and also its configuration.
//      NOTE: it is filled with 0 at each start, so keep it in such
//          a way that 0 is a correct initial value.
struct symspi_dev_private {
	struct symspi_dev *symspi;

	int next_xfer_id;
	struct full_duplex_xfer current_xfer;

	struct spi_transfer spi_xfer;
	struct spi_message spi_msg;

#if SYMSPI_WQ_MODE_MATCH(PRIVATE)
	struct workqueue_struct *work_queue;
#endif

	struct work_struct xfer_work;
	struct work_struct postprocessing_work;
	struct work_struct recover_work;

	char state;

	int their_flag_drop_counter;

	bool spi_master_mode;
	bool hardware_spi_rdy;

	int their_flag_irq_number;

	bool delayed_xfer_request;
	bool close_request;
	struct completion final_leave_xfer_completion;

	int last_error;

	struct timer_list wait_timeout_timer;

	unsigned int magic;

	struct symspi_error_rec errors[SYMSPI_ERROR_TYPES_COUNT];

	uint8_t init_level;

	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *info_file;
	struct file_operations info_ops;

	struct symspi_info info;
};


/* --------------------- KERNEL SPACE API -------------------------------*/

// API:
//      entry point, to get the symspi device.
// NOTE:
//      most probably access architecture is to be changed, or
//      at least revised.
// RETURNS:
//      valid ptr: if the global device exists
//      error ptr: if the global device was not created due to
//          errors.
__maybe_unused
struct symspi_dev *symspi_get_global_device(void)
{
	return symspi_global_device_ptr;
}


// API:
//
// Makes data xchange with given data, or with default if xfer
// is not provided
//
// If xfer is NULL and SymSPI is not in IDLE state, then the
// xfer request will be scheduled.
//
// It is guaranteed, that in case of delayed xfer, the xfer
// done callback invocation will follow after the
// symspi_data_xchange(...) call.
//
// Context: any
//
// PARAMS:
//      symspi  - {!NULL} pointer to symspi data allocated before.
//      xfer    - the new xfer data pointer.
//                It is not used after we return from the call
//                (we make a copy of the structure). But not
//                to be modified within the call.
//
//                NOTE: owned by caller.
//
//                If NULL - ignored, and current xfer is used for
//                initiated xfer.
//
//      force_size_change
//              - set this to true if we want to force resizing of the
//                xfer (use it ONLY when your higher level protocol
//                knows what it is doing). See also
//                symspi_replace_xfer(...) description.
//
// RETURNS:
//      > 0     - no error the xfer ID for new xfer was assigned, note
//                that xfer ID is written also to xfer.
//        0     - no error, no new xfer created.
//      < 0     - the negated error code, see SYMSPI_ERROR_*
//
// ERRORS:
//      FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED
//      FULL_DUPLEX_ERROR_NOT_READY
__maybe_unused
int symspi_data_xchange(void __kernel *device
			, struct __kernel full_duplex_xfer *xfer
			, bool force_size_change)
{
	struct symspi_dev __kernel *symspi = (struct symspi_dev *)device;
	SYMSPI_CHECK_NOT_CLOSING("will not invoke"
				 , return -FULL_DUPLEX_ERROR_NOT_READY);

	// will check input data correctness internally
	int res = symspi_idle_to_xfer_prepare_sequence(xfer, symspi
						       , force_size_change);
	// if we are in xfer right now
	if (res == -FULL_DUPLEX_ERROR_NOT_READY && xfer == NULL) {
		symspi->p->delayed_xfer_request = true;
		return -FULL_DUPLEX_ERROR_NOT_READY;
	}
	if (res != SYMSPI_SUCCESS) {
		return res;
	}

	// here our data preparation is done

	// TODO: it looks like SPI_RDY is not supported by our SPI iMX driver
	//          * Check in modern kernel (there is some minor suport, but
	//            seems not enough to support our workflow: fill up the buffers,
	//            then trigger xfer, but we probably can work this way)
	//          * implement/fix as possible (we can manually set SPI_RDY flag
	//            before the xfer)

	// TODO: shedule update upon xfer done if failed due to
	//      "not in IDLE state".
	res = symspi_xfer_prepare_to_waiting_prev_sequence(symspi);

	return ((res == SYMSPI_SUCCESS) && xfer) ? xfer->id : res;
}

// API:
//
// Updates our default TX data but doesn't start the xfer.
//
// RETURNS:
//      > 0     - no error the xfer ID for new xfer was assigned, note
//                that xfer ID is written also to xfer.
//        0     - no error, no new xfer created.
//      < 0     - the negated error code, see SYMSPI_ERROR_*
//
// ERRORS:
//      FULL_DUPLEX_ERROR_NOT_READY
__maybe_unused
int symspi_default_data_update(void __kernel *device
			       , struct full_duplex_xfer *xfer
			       , bool force_size_change)
{
	struct symspi_dev* symspi = (struct symspi_dev*)device;
	SYMSPI_CHECK_NOT_CLOSING("will not invoke"
				 , return -FULL_DUPLEX_ERROR_NOT_READY);

	// will check input data correctness internally
	int res = symspi_idle_to_xfer_prepare_sequence(xfer, symspi
						       , force_size_change);
	if (res != SYMSPI_SUCCESS) {
		// TODO: shedule update upon xfer done if failed due to
		//      "not in IDLE state".
		return res;
	}

	// and then back to IDLE state
	return symspi_to_idle_sequence(symspi, SYMSPI_STATE_XFER_PREPARE
				       , false, SYMSPI_SUCCESS);
}

// API:
//
// Initialize the symspi device using the provided symspi_dev
// data. The initialized SymSPI device appears in IDLE state and
// fully functional after @symspi_ini call. The consumer
// must provide the data for default xfer (as long as there is
// no one existing before).
//
// @symspi_dev {!NULL} the symspi device struct which describes the
//      configuration of SymSPI device to initialize. All fields
//      of the structure should be assigned. Consumer guarantees
//      that symspi_dev structure remains untouched between
//      @symspi_init and @symspi_close calls.
//
//      OWNERSHIP:
//          consumer
// @default_xfer {valid ptr to valid xfer with non zero data}
//      the initial default xfer to be used. Default
//      xfer data is used when xfer is started by the other side, then
//      data default xfer data will be xfered to the other side.
//      According to SymSPI protocol we need to send data as fast
//      as possible, so we will not disturb consumer trying to get
//      data update. Consumer will update data as it decides to do
//      so.
//
// CONCURRENCE: not thread safe, no other calls to symspi device are
//      allowed before symspi_init exits with success status.
//
// RETURNS:
//      0 on success
//      negated error code on failure
//
// ERRORS:
//      FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED
//      SYMSPI_ERROR_NO_MEMORY
//      SYMSPI_ERROR_IRQ_ACQUISITION
//      SYMSPI_ERROR_ISR_SETUP
//      SYMSPI_ERROR_WORKQUEUE_INIT
//
__maybe_unused
int symspi_init(void __kernel *device
		, struct full_duplex_xfer *default_xfer)
{
	struct symspi_dev *symspi = (struct symspi_dev *)device;
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "starting initialization");

	int res = symspi_verify_consumer_input(symspi, default_xfer, true);
	if (res != SYMSPI_SUCCESS) {
		symspi_err("Incorrect input. Abort.");
		return res;
	}

	// Verify if this device is already initialized
	if (!IS_ERR_OR_NULL(symspi->p)) {
		if (symspi->p->magic == SYMSPI_PRIVATE_MAGIC) {
			symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL
				    , "Given symspi instance is already"
				    " initialized. Will reuse it.");
			return SYMSPI_SUCCESS;
		}
		symspi_warning_raw("Private magic of provided private"
				   " instance missmatch, will"
				   " initialize the new device.");
	}

	symspi->p = kzalloc(sizeof(struct symspi_dev_private), GFP_KERNEL);
	if (!symspi->p) {
		symspi_err("Failed to allocate the private part of device. Abort!");
		symspi_close((void*)symspi);
		return -SYMSPI_ERROR_NO_MEMORY;
	}
	__SYMSPI_INIT_LEVEL(PRIVATE_ALLOCATED);

	// initializing the fields of symspi private
	symspi->p->magic = SYMSPI_PRIVATE_MAGIC;
	symspi->p->close_request = true;
	symspi->p->symspi = symspi;
	symspi->p->last_error = SYMSPI_SUCCESS;

	symspi->p->next_xfer_id = SYMSPI_INITIAL_XFER_ID;

	__symspi_error_report_init(symspi);

	// timeout timer
	timer_setup(&symspi->p->wait_timeout_timer,
		    __symspi_other_side_wait_timeout, 0);

	res = symspi_xfer_init_copy(&symspi->p->current_xfer, default_xfer);
	if (res < 0) {
		symspi_err("Failed to init new xfer, error: %d. Abort!", -res);
		symspi_close((void*)symspi);
		return res;
	};
	__SYMSPI_INIT_LEVEL(XFER_CREATED);
	symspi->p->current_xfer.xfers_counter = 0;
	symspi->p->current_xfer.id = symspi_get_next_xfer_id(symspi);
	default_xfer->xfers_counter = symspi->p->current_xfer.xfers_counter;
	default_xfer->id = symspi->p->current_xfer.id;

	// SPI services data initialization (see spi_write() for example)
	// The SPI transfer pointer remains always the same, as does
	// the message. We don't use queuing mechanics for messages
	// cause it doesn't fit our flow (when size and contents of
	// next xfer are in general case defined by data from previous
	// xfer).

	// init spi xfer
	memset(&symspi->p->spi_xfer, 0, sizeof(struct spi_transfer));
	symspi_do_update_native_spi_xfer_data(symspi);

	// init spi message
	spi_message_init(&symspi->p->spi_msg);
	spi_message_add_tail(&symspi->p->spi_xfer, &symspi->p->spi_msg);
	symspi->p->spi_msg.spi = symspi->spi;
	// We will call consumer callback indirectly (through the
	// work queue) from our.
	symspi->p->spi_msg.complete = &symspi_spi_xfer_done_callback;
	symspi->p->spi_msg.context = (void*)symspi;

	// init workqueue to be used
	res = __symspi_init_workqueue(symspi);
	if (res < 0) {
		symspi_err("Failed to init SymSPI private workqueue"
				   ", error: %d. Abort!", -res);
		symspi_close((void*)symspi);
		return -SYMSPI_ERROR_WORKQUEUE_INIT;
	}
	// works init
	INIT_WORK(&symspi->p->xfer_work, symspi_do_xfer_work_wrapper);
	INIT_WORK(&symspi->p->postprocessing_work, symspi_postprocessing_sequence);
	INIT_WORK(&symspi->p->recover_work, symspi_recovery_sequence_wrapper);
	__SYMSPI_INIT_LEVEL(WORKQUEUE_INIT);

	// still cold for now
	symspi->p->state = SYMSPI_STATE_COLD;

	// NOTE: to be selfconsistend with regular flow, we need to set up
	//      counter to 1 here, to assume, that other side finished with
	//      previous xfer, as long either there was no xfer before at all
	//      either it was reset after error, so previous xfer is still done.
	symspi->p->their_flag_drop_counter = 1;

	// our steady configuration
	symspi->p->spi_master_mode = SYMSPI_SPI_MASTER;
	symspi->p->hardware_spi_rdy = !!(symspi->spi->mode & SPI_READY);

	// leave xfer completion
	init_completion(&symspi->p->final_leave_xfer_completion);

	// drop our flag
	symspi_our_flag_drop(symspi);

	// init gpio IRQs
	res = symspi_init_gpio_irqs(symspi);
	if (res != SYMSPI_SUCCESS) {
		symspi_err("Failed to init SymSPI GPIO IRQs"
				   ", error: %d. Abort!", -res);
		symspi_close((void*)symspi);
		return res;
	}
	__SYMSPI_INIT_LEVEL(GPIO_IRQS);

	__symspi_procfs_init(symspi);
	__symspi_info_init(symspi);

	// Make it run. Starting from that point
	// we go to normal workflow.
	// TODO: verify close_request sequence
	__SYMSPI_INIT_LEVEL(FULL);
	symspi->p->close_request = false;
	symspi->p->state = SYMSPI_STATE_IDLE;

	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "initialization done");
#ifdef SYMSPI_DEBUG
	symspi_printout(symspi);
#endif

	// NOTE: we should not use here symspi_is_their_request(...)
	//      cause there is no previous drop of the flag as no
	//      previous xfer
	if (symspi_their_flag_is_set(symspi)) {
		symspi_data_xchange(symspi, NULL, false);
	}

	return SYMSPI_SUCCESS;
}

// API:
//
// Should be called to free all resources allocated (owned) by
// SYMSPI device. If device is not in closeable state, will wait
// for device to get out to any closeable state.
//
// CONCURRENCE: thread safe with self and other symspi calls
//      except symspi_init(...)
//
// RETURNS:
//      0: on success
//          -ENODEV: broken device data, or device data pointer
//          -EALREADY: the device is already closing
//      <0: negated error code on error
__maybe_unused
int symspi_close(void __kernel *device)
{
	struct symspi_dev *symspi = (struct symspi_dev *)device;
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("can't close;", return -ENODEV);

	bool expected_state = false;
	bool dst_state = true;
	// NOTE: this will prevent all strict state switches (EXCEPT
	//      leaving the XFER state), as well as API entries from
	//      consumer code (EXCEPT for init and reset calls)
	bool res = __atomic_compare_exchange_n(&symspi->p->close_request
			, &expected_state, dst_state, false
			, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	if (!res) {
		symspi_err("device is closing already");
		return -EALREADY;
	}
	if (symspi->p->state == SYMSPI_STATE_COLD) {
		symspi_err("device is already closed");
		return SYMSPI_SUCCESS;
	}
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "closing started");

	// we can be called with partially opened device,
	// handling this here
	switch(symspi->p->init_level) {
	case SYMSPI_INIT_LEVEL_FULL: goto full;
	case SYMSPI_INIT_LEVEL_GPIO_IRQS: goto gpio_irqs;
	case SYMSPI_INIT_LEVEL_WORKQUEUE_INIT: goto workqueue_init;
	case SYMSPI_INIT_LEVEL_XFER_CREATED: goto xfer_created;
	case SYMSPI_INIT_LEVEL_PRIVATE_ALLOCATED: goto private_allocated;
	default:
		symspi_err("unknown init level: %d. Will still try to close."
				   , symspi->p->init_level);
	}

full:
	__symspi_info_close(symspi);
	__symspi_procfs_close(symspi);

	// at this point we have
	// * prevented all entrypoints from consumer side (API)
	// * allowed only one symspi_close routine to work here
	// * blocked all state transitions except those which
	//   exit the XFER state (see symspi_switch_strict)
	//
	// meanwhile the device can be in the following states
	// at this point:
	// * closeable (can just abort everything and close):
	//   * SYMSPI_STATE_IDLE
	//   * SYMSPI_STATE_XFER_PREPARE
	//   * SYMSPI_STATE_WAITING_PREV
	//   * SYMSPI_STATE_WAITING_RDY
	//   * SYMSPI_STATE_ERROR
	// * non closeable (need to wait for hardware)
	//   * SYMSPI_STATE_XFER

	if (symspi->p->state == SYMSPI_STATE_XFER) {
		const unsigned long wait_jiffies = msecs_to_jiffies(
				  SYMSPI_CLOSE_HW_WAIT_TIMEOUT_MSEC);
		unsigned long res = wait_for_completion_timeout(
				&symspi->p->final_leave_xfer_completion
				, wait_jiffies);
		if (res <= 0) {
			symspi_err("timeout waiting for SPI xfer"
				       " to be finished, will force"
				       " abort.");

			// TODO: force SPI hardware abort if there is
			// such ability
		}
	}

	// Unless SPI transfer got frozen, we are effectively
	// stopped already at this point - no state switches allowed,
	// no API calls from this point (can be called but they do
	// nothing), IRQs are also not doing anything.

	__SYMSPI_INIT_LEVEL(GPIO_IRQS);

gpio_irqs:
	// remove ISRs, this also disables external error trigger
	symspi_close_qpio_irqs(symspi);

	symspi_our_flag_drop(symspi);

	// close waiting timer
	__symspi_stop_timeout_timer_sync(symspi);

	// No one can leave this state except init(), which should
	// not be called by contract
	if (symspi_switch_state_val_forced(symspi, SYMSPI_STATE_COLD)
			== SYMSPI_STATE_COLD) {
		symspi_warning("On closing the device was already in"
			       " COLD state.");
	}

	// Unless SPI transfer got frozen,
	// only workqueue queued works can bother us from this
	// point, however, even those do nothing.

	__SYMSPI_INIT_LEVEL(WORKQUEUE_INIT);

workqueue_init:
	// close async works
	__symspi_cancel_work_sync(symspi, &symspi->p->xfer_work);
	__symspi_cancel_work_sync(symspi, &symspi->p->postprocessing_work);
	__symspi_cancel_work_sync(symspi, &symspi->p->recover_work);

	// wrap up with used workqueue
	__symspi_close_workqueue(symspi);

	__SYMSPI_INIT_LEVEL(XFER_CREATED);

	// Unless SPI transfer stalled, all active entities are
	// dead by now, and we can run raw resources cleanup.

xfer_created:
	// Spi_message doesn't point to any resources which
	// are needed to be freed by us. SPI device is owned
	// by consumer.
	memset(&symspi->p->spi_msg, 0, sizeof(struct spi_message));

	// spi xfer and symspi xfer point on the same data,
	// so we free only once
	symspi_xfer_free(&symspi->p->current_xfer);
	symspi_do_update_native_spi_xfer_data(symspi);

	__SYMSPI_INIT_LEVEL(PRIVATE_ALLOCATED);

private_allocated:
	// returning symspi_dev to original state
	symspi->p->magic = 0;
	kfree(symspi->p);
	symspi->p = NULL;

	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "closing done");

	return SYMSPI_SUCCESS;
}

__maybe_unused
bool symspi_is_running(void __kernel *device)
{
	struct symspi_dev *symspi = (struct symspi_dev *) device;
	if (IS_ERR_OR_NULL(symspi) || IS_ERR_OR_NULL(symspi->p)) {
		return false;
	}
	// TODO: to update as mutex on init is done is used
	return symspi->p->state != SYMSPI_STATE_COLD;
}

// API:
//
// Restarts the interface. Should be called if SPI level
// errors were encountered.
// PARAMS:
//      @default_xfer {valid xfer ptr || NULL when current_xfer is OK}
//          defines the default xfer to be used. Can be NULL, then
//          we'll try to use existing current xfer (if one).
//      @symspi {valid ptr to the symspi dev to be reset}
//          the symspi device can be in any state (totaly uninited,
//          COLD, IDLE,...)
//
// CONCURRENCE: not thread safe, no other calls to symspi device are
//      allowed before symspi_init exits with success status.
//
// RETURNS:
//      see @symspi_init
__maybe_unused
int symspi_reset(void __kernel *device
		 , struct full_duplex_xfer *default_xfer)
{
	struct symspi_dev *symspi = (struct symspi_dev *)device;
	struct full_duplex_xfer tmp_xfer;
	if (symspi_is_current_xfer_ok(symspi) && !default_xfer) {
		int res = symspi_xfer_init_copy(&tmp_xfer
										, &symspi->p->current_xfer);
		if (res < 0) {
			symspi_err("Failed to init xfer, error: %d. Abort!"
						, -res);
			return res;
		}
		default_xfer = &tmp_xfer;
	}

	int res;
	res = symspi_verify_consumer_input(symspi, default_xfer, true);
	if (res != SYMSPI_SUCCESS) {
		symspi_err("Incorrect input. Abort.");
		return res;
	}

	symspi_close((void*)symspi);

	return symspi_init((void*)symspi, default_xfer);
}

const struct full_duplex_sym_iface symspi_full_duplex_iface = {
	.data_xchange = &symspi_data_xchange
	, .default_data_update = &symspi_default_data_update
	, .is_running = &symspi_is_running
	, .init = &symspi_init
	, .reset = &symspi_reset
	, .close = &symspi_close
};

// API
//
// Returns ptr to the full duplex device interface object.
//
// RETURNS:
//      valid ptr to struct full_duplex_sym_iface with all
//      fields filled
__maybe_unused
const struct full_duplex_sym_iface *symspi_iface(void)
{
	return &symspi_full_duplex_iface;
}

/* ------------------------- MAIN SECTION -------------------------------*/

// Helper.
// Initializes the error report array.
static void __symspi_error_report_init(struct symspi_dev *symspi)
{
	memset(symspi->p->errors, 0, sizeof(symspi->p->errors));

#define SYMSPI_ERR_REC(idx, ERR_NAME, threshold_err_per_sec)		\
	symspi->p->errors[idx].err_num = SYMSPI_ERROR_##ERR_NAME;	\
	symspi->p->errors[idx].err_msg					\
		= (const char*)&SYMSPI_ERROR_S_##ERR_NAME;		\
	symspi->p->errors[idx].err_per_sec_threshold			\
		= threshold_err_per_sec;

	SYMSPI_ERR_REC(0, LOGICAL, 0);
	SYMSPI_ERR_REC(1, XFER_SIZE_MISMATCH, 0);
	SYMSPI_ERR_REC(2, XFER_SIZE_ZERO, 0);
	SYMSPI_ERR_REC(3, NO_MEMORY, 0);
	SYMSPI_ERR_REC(4, OTHER_SIDE, 5);
	SYMSPI_ERR_REC(5, STATE, 0);
	SYMSPI_ERR_REC(6, OVERLAP, 0);
	SYMSPI_ERR_REC(7, SPI, 0);
	SYMSPI_ERR_REC(8, NO_SPI, 0);
	SYMSPI_ERR_REC(9, NO_GPIO, 0);
	SYMSPI_ERR_REC(10, NO_XFER, 0);
	SYMSPI_ERR_REC(11, IRQ_ACQUISITION, 0);
	SYMSPI_ERR_REC(12, ISR_SETUP, 0);
	SYMSPI_ERR_REC(13, WAIT_OTHER_SIDE, 5);
#if SYMSPI_WQ_MODE_MATCH(PRIVATE)
	SYMSPI_ERR_REC(14, WORKQUEUE_INIT, 0);
#endif

#undef SYMSPI_ERR_REC
}

// Helper.
// Returns the error record pointer given by error number
//
// @symspi {valid ptr to symspi device}
// @err_no {the valid error number to report}
static inline struct symspi_error_rec *__symspi_get_error_rec(
		struct symspi_dev *symspi, unsigned char err_no)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("no device", return NULL);
	for (int i = 0; i < ARRAY_SIZE(symspi->p->errors); ++i) {
		if (symspi->p->errors[i].err_num == err_no) {
			return &(symspi->p->errors[i]);
		}
	}
	return NULL;
}

// Helper.
// Reports error to the kernel log and also tracks the history of errors
// and protects kernel log from error messages flood in case of external
// errors triggering.
//
// @symspi {valid ptr to symspi device}
// @err_no {the valid error number to report}
// @sub_error_no subsystem error number (might be used to pass
//      subsystem error code)
// @func_name {NULL || valid string pointer} function name where
//      the error was raised
//
// RETURNS:
//      true: if it is OK to be verbose now
//      false: else (silence required)
static bool __symspi_error_report(struct symspi_dev *symspi
				  , unsigned char err_no
				  , int sub_error_no
				  , const char *func_name)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("no device", return true);
	struct symspi_error_rec *e_ptr = __symspi_get_error_rec(symspi
								, err_no);
	if (e_ptr == NULL) {
		symspi_err("unknown error type given: %u" , err_no);
		return true;
	}
	// NOTE: wraps every ~24 hours
	const uint32_t now_msec = (uint32_t)(ktime_divns(ktime_get(), 1000000U));

	e_ptr->total_count++;
	const unsigned int since_last_report_msec
			= (now_msec >= e_ptr->last_report_time_msec)
			  ? (now_msec - e_ptr->last_report_time_msec)
			  : (e_ptr->last_report_time_msec - now_msec);
	const unsigned int since_last_occurance_msec
			= (now_msec >= e_ptr->last_occurence_time_msec)
			  ? (now_msec - e_ptr->last_occurence_time_msec)
			  : (e_ptr->last_occurence_time_msec - now_msec);
	e_ptr->last_occurence_time_msec = now_msec;

	// approximately calculating the decay rate at this time point
	// surely it will not be exactly the exp decay, but will resemble
	// the general behaviour
	const unsigned int decay_percent
		= max(min((unsigned int)((50 * since_last_occurance_msec)
					 / SYMSPI_ERR_RATE_DECAY_RATE_MSEC_PER_HALF)
		      , (unsigned int)100), (unsigned int)SYMSPI_ERR_RATE_DECAY_RATE_MIN);
	const unsigned int threshold = e_ptr->err_per_sec_threshold;
	const unsigned int prev_rate
	    	= 1000 / max((unsigned int)(e_ptr->exp_avg_interval_msec), 1U);

	e_ptr->exp_avg_interval_msec
		= max((unsigned int)(((100 - decay_percent) * e_ptr->exp_avg_interval_msec
		      		      + decay_percent * since_last_occurance_msec) / 100)
		      , 1U);

	const unsigned int rate = 1000 / e_ptr->exp_avg_interval_msec;

#ifdef SYMSPI_DEBUG
	symspi_err_raw("====== error %d ======", err_no);
	symspi_err_raw("diff interval: %u", since_last_occurance_msec);
	symspi_err_raw("decay percent: %u", decay_percent);
	symspi_err_raw("new avg interval: %lu", e_ptr->exp_avg_interval_msec);
	symspi_err_raw("rate_prev = %u", prev_rate);
	symspi_err_raw("rate = %u", rate);
#endif

	if (since_last_report_msec < SYMSPI_MIN_ERR_REPORT_INTERVAL_MSEC
			&& !(prev_rate < threshold && rate >= threshold)) {
		e_ptr->unreported_count++;
		e_ptr->last_reported = false;
		return false;
	}

	e_ptr->last_report_time_msec = now_msec;
	e_ptr->last_reported = true;

	static const char *const level_err = "error";
	static const char *const level_warn = "warning";
	const char *const report_class_str = (rate >= threshold)
	                                     ? level_err : level_warn;

	if (func_name) {
		symspi_err_raw("SymSPI %s %u (ARpS: %d): "
			       "%s (sub %s: %d), by %s"
			       , report_class_str, err_no
			       , rate, e_ptr->err_msg
			       , report_class_str, sub_error_no, func_name);
	} else {
		symspi_err_raw("SymSPI %s %u (ARpS: %d): "
			       "%s (sub %s: %d)"
			       , report_class_str, err_no
			       , rate, e_ptr->err_msg
			       , report_class_str, sub_error_no);
	}

	if (e_ptr->unreported_count > 0) {
		symspi_err_raw("%s %d -> %d since "
			       " %u msecs. Total: %u."
			       , report_class_str, err_no
			       , e_ptr->unreported_count
			       , since_last_report_msec
			       , e_ptr->total_count);
		e_ptr->unreported_count = 0;
	}

	return true;
}

// Helper.
// Runs the error handling procedure (report an error, and then launch
// error recovery).
//
// @symspi {valid ptr to symspi device}
// @err_no {the valid error number to report && !SYMSPI_SUCCESS}
// @sub_error_no subsystem error number (might be used to pass
//      subsystem error code)
// @func_name {NULL || valid string pointer} function name where
//      the error was raised
//
// CONTEXT:
// 		can not sleep (must be callable from ISR context)
static void __symspi_error_handle_(struct symspi_dev *symspi
				  , unsigned char err_no
				  , int sub_error_no
				  , const char *func_name)
{
	if (err_no == SYMSPI_SUCCESS) {
		symspi_err("Error handling triggered without error no.");
		return;
	}

	// Update info/error statistics
	if (err_no == SYMSPI_ERROR_OTHER_SIDE) {
		symspi->p->info.other_side_indicated_errors += 1;
	} else if (err_no == SYMSPI_ERROR_WAIT_OTHER_SIDE) {
		symspi->p->info.other_side_no_reaction_errors += 1;
	}

	bool report = __symspi_error_report(symspi, err_no, sub_error_no
					    , func_name);

	// NOTE: if error happened while we are in XFER state, we will
	//      wait until SPI layer ends its xfer and returns with callback
	//      and only then will goto ERROR state. Aborting SPI can be
	//      tricky and doesn't worth it.
	//
	// can goto ERROR: directly
	//      SYMSPI_STATE_IDLE
	//      SYMSPI_STATE_XFER_PREPARE
	//      SYMSPI_STATE_WAITING_PREV
	//      SYMSPI_STATE_WAITING_RDY
	//      SYMSPI_STATE_POSTPROCESSING
	//
	// can't goto ERROR: nop
	//      SYMSPI_STATE_COLD
	//      SYMSPI_STATE_ERROR
	//
	// move to error pending state:
	//      SYMSPI_STATE_XFER


	// TODO: instead of making the loop here, we can do the following
	//      states are represented as bits in a state variable.
	//      like:   COLD:  -------1
	//              IDLE:  ------1-
	//              XFER:  -----1--
	//              ...............
	//              ERRO:  --------
	//      and then we make an atomic:   __atomic_fetch_and(...)
	//      operation with the states we want to drop, say with:
	//                     1111---1
	//      which will atomically drop any state which we
	//      want to drop, and provide us original state value.
	//      And if state is dropped it goes automatically into
	//      the error state, so that the rest of the code
	//      functions by old means of __atomic_compare_exchange(...)
	//
	//      As a result: the masked states are not affected and
	//          identified, and unmasked states are triggered to the
	//          error state and the original state is saved
	//
	// NOTE: thanks Peter for the hint for the idea =)


	// NOTE:
	//      in this case time efficiency is not really relevant
	//      so we can spin a bit here. Other solutions will
	//      require additional locking over the normal workflow
	//      to be robust. While loop is needed cause in corner
	//      case the rest of the SymSPI can switch the state
	//      just within our comparison sequence, and thus we
	//      may fail to detect any state.
	// TODO: extract body into separate function
	while (true) {
		if (SYMSPI_SWITCH_STRICT(IDLE, ERROR)
				|| SYMSPI_SWITCH_STRICT(XFER_PREPARE, ERROR)
				|| SYMSPI_SWITCH_STRICT(WAITING_PREV, ERROR)
				|| SYMSPI_SWITCH_STRICT(WAITING_RDY, ERROR)
				|| SYMSPI_SWITCH_STRICT(POSTPROCESSING, ERROR)) {
			if (report) {
				symspi_warning_raw("Sheduling recovery.");
			}
			symspi->p->last_error = err_no;
			// not a direct call cause recovery should discard and wait
			// for completion of the timer, which will cause the
			// softlock if is called from timer handler
			__symspi_schedule_work(symspi, &symspi->p->recover_work);
			return;
		}

		if (SYMSPI_SWITCH_STRICT(ERROR, ERROR)) {
			return;
		}

		// then we are in SYMSPI_STATE_XFER state
		if (SYMSPI_SWITCH_STRICT(XFER, XFER)) {
			symspi->p->last_error = err_no;
			symspi_info_raw(SYMSPI_LOG_INFO_DBG_LEVEL
					, "recovery postponed till"
					  " spi xfer is complete");
			// Secondary call is needed due to possible races
			// with symspi_spi_xfer_done_callback
			if (SYMSPI_SWITCH_STRICT(POSTPROCESSING, ERROR)) {
				__symspi_schedule_work(symspi, &symspi->p->recover_work);
			}
			return;
		}

		// in these states we do nothing (new) about error handling
		if (SYMSPI_SWITCH_STRICT(COLD, COLD)) {
			return;
		}
	}
}

// Helper.
// Starts/restarts timeout timer
//
// CONTEXT:
//      any
static inline void __symspi_restart_timeout_timer(struct symspi_dev *symspi)
{
	const unsigned long expiration_time_jf
		= jiffies + msecs_to_jiffies(SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC);
	mod_timer(&symspi->p->wait_timeout_timer, expiration_time_jf);
	symspi_trace(
		    "timer set: in %d ms, in %lu jiffies"
		    " (at %lu jiffies), timer: %px, now: %lu jiffies"
		    , SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC
		    , expiration_time_jf >= jiffies ? (expiration_time_jf - jiffies) : 0
		    , expiration_time_jf
		    , &symspi->p->wait_timeout_timer
		    , jiffies);
	if (timer_pending(&symspi->p->wait_timeout_timer)) {
		symspi_trace(
			    "timer status: pending at %lu jiffies, now: %lu jiffies"
			    , symspi->p->wait_timeout_timer.expires
			    , jiffies);
	} else {
		symspi_trace("timer status: idle");
	};
}

// Helper.
// Stops timeout timer. If timer function executes doesn't wait for them.
//
// CONTEXT:
//      any
static inline void __symspi_stop_timeout_timer(struct symspi_dev *symspi)
{
	del_timer(&symspi->p->wait_timeout_timer);
	symspi_trace("Timer stop");
}

// Helper.
// Stops timeout timer and waits for all timer functions executions
// are done.
//
// CONTEXT:
//      sleepable
static inline void __symspi_stop_timeout_timer_sync(struct symspi_dev *symspi)
{
	del_timer_sync(&symspi->p->wait_timeout_timer);
	symspi_trace("Timer stop (sync)");
}


// Launches error recovery on timeout
static void __symspi_other_side_wait_timeout(struct timer_list *t)
{
	struct symspi_dev_private *priv = from_timer(priv, t, wait_timeout_timer);
	struct symspi_dev *symspi = container_of(&priv, struct symspi_dev, p);

	SYMSPI_CHECK_DEVICE_AND_PRIVATE("No device provided for recovery."
					, return);
	__symspi_error_handle(SYMSPI_ERROR_WAIT_OTHER_SIDE, 0);
}

// Helper.
// Inits the workqueue which is to be used by SymSPI
// in its current configuration. If we use system-provided
// workqueu - does nothing.
//
// RETURNS:
//      >= 0     - on success
//      < 0     - negative error code
//
// ERRORS:
//      SYMSPI_ERROR_WORKQUEUE_INIT
static inline int __symspi_init_workqueue(
		const struct symspi_dev *const symspi)
{
#if SYMSPI_WQ_MODE_MATCH(SYSTEM)
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "using system wq");
	(void)symspi;
	return SYMSPI_SUCCESS;
#elif SYMSPI_WQ_MODE_MATCH(SYSTEM_HIGHPRI)
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "using system_highpri wq");
	(void)symspi;
	return SYMSPI_SUCCESS;
#elif SYMSPI_WQ_MODE_MATCH(PRIVATE)
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "using private wq");
	symspi->p->work_queue = alloc_workqueue("symspi", WQ_HIGHPRI, 0);

	if (symspi->p->work_queue) {
		return SYMSPI_SUCCESS;
	} else {
		symspi_err("%s: the private work queue init failed."
				   , __func__);
		return -SYMSPI_ERROR_WORKQUEUE_INIT;
	}
#endif
}

// Helper.
// Closes the workqueue which was used by SymSPI
// in its current configuration. If we use system-provided
// workqueue - does nothing.
static inline void __symspi_close_workqueue(
		const struct symspi_dev *const symspi)
{
#if SYMSPI_WQ_MODE_MATCH(PRIVATE)
	destroy_workqueue(symspi->p->work_queue);
	symspi->p->work_queue = NULL;
#else
	(void)symspi;
#endif
}


// Helper.
// Wrapper over schedule_work(...) for queue selected by configuration.
// Schedules SymSPI work to the target queue.
static inline void __symspi_schedule_work(
		const struct symspi_dev *const symspi
		, struct work_struct *work)
{
#if SYMSPI_WQ_MODE_MATCH(SYSTEM)
	(void)symspi;
	schedule_work(work);
#elif SYMSPI_WQ_MODE_MATCH(SYSTEM_HIGHPRI)
	(void)symspi;
	queue_work(system_highpri_wq, work);
#elif SYMSPI_WQ_MODE_MATCH(PRIVATE)
	queue_work(symspi->p->work_queue, work);
#else
#error no known SymSPI work queue mode defined
#endif
}

// Helper.
// Wrapper over cancel_work_sync(...) in case we will
// need some custom queue operations on cancelling.
static inline void __symspi_cancel_work_sync(
		const struct symspi_dev *const symspi
		, struct work_struct *work)
{
	cancel_work_sync(work);
}

// Helper.
// RETURNS:
//      true: if device was asked to be closed
//      false: else
static inline bool __symspi_is_closing(struct symspi_dev *symspi)
{
	return symspi->p->close_request;
}

// Starts from IDLE state. Updates our default TX data.
// Leaves system in XFER_PREPARE state.
//
// CONTEXT:
//      sleepable
//
// NOTE: doesn't start the xfer
// NOTE: doesn't return to IDLE state
// NOTE: handles consumer data checks
static int symspi_idle_to_xfer_prepare_sequence(struct full_duplex_xfer *xfer
						, struct symspi_dev* symspi
						, bool force_size_change)
{
	int res = symspi_verify_consumer_input(symspi, xfer, xfer != NULL);
	if (res != SYMSPI_SUCCESS) {
		symspi_err("Incorrect input. Abort.");
		return res;
	}

	if (!symspi_switch_strict((void*)symspi,
			SYMSPI_STATE_IDLE, SYMSPI_STATE_XFER_PREPARE)) {
		symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
			    , "Xfer request while not in IDLE state."
			    " Will enqueue request.");
		return -FULL_DUPLEX_ERROR_NOT_READY;
	}

	res = symspi_try_to_error_sequence(symspi, SYMSPI_SUCCESS);
	if (res != SYMSPI_SUCCESS) {
		return res;
	}

	// if we reached this point, then we control the workflow and no one
	// except us will change / read current xfer data even if our interrupts
	// happen. The only thing which can be changed is their flag drop
	// counter (and switching to ERROR state : TODO verify ERROR state switch
	// status).

	return symspi_update_xfer_sequence(symspi, xfer
			, SYMSPI_STATE_XFER_PREPARE, force_size_change);
}

// Helper function. No checks thus. Initializes the empty xfer.
// Empty xfer is to to be used by device.
static void symspi_xfer_init_empty(struct full_duplex_xfer *xfer)
{
	memset(xfer, 0, sizeof(struct full_duplex_xfer));
	xfer->id = SYMSPI_INITIAL_XFER_ID;
}

// Helper function. No checks thus. Replaces the empty xfer data
// with data from provided one. Both @target and @source should
// be allocated. @source should be valid (which also means not
// empty) xfer.
static int symspi_xfer_init_copy(struct full_duplex_xfer *target
				 , struct full_duplex_xfer *source)
{
#ifdef SYMSPI_DEBUG
	if (!target) {
		symspi_err("no target");
		return -SYMSPI_ERROR_LOGICAL;
	}
	if (!source) {
		symspi_err("no source");
		return -SYMSPI_ERROR_LOGICAL;
	}
	if (source->size_bytes == 0 || source->data_tx == NULL) {
		symspi_err("source empty");
		return -SYMSPI_ERROR_LOGICAL;
	}
#endif
	symspi_xfer_init_empty(target);
	symspi_do_resize_xfer(target, source->size_bytes);
	if (target->size_bytes == 0) {
		symspi_err("No memory for new xfer.");
		return -SYMSPI_ERROR_NO_MEMORY;
	}
	memcpy(target->data_tx, source->data_tx, source->size_bytes);
	target->done_callback = source->done_callback;
	target->consumer_data = source->consumer_data;
	return SYMSPI_SUCCESS;
}

// Helper function.
static void symspi_xfer_free(struct full_duplex_xfer *target)
{
	symspi_do_resize_xfer(target, 0);
	target->done_callback = NULL;
	target->consumer_data = NULL;
}

#ifdef SYMSPI_DEBUG
static void symspi_xfer_printout(struct full_duplex_xfer *xfer)
{
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "ptr: %px", xfer);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "size: %d", (int)xfer->size_bytes);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "TX buf: %px", xfer->data_tx);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "RX buf: %px", xfer->data_rx_buf);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "xfers counter: %d", xfer->xfers_counter);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "id: %d", xfer->id);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "consumer data: %px", xfer->consumer_data);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "done callback: %px", xfer->done_callback);
}

static void symspi_printout(struct symspi_dev *symspi)
{
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "=========");
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "symspi device: %px", symspi);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "symspi device private: %px", symspi->p);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "spi device: %px", symspi->spi);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "their flag gpio device: %px", symspi->gpiod_their_flag);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "our flag gpio device: %px", symspi->gpiod_our_flag);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "xfer accepted callback: %px"
		    , symspi->xfer_accepted_callback);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "symspi state: %d", (int)symspi->p->state);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "their flag drop counter: %d"
		    , symspi->p->their_flag_drop_counter);
	if (symspi->p->spi_master_mode) {
		symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "master mode");
	} else {
		symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "slave mode");
	}
	if (symspi->p->hardware_spi_rdy) {
		symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
			    , "using hardware SPI RDY");
	} else {
		symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "software SPI RDY");
	}
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "default xfer:");
	symspi_xfer_printout(&symspi->p->current_xfer);
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL, "=========");
}
#endif

// Helper function. Verifies the consumer input data for
// symspi init procedure.
// RETURNS:
//      SYMSPI_SUCCESS if data is correct
//      negaged error code if data is not correct
static int symspi_verify_consumer_input(struct symspi_dev *symspi
					, struct full_duplex_xfer *xfer
					, bool check_xfer)
{
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED);
	if (IS_ERR_OR_NULL(symspi->spi)) {
		symspi_err("%s: No SPI device provided.", __func__);
		return -SYMSPI_ERROR_NO_SPI;
	}
	if (IS_ERR_OR_NULL(symspi->gpiod_our_flag)
			|| IS_ERR_OR_NULL(symspi->gpiod_their_flag)) {
		symspi_err("%s: No GPIO device provided.", __func__);
		return -SYMSPI_ERROR_NO_GPIO;
	}

	if (!check_xfer) {
		return SYMSPI_SUCCESS;
	}

	if (IS_ERR_OR_NULL(xfer)) {
		symspi_err("%s: No default xfer.", __func__);
		return -SYMSPI_ERROR_NO_XFER;
	}
	if (xfer->size_bytes == 0) {
		symspi_err("%s: Zero size default xfer.", __func__);
		return -SYMSPI_ERROR_NO_XFER;
	}
	if (IS_ERR_OR_NULL(xfer->data_tx)) {
		symspi_err("%s: Default xfer no TX data.", __func__);
		return -SYMSPI_ERROR_NO_XFER;
	}

	return SYMSPI_SUCCESS;
}

// Helper function. Called from main init routine. Sets up the IRQs
// and ISRs for our GPIOs. All necessary data from symspi_dev should
// be already filled up.
static int symspi_init_gpio_irqs(struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("nothing to initialize"
					, return -SYMSPI_ERROR_LOGICAL);
	if (IS_ERR(symspi->gpiod_their_flag)) {
		symspi_err("%s: no gpio device for their flag.", __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
#endif

	int irqnr = gpiod_to_irq(symspi->gpiod_their_flag);
	if (irqnr < 0) {
		symspi_err("%s: could not allocate GPIO IRQ number."
			   "Underlying error: %d", __func__, irqnr);
		return -SYMSPI_ERROR_IRQ_ACQUISITION;
	}
	symspi->p->their_flag_irq_number = irqnr;

	// if SPI hardware handles the SPI ready signal automatically
	// then we don't need to handle
	// TODO: FIXME: correlate between ACTIVE state and falling/rising edge
	// TODO: FIXME: in case of SPI_READY active if we are SPI master
	//      take their flag configuration from SPI device SPI_RDY line
	//      configuration. Now they should be manually be equally
	//      configured by consumer.
	unsigned long irq_flags = IRQF_TRIGGER_FALLING;
	irq_flags |= (symspi->p->hardware_spi_rdy ? 0 : IRQF_TRIGGER_RISING);

	int ret = request_irq(irqnr, symspi_their_flag_isr, irq_flags
			      , SYMSPI_DRIVER_NAME, symspi);
	if (ret < 0) {
		symspi_err("%s: setup ISR failed, underlying error: %d\n"
			   , __func__, ret);

		symspi->p->their_flag_irq_number = ret;
		return -SYMSPI_ERROR_ISR_SETUP;
	}
	return SYMSPI_SUCCESS;
}

// Helper function. Called from main close routine. Removes our GPIO ISRs.
static void symspi_close_qpio_irqs(struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("nothing to unset", return);
#endif

	if (symspi->p->their_flag_irq_number < 0) {
		symspi_warning("%s: ISR was not installed. Nothing"
			       " to remove.", __func__);
		return;
	}

	free_irq(symspi->p->their_flag_irq_number, symspi);
	symspi->p->their_flag_irq_number = -ENOLINK;

	return;
}


// Sequence of operations to leave the SYMSPI_STATE_XFER_PREPARE state.
//
// CONTEXT:
//      any
//
// STATE:
//      SYMSPI_STATE_XFER_PREPARE -> SYMSPI_STATE_WAITING_PREV
//
// ERRORS:
//      SYMSPI_SUCCESS
//
// DBG ERRORS:
//      SYMSPI_ERROR_LOGICAL
static int symspi_xfer_prepare_to_waiting_prev_sequence(
		struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -SYMSPI_ERROR_LOGICAL);
	SYMSPI_CHECK_STATE(SYMSPI_STATE_XFER_PREPARE
			    , return -SYMSPI_ERROR_LOGICAL);
#endif

	// TODO: Consider optimization to set the flag earlier to give other side
	// more time to prepare for xfer.
	symspi_our_flag_set(symspi);

	// as long as xfer prepare stage can last for relatively long time,
	// then it is good to check again the counter status here
	// just to not to goto xfer if there is an error have already
	// happened

	int res = symspi_try_to_error_sequence(symspi, SYMSPI_SUCCESS);
	if (res != SYMSPI_SUCCESS) {
		return res;
	}

	if (symspi_switch_strict((void*)symspi
				 , SYMSPI_STATE_XFER_PREPARE
				 , SYMSPI_STATE_WAITING_PREV)) {
		__symspi_restart_timeout_timer(symspi);
	}

	// note, spi slave will bypass waiting prev state
	// immediately to xfer state
	if (symspi->p->their_flag_drop_counter == 1
			|| !symspi->p->spi_master_mode) {
		return symspi_try_leave_waiting_prev_sequence(symspi);
	}

	return SYMSPI_SUCCESS;
}


// Helper function. Updates underlying SPI layer transfer data
// from our current xfer
inline static void symspi_do_update_native_spi_xfer_data(
		struct symspi_dev *symspi)
{
	struct spi_transfer *dst = &symspi->p->spi_xfer;
	struct full_duplex_xfer *src = &symspi->p->current_xfer;

	// this shall configure the transport level details
	// of the transfer if it is needed
	if (symspi->native_transfer_configuration_hook) {
		symspi->native_transfer_configuration_hook(
				    src, dst, sizeof(*dst));
	}

	dst->tx_buf = src->data_tx;
	dst->rx_buf = src->data_rx_buf;
	dst->len = src->size_bytes;
}


// Helper function ('do_'). Thus no checks.
//
// Resizes our xfer data.
// If resize fails, frees xfer data and sets it's size to 0.
//
// PARAMS:
//      @xfer {valid pointer to selfconsistent xfer} xfer
//          might be empty, and then all its internal pointers
//          should be NULL, and @size_bytes should be 0.
//      @new_size_bytes { >= 0 } new xfer data size in bytes
//          (size of one buffer)
//
// TODO: to implement growing-only allocation mode for the xfer
//          to reduce real allocations amount and increase speed.
//          Other option might be just pointers switching to
//          switch the current xfer.
// TODO: move all xfer accociated methods to the full_duplex_interface
//      and make them globally accessible
static void symspi_do_resize_xfer(struct full_duplex_xfer *xfer
				  , const size_t new_size_bytes)
{
	if (xfer->size_bytes == new_size_bytes) {
		return;
	}

	xfer->size_bytes = new_size_bytes;

	if (new_size_bytes == 0) {
		goto out_of_memory;
	}

	// reallocating current xfer buffers
	if (!(xfer->data_tx = krealloc(xfer->data_tx
				       , new_size_bytes
				       , GFP_KERNEL))) {
		goto out_of_memory;
	}
	if (!(xfer->data_rx_buf = krealloc(xfer->data_rx_buf
					   , new_size_bytes
					   , GFP_KERNEL))) {
		goto out_of_memory;
	}

	return;

out_of_memory:
	kfree(xfer->data_tx);
	kfree(xfer->data_rx_buf);
	xfer->size_bytes = 0;

	return;
}

// Helper.
// RETURNS:
//      true if current default xfer exists and is not empty,
//      else false.
static inline bool symspi_is_current_xfer_ok(struct symspi_dev *symspi)
{
	return symspi && symspi->p
		      && symspi->p->current_xfer.size_bytes
		      && symspi->p->current_xfer.data_tx
		      && symspi->p->current_xfer.data_rx_buf;
}

// Helper function, checks if memory regions overlap
//
// RETURNS:
//      true - if overlap, fals - else
static inline bool regions_overlap(void *r_1, size_t size_1
		, void *r_2, size_t size_2)
{
	return (r_2 < r_1 + size_1) && (r_2 + size_2 - 1 >= r_1);
}

// Replaces our current xfer with newly provided one
// (including the underlying SPI transfer data).
//
// NOTE:
//      No one is assumed to using the current_xfer data
//      at the moment of execution of this function.
// NOTE:
//      data_rx_buf is not zeroed, to not to waste
//      resourses
//
// PARAMS:
//      symspi
//              - {!NULL} pointer to symspi device
//      new_xfer
//              - {! NULL} pointer to consumer owned new xfer structure
//                (internally pointed data is also owned by consumer)
//      force_size_change
//              - set this to true if we want to force resizing of the
//                xfer (use it only when your higher level protocol
//                knows what it is doing)
//
// STATE:
//      SYMSPI_STATE_XFER_PREPARE
//              - when consumer updates data
//      SYMSPI_STATE_XFER
//              - when xfer was just completed and consumer
//                updates xfer (only here xfer length change
//                is possible)
//      SYMSPI_STATE_ERROR
//              - when xfer should be updated to recover from error
//
// RETURNS:
//      >= 0     - on success
//      < 0     - negative error code
//
// ERRORS:
//      SYMSPI_ERROR_XFER_SIZE_ZERO
//      SYMSPI_ERROR_XFER_SIZE_MISMATCH
//      SYMSPI_ERROR_NO_MEMORY
//      SYMSPI_ERROR_OVERLAP
//
// DBG ERRORS:
//      SYMSPI_ERROR_LOGICAL
//      FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED
//
inline static int symspi_replace_xfer(
		struct __kernel symspi_dev *symspi
		, struct __kernel full_duplex_xfer *new_xfer
		, bool force_size_change)
{
#ifdef SYMSPI_DEBUG
	// assertions
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED);
	if (!new_xfer) {
		symspi_err("%s: zero ptr to new xfer.", __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
	if (symspi->p->state != SYMSPI_STATE_XFER_PREPARE
			&& symspi->p->state != SYMSPI_STATE_XFER
			&& symspi->p->state != SYMSPI_STATE_ERROR) {
		symspi_err("%s: was executed while not in XFER_PREPARE"
			   " or XFER or ERROR state.", __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
#endif
	struct full_duplex_xfer *curr_xfer = &(symspi->p->current_xfer);

	if (new_xfer->size_bytes == 0) {
		symspi_err("%s: new xfer orders 0 bytes new size."
			   " Will not apply.", __func__);
		return -SYMSPI_ERROR_XFER_SIZE_ZERO;
	}
	if (regions_overlap(curr_xfer->data_tx, curr_xfer->size_bytes
			    , new_xfer->data_tx, new_xfer->size_bytes)) {
		symspi_err("%s: new and current xfers TX datas overlap."
			   "Current data: %px, size %d;"
			   "New data: %px, size %d."
			   " Will not apply.", __func__, curr_xfer->data_tx
			   , (int)curr_xfer->size_bytes, new_xfer->data_tx
			   , (int)new_xfer->size_bytes);
		return -SYMSPI_ERROR_OVERLAP;
	}

	if (curr_xfer->size_bytes != new_xfer->size_bytes) {
		// If consumer requested to change the xfer size, after
		// previous xfer was closed, data race conditions and
		// sync between sides loss may appear (if other side is
		// not aware about this change).
		if (symspi->p->state != SYMSPI_STATE_XFER
				&& !force_size_change) {
			symspi_err("%s: sudden change in xfer size"
				   " while not in XFER state. Will"
				   " not apply.", __func__);
			return -SYMSPI_ERROR_XFER_SIZE_MISMATCH;
		}

		symspi_do_resize_xfer(curr_xfer, new_xfer->size_bytes);

		if (curr_xfer->size_bytes == 0) {
			return -SYMSPI_ERROR_NO_MEMORY;
		}
	}

	memcpy(curr_xfer->data_tx , new_xfer->data_tx
	       , curr_xfer->size_bytes);

	// TODO: to make a bulk copy, to avoid naming members
	//      (will avoid complications in debugging)
	curr_xfer->id = new_xfer->id;
	curr_xfer->done_callback = new_xfer->done_callback;
	curr_xfer->fail_callback = new_xfer->fail_callback;
	curr_xfer->consumer_data = new_xfer->consumer_data;
	curr_xfer->xfers_counter = new_xfer->xfers_counter;

	symspi_do_update_native_spi_xfer_data(symspi);

	return SYMSPI_SUCCESS;
}

// CONTEXT:
//      sleepable
//
static int symspi_update_xfer_sequence(
		struct symspi_dev __kernel *symspi
		, struct __kernel full_duplex_xfer *xfer
		, int original_state
		, bool force_size_change)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No symspi device given."
			    , return -FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED);
	SYMSPI_CHECK_STATE(original_state
			   , return -SYMSPI_ERROR_LOGICAL);
#endif
	if (!xfer) {
		return SYMSPI_SUCCESS;
	}

	xfer->id = symspi_get_next_xfer_id(symspi);
	xfer->xfers_counter = 0;

	int res = symspi_replace_xfer(symspi, xfer, force_size_change);

	// TODO: verify sequence
	if (res != SYMSPI_SUCCESS) {
		symspi_err("%s: Update xfer failed.", __func__);

		if (res != -SYMSPI_ERROR_LOGICAL
				&& res != -SYMSPI_ERROR_NO_MEMORY) {
			res = SYMSPI_SUCCESS;
		}

		// will not recursively goto error state
		if (original_state != SYMSPI_STATE_ERROR) {
			symspi_to_idle_sequence(symspi, original_state
						, false , res);
		}
	}

	return res;
}


// Returns if there was request for new xfe from the other side since
// the previous/current (which was last active) xfer start.
inline static bool symspi_is_their_request(
		struct symspi_dev __kernel *symspi)
{
	// see their_flag_drop_counter description
	return symspi->p->their_flag_drop_counter == 1
		    && symspi_their_flag_is_set(symspi);
}

// Small helper to get our state pointer from our general data
//
// NOTE: only for internal use
inline static char *symspi_get_state_ptr(void *symspi_dev)
{
	return &(((struct symspi_dev *)symspi_dev)->p->state);
}

// Small helper to get our state
//
// NOTE: only for internal use
inline static char symspi_get_state(void *symspi_dev)
{
	return ((struct symspi_dev *)symspi_dev)->p->state;
}

// Atomically swiches the state from expected_state to
// dst_state if and only if current state == expected_state.
//
// \returns
//      true, if state was changed from expected_state to
//              dst_state;
//      false, otherwise (even if current state is already
//              equal to dst_state)
//
// NOTE: only for internal use
inline static bool symspi_switch_strict(void *symspi_dev_ptr,
	char expected_state, char dst_state)
{
	char *state_ptr = symspi_get_state_ptr(symspi_dev_ptr);

	// as closing request comes we can't do anything except
	// leaving the XFER state
	if (__symspi_is_closing((struct symspi_dev*)symspi_dev_ptr)) {
		// we should not change any state except XFER and we
		// should change it to state other than XFER when closing
		if (expected_state != SYMSPI_STATE_XFER
				|| dst_state == SYMSPI_STATE_XFER) {
			return false;
		}

		__atomic_compare_exchange_n(state_ptr, &expected_state
				, dst_state, false, __ATOMIC_SEQ_CST
				, __ATOMIC_SEQ_CST);

		// at this point we are in correct state for closing
		// anyway ,
		complete(&(((struct symspi_dev*)symspi_dev_ptr)
				->p->final_leave_xfer_completion));
		return false;
	}

	bool res = __atomic_compare_exchange_n(state_ptr, &expected_state
			, dst_state, false, __ATOMIC_SEQ_CST
			, __ATOMIC_SEQ_CST);
	if (res) {
		symspi_trace_raw(
				"Switched from %d to %d", (int)expected_state
				, (int)dst_state);
	} else {
		symspi_trace_raw(
				"Tried switch from %d to %d, but failed"
				, (int)expected_state
				, (int)dst_state);
		symspi_trace_raw(
				"Current state: %d", (int)(*state_ptr));
	}
	return res;
}

// Atomically sets the state value to destination.
//
// RETURNS:
//      the old state value
//
// NOTE: only for internal use
inline static char symspi_switch_state_val_forced(void *symspi_dev_ptr
	, char dst_state)
{
	symspi_info(SYMSPI_LOG_INFO_DBG_LEVEL
		    , "Forced switching to %d.", (int)dst_state);
	char *state_ptr = symspi_get_state_ptr(symspi_dev_ptr);
	return __atomic_exchange_n(state_ptr, dst_state, __ATOMIC_SEQ_CST);
}

// TODO: not used for now, need performance tests to decide
// Wrapper to launxh xfer. This wrapper is launched by
// worker from work queue.
static void symspi_do_xfer_work_wrapper(struct work_struct *work)
{
	SYMSPI_CHECK_WORK(work, return);

	struct symspi_dev *symspi;
	SYMSPI_GET_DEVICE_FROM_WORK(symspi, work, xfer_work);

	symspi_do_xfer(symspi);
}


// Triggers the SPI hardware xfer control sequence.
// (hardware works with or without waiting to
// SPI_READY depending on the current configuration).
//
// As long as this function can be called from ISR it should
// be lightweight.
//
// It is called on SPI master side only.
//
// CONTEXT:
//      Any
//
// STATE:
//      SYMSPI_STATE_XFER
//
// RETURNS:
//      SYMSPI_SUCCESS
//              - on success
//      < 0
//              - negated error code
//
// ERRORS:
//      SYMSPI_ERROR_SPI
//
// DBG ERRORS:
//      SYMSPI_ERROR_LOGICAL
static int symspi_do_xfer(struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	// assertions
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -SYMSPI_ERROR_LOGICAL);
	SYMSPI_CHECK_STATE(SYMSPI_STATE_XFER
			    , return -SYMSPI_ERROR_LOGICAL);
	if (!symspi->p->spi_master_mode) {
		symspi_err("%s: called on SPI slave.", __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
#endif

	// dropping their flag falling edge counter right before xfer
	__atomic_store_n(&symspi->p->their_flag_drop_counter
			 , 0, __ATOMIC_SEQ_CST);

	// note, SPI_READY flow is enabled/disabled at SPI init time
	int res = spi_async(symspi->spi, &symspi->p->spi_msg);
	if (res == 0) {
		return SYMSPI_SUCCESS;
	}

	symspi_err("%s: SPI driver returned with error: %d."
		   , __func__, res);

	return -SYMSPI_ERROR_SPI;
}


// Work wrapper for recovery sequence
static void symspi_recovery_sequence_wrapper(struct work_struct *work)
{
	SYMSPI_CHECK_WORK(work, return);

	struct symspi_dev *symspi;
	SYMSPI_GET_DEVICE_FROM_WORK(symspi, work, recover_work);

	symspi_recovery_sequence(symspi);
}

// Attempts to restore the correct device state and bring
// communication back.
//
// NOTE:
//      not to be called from the timer directly, cause should
//      wait for timer handler to exit.
//
// CONTEXT:
//      sleepable
//
// RETURNS:
//      0: (default recovery)/(consumer layer recovery action) was successfull
//      <0: negated error code, if recovery failed
//
// STATE
//      SYMSPI_STATE_ERROR -> SYMSPI_STATE_IDLE (if recoverable)
//                         -> or SYMSPI_STATE_COLD (if non recoverable)
// TODO: consider - the error can be triggered almost simultaneously
//      from other side and from timer, so need to check for concurrence
//      protection
static int symspi_recovery_sequence(struct symspi_dev *symspi)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("device data/pointer broken, "
			"can't recover", return -ENODEV);
	SYMSPI_CHECK_STATE(SYMSPI_STATE_ERROR, return -SYMSPI_ERROR_LOGICAL);

	int error_code = symspi->p->last_error;
	struct full_duplex_xfer *next_xfer = NULL;
	struct full_duplex_xfer *current_xfer = &symspi->p->current_xfer;

	struct symspi_error_rec *e_ptr = __symspi_get_error_rec(symspi
								, error_code);
	bool report = e_ptr ? e_ptr->last_reported : true;

	if (report) {
		symspi_warning_raw("starting recovery of SymSPI, "
				   "after warning/error: %d", error_code);
	}

	__symspi_stop_timeout_timer_sync(symspi);

	// report about an error to the other side
	symspi_our_flag_drop(symspi);
	symspi_wait_flag_silence_period();
	symspi_our_flag_set(symspi);
	symspi_wait_flag_silence_period();
	symspi_our_flag_drop(symspi);
	symspi_wait_flag_silence_period();
	symspi_our_flag_set(symspi);
	symspi_wait_flag_silence_period();
	symspi_our_flag_drop(symspi);
	symspi_wait_flag_silence_period();

	// idle time of scilence to give other side time to react
	const unsigned long idle_time_us
		= SYMSPI_ERROR_RECOVERY_SILENCE_TIME_MS * 1000;
	const int variance
		= SYMSPI_ERROR_RECOVERY_SILENCE_TIME_VARIANCE_PERCENT;
	// we allow 10% of variance in sleeping time
	usleep_range((idle_time_us * (100 - variance)) / 100
		     , (idle_time_us * (100 + variance)) / 100);

	if (!IS_ERR_OR_NULL(current_xfer->fail_callback)) {
		next_xfer = current_xfer->fail_callback(
				current_xfer
				, symspi->p->next_xfer_id
				, error_code
				, current_xfer->consumer_data);
	}

	if (IS_ERR(next_xfer)) {
		symspi_warning_raw("Device was halted in XFER by"
				   " consumer request.");
		return SYMSPI_SUCCESS;
	} else if (next_xfer != NULL) {
		if (report) {
			symspi_warning_raw("Consumer layer provided"
					   " xfer update.");
		}
		// size change upon end of postprocessing is totally OK
		int res = symspi_update_xfer_sequence(symspi, next_xfer
						      , SYMSPI_STATE_ERROR
						      , true);

		// we need to indicate to consumer that next_xfer
		// will not be used by us any more
		if (symspi->xfer_accepted_callback) {
			symspi->xfer_accepted_callback(next_xfer);
		}

		if (res != SYMSPI_SUCCESS) {
			symspi_warning_raw("Error recovery failed, couldn't"
					   " update to new xfer, error: %d."
					   " Device halted.", -res);
			return res;
		};
	} else {
		if (report) {
			symspi_warning_raw("Restarting the current xfer.");
		}
	}

	// dropping their error indication
	__atomic_store_n(&symspi->p->their_flag_drop_counter
			 , 1, __ATOMIC_SEQ_CST);

	symspi->p->last_error = SYMSPI_SUCCESS;
	if (report) {
		symspi_warning_raw("Recovery completed.");
	}
	return symspi_to_idle_sequence(symspi, SYMSPI_STATE_ERROR
				       , true, SYMSPI_SUCCESS);
}

// Helper.
// Waits for appropriate flag silence period (to make
// other side to detect the drop-raise or raise-drop sequence)
static void symspi_wait_flag_silence_period()
{
	// the delay is needed to make other side detect our flag
	// raise and drop, otherwise other side may not detect the
	// drop-raise of our flag
	const unsigned long usecs
		= SYMSPI_OUR_FLAG_INACTIVE_STATE_MIN_TIME_USEC;
	const unsigned int variance
		= SYMSPI_OUR_FLAG_INACTIVE_STATE_MIN_TIME_VARIANCE_PERCENT;
	if (usecs) {
		usleep_range((usecs * (100 - variance)) / 100
			     , (usecs * (100 + variance)) / 100);
	}
}

// Sets our flag line to ACTIVE state
static void symspi_our_flag_set(struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided.", return);
#endif
	symspi_trace("Our flag SET.");
	gpiod_set_raw_value(symspi->gpiod_our_flag
			    , symspi->p->spi_master_mode
			      ? SYMSPI_MASTER_FLAG_ACTIVE_VALUE
			      : SYMSPI_SLAVE_FLAG_ACTIVE_VALUE);
}

// Drops our flag line to INACTIVE state
//
// Has integrated blind time delay, to make other side to react
// on fast drop-raise conditions.
static void symspi_our_flag_drop(struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided.", return);
#endif
	symspi_trace("Our flag DROP.");
	gpiod_set_raw_value(symspi->gpiod_our_flag
			    , symspi->p->spi_master_mode
			      ? !SYMSPI_MASTER_FLAG_ACTIVE_VALUE
			      : !SYMSPI_SLAVE_FLAG_ACTIVE_VALUE);
}

// Returns status of their flag (true: ACTIVE; false: INACTIVE)
inline static bool symspi_their_flag_is_set(struct symspi_dev *symspi)
{
	// NOTE: we test against other side
	symspi_trace("Their flag raw value: %d"
		    , gpiod_get_raw_value(symspi->gpiod_their_flag));
	if ((symspi->p->spi_master_mode ? SYMSPI_SLAVE_FLAG_ACTIVE_VALUE
					: SYMSPI_MASTER_FLAG_ACTIVE_VALUE)
			== gpiod_get_raw_value(symspi->gpiod_their_flag)) {
		symspi_trace("Their flag is SET");
	} else {
		symspi_trace("Their flag is NOT SET");
	}
	return (symspi->p->spi_master_mode ? SYMSPI_SLAVE_FLAG_ACTIVE_VALUE
					   : SYMSPI_MASTER_FLAG_ACTIVE_VALUE)
		== gpiod_get_raw_value(symspi->gpiod_their_flag);
}


// If appropriate, makes transition from WAITING_PREV state
// to the next state (according to correcto workflow). Try
// is because we have concurrency with interrupts which may
// trigger the same sequence as we whant and it is OK.
//
// To be called when conditions for transition are fulfilled.
//
// CONTEXT:
//      any
//
// STATE:
//      any, but switches only from WAITING_PREV state
//
// ERRORS:
//      SYMSPI_ERROR_SPI
//
// ERRORS DBG:
//      SYMSPI_ERROR_LOGICAL
//
static int symspi_try_leave_waiting_prev_sequence(
	struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -SYMSPI_ERROR_LOGICAL);
#endif
	// slave spi (always syncroneously bypasses the WAITING_PREV
	// state and their flag drop ISR does no state switch
	// so we have no concurrency here and thus switch should always
	// be successfull)
	// TODO: check! @@@@@@@@@
	if (!symspi->p->spi_master_mode) {
#ifdef SYMSPI_DEBUG
		// TODO: check! @@@@@@@@@ it looks that we need to
		//  switch even not in DEBUG mode
		if (!SYMSPI_SWITCH_STRICT(WAITING_PREV, XFER)) {
			symspi_err("%s: couldn't bypass WAITING_PREV state"
				   " on slave.", __func__);
			return -SYMSPI_ERROR_LOGICAL;
		}
#endif
		__symspi_stop_timeout_timer(symspi);
		return symspi_do_xfer(symspi);
	}

	// master spi
	if (symspi->p->hardware_spi_rdy) {
		if (SYMSPI_SWITCH_STRICT(WAITING_PREV, XFER)) {
			__symspi_stop_timeout_timer(symspi);
			return symspi_do_xfer(symspi);
		}
	} else if (SYMSPI_SWITCH_STRICT(WAITING_PREV, WAITING_RDY)) {
		__symspi_restart_timeout_timer(symspi);
		if (symspi_is_their_request(symspi)) {
			return symspi_try_leave_waiting_rdy_sequence(symspi);
		}
	}

	return SYMSPI_SUCCESS;
}

// Tries to leave the waiting ready state.
//
// To be called when conditions for transition are fulfilled.
//
// CONTEXT:
//      any
//
// SIDE:
//      only on SPI master
//
// STATE:
//      any, but switches only from WAITING_RDY state
//
// ERRORS:
//      SYMSPI_ERROR_SPI
//
// ERRORS DBG:
//      SYMSPI_ERROR_LOGICAL
static int symspi_try_leave_waiting_rdy_sequence(
	struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -SYMSPI_ERROR_LOGICAL);
	if (!symspi->p->spi_master_mode) {
		symspi_err("%s: called when in slave mode"
			   , __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
	if (symspi->p->hardware_spi_rdy) {
		symspi_err("%s: called when in SPI_RDY hardware"
			   " support used.", __func__);
		return -SYMSPI_ERROR_LOGICAL;
	}
#endif
	if (SYMSPI_SWITCH_STRICT(WAITING_RDY, XFER)) {
		__symspi_stop_timeout_timer(symspi);
		return symspi_do_xfer(symspi);
	}
	return SYMSPI_SUCCESS;
}


// Helper function to retrieve the next xfer id.
//
// Wraps IDs around.
static inline int symspi_get_next_xfer_id(struct symspi_dev *symspi)
{
	int xfer_id = symspi->p->next_xfer_id++;
	// wrapping around
	if (xfer_id <= 0) {
		xfer_id = SYMSPI_INITIAL_XFER_ID;
		symspi->p->next_xfer_id = SYMSPI_INITIAL_XFER_ID + 1;
	}
	return xfer_id;
}

// Helper function.
static void symspi_inc_current_xfer_counter(struct symspi_dev *symspi)
{
	struct full_duplex_xfer *current_xfer = &symspi->p->current_xfer;

	current_xfer->xfers_counter++;
	if (current_xfer->xfers_counter < 0) {
		current_xfer->xfers_counter = 1;
		symspi_warning("%s: xfer counter overflow. Set to 1."
			       , __func__);
	}
}

// This function is to be called in sleepable context (say from
// work queue process context) cause it launches the consumer
// xfer done callback.
//
// CONTEXT:
//      sleepable
//
static void symspi_postprocessing_sequence(struct work_struct *work)
{
	struct symspi_dev *symspi = NULL;

	SYMSPI_CHECK_WORK(work, goto error_handling);
	SYMSPI_GET_DEVICE_FROM_WORK(symspi, work, postprocessing_work);
	SYMSPI_CHECK_DEVICE("No device provided.", goto error_handling);

#ifdef SYMSPI_DEBUG
	if (!(&symspi->p->current_xfer)) {
		symspi_err("%s: no current xfer.", __func__);
		goto error_handling;
	}
#endif

	SYMSPI_CHECK_STATE(SYMSPI_STATE_POSTPROCESSING, return);

	struct full_duplex_xfer *current_xfer = &symspi->p->current_xfer;
	struct full_duplex_xfer *next_xfer = NULL;
	bool start_immediately = false;

	symspi_inc_current_xfer_counter(symspi);

	// notify, provide data to our consumer and optionally get new
	if (current_xfer->done_callback) {
		next_xfer = current_xfer->done_callback(
				current_xfer, symspi->p->next_xfer_id
				, &start_immediately
				, current_xfer->consumer_data);
	}

	// if consumer returns an error value, then we halt in XFER state
	// with our flag raised. So connection freeses until reset explicitly
	// by consumer.
	if (IS_ERR(next_xfer)) {
		// NOTE: here we don't drop our flag to indicate to the other
		//      side, that connection is halted.
		symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL
			    , "Device was halted in XFER state by"
			    " consumer request");
		return;
	}

	// consumer provided new default xfer
	if (next_xfer) {
		// size change upon end of postprocessing is totally OK
		int res = symspi_update_xfer_sequence(symspi, next_xfer
					, SYMSPI_STATE_POSTPROCESSING
					, true);

		// we need to indicate to consumer that next_xfer
		// will not be used by us any more
		if (symspi->xfer_accepted_callback) {
			symspi->xfer_accepted_callback(next_xfer);
		}

		if (res != SYMSPI_SUCCESS) {
			symspi_our_flag_drop(symspi);
			symspi_wait_flag_silence_period();
			return;
		};
	}

	symspi_our_flag_drop(symspi);
	symspi_wait_flag_silence_period();

	// And only after postprocessing of the data is done, then
	// the xfer cycle is really done, so we move either to IDLE state
	// or to next xfer.
	symspi_to_idle_sequence(symspi, SYMSPI_STATE_POSTPROCESSING
				, start_immediately
				, SYMSPI_SUCCESS);

	return;

error_handling:

	if (!symspi) {
		symspi_err("%s: no device, could not start recovery."
			   , __func__);
		return;
	}

	symspi_to_idle_sequence(symspi, SYMSPI_STATE_POSTPROCESSING
				, start_immediately
				, FULL_DUPLEX_ERROR_NO_DEVICE_PROVIDED);
}

// Goes to error processing path if there is any error detected/provided.
//
// To be called when error detected or when external error absence
// verification is needed.
//
// @internal_error {<=0} the negated error code
//
// STATE:
//      any -> SYMSPI_STATE_ERROR
// CONTEXT:
//      any
// RETURNS:
//      SYMSPI_SUCCESS if no errors were detected
//      negated error code (<0) if there were errors (correspondingly
//      error recovery procedure either was executed or
//      scheduled)
static int symspi_try_to_error_sequence(struct symspi_dev *symspi
					, int internal_error)
{
	bool other_side_error = (symspi->p->their_flag_drop_counter > 1);

	if (internal_error != SYMSPI_SUCCESS || other_side_error) {
		int err_no = (internal_error == SYMSPI_SUCCESS)
					? SYMSPI_ERROR_OTHER_SIDE
					: -internal_error;
		__symspi_error_handle(err_no, 0);
		return -err_no;
	}

	return SYMSPI_SUCCESS;
}

// Does state cycle finalizing sequence. To be called to return to
// IDLE state. In case of errors launches the error recovery sequence.
// In case of previously postponed xfer requests, starts the new xfer
// sequence.
//
// @internal_error {<=0} negated code of internal error
//
// NOTE:
//      not to be called (directly or indirectly from timer) cause
//      waits for timer handler to exit.
//
// RETURNS:
//      0 on success
//      < 0 negated error code
//
// STATE:
//      defined by caller, considered:
//
//      XFER
//      XFER_PREPARE
//
// CONTEXT:
//      sleepable
//
static int symspi_to_idle_sequence(struct symspi_dev *symspi
				   , const int original_state
				   , bool start_next_xfer
				   , const int internal_error)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided."
			    , return -SYMSPI_ERROR_LOGICAL);
	SYMSPI_CHECK_STATE(original_state
			   , return -SYMSPI_ERROR_LOGICAL);
#endif

	// in case we had subtle data races within last frame
	// (when timer was first deleted and then started instead
	// of inverse order) then we delete the timer upon the
	// returning to the IDLE state.
	__symspi_stop_timeout_timer_sync(symspi);

	start_next_xfer = start_next_xfer || symspi->p->delayed_xfer_request;

	symspi_switch_strict((void*)symspi, original_state
			     , SYMSPI_STATE_IDLE);

	if (original_state != SYMSPI_STATE_ERROR) {
		int res = symspi_try_to_error_sequence(symspi, internal_error);
		if (res != SYMSPI_SUCCESS) {
			return res;
		}
	} else {
		symspi_info_raw(SYMSPI_LOG_INFO_OPT_LEVEL
				, "Recovered. Resuming.");
	}

	start_next_xfer = start_next_xfer || symspi->p->delayed_xfer_request;
	symspi->p->delayed_xfer_request = false;

	// There will be no uncontrollable double xfer from our side
	// as long as first passed call will switch the state and the next
	// call will be rejected. And even if one execution branch will
	// make whole sequence so fast that will turn around and get back to
	// IDLE state while other execution thread will sleep, it will notify
	// consumer first, and consumer is either
	// OK with repeating current xfer or will update the current xfer.
	//
	// Then the additional customer call will look like other side
	// ordinary request.
	if (start_next_xfer || symspi_is_their_request(symspi)) {
		return symspi_data_xchange(symspi, NULL, false);
	}

	return SYMSPI_SUCCESS;
}

// Helper. Inits the SymSPI procfs.
// RETURNS:
//      >= 0: on success,
//      < 0: on failure (negated error code)
static inline int __symspi_procfs_init(struct symspi_dev *symspi)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("", return -ENODEV);

	symspi->p->proc_root = proc_mkdir(SYMSPI_PROC_ROOT_NAME, NULL);

	if (IS_ERR_OR_NULL(symspi->p->proc_root)) {
		symspi_err("failed to create SymSPI proc root folder"
			  " with name: "SYMSPI_PROC_ROOT_NAME);
		return -EIO;
	}
	return 0;
}

// Closes the SymSPI proc root
static inline void __symspi_procfs_close(struct symspi_dev *symspi)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("", return);

	if (IS_ERR_OR_NULL(symspi->p->proc_root)) {
		return;
	}

	proc_remove(symspi->p->proc_root);
	symspi->p->proc_root = NULL;
}

// Helper. Initializes the info structure of SymSPI.
// NOTE: the SymSPI proc rootfs should be created beforehand,
//      if not: then we will fail to create info node.
//
// RETURNS:
//      >= 0: on success,
//      < 0: on failure (negated error code)
static inline int __symspi_info_init(struct symspi_dev *symspi)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("", return -ENODEV);

	// initial statistics data
	memset(&symspi->p->info, 0, sizeof(symspi->p->info));

	// info access operations
	memset(&symspi->p->info_ops, 0, sizeof(symspi->p->info_ops));
	symspi->p->info_ops.read  = &__symspi_info_read;
	symspi->p->info_ops.owner = THIS_MODULE;

	if (IS_ERR_OR_NULL(symspi->p->proc_root)) {
		symspi_err("failed to create info proc entry:"
			  " no SymSPI root proc entry");
		symspi->p->info_file = NULL;
		return -ENOENT;
	}

	symspi->p->info_file = proc_create_data(
					   SYMSPI_INFO_FILE_NAME
					   , SYMSPI_PROC_R_PERMISSIONS
					   , symspi->p->proc_root
					   , &symspi->p->info_ops
					   , (void*)symspi);

	if (IS_ERR_OR_NULL(symspi->p->info_file)) {
		symspi_err("failed to create info proc entry.");
		return -EIO;
	}

	return 0;
}

// Removes the SymSPI proc info file
static void __symspi_info_close(struct symspi_dev *symspi)
{
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("", return);

	if (IS_ERR_OR_NULL(symspi->p->info_file)) {
		return;
	}

	proc_remove(symspi->p->info_file);
	symspi->p->info_file = NULL;
}

// Provides the read method for SymSPI info to user world.
// Is invoked when user reads the /proc/<SYMSPI>/<INFO> file.
//
// Is restricted to the file size of SIZE_MAX bytes.
//
// RETURNS:
//      >= 0: number of bytes actually provided to user space, on success
//      < 0: negated error code, on failure
//
static ssize_t __symspi_info_read(struct file *file
		, char __user *ubuf, size_t count, loff_t *ppos)
{
	SYMSPI_CHECK_PTR(file, return -EINVAL);
	SYMSPI_CHECK_PTR(ubuf, return -EINVAL);
	SYMSPI_CHECK_PTR(ppos, return -EINVAL);

	struct symspi_dev *symspi = (struct symspi_dev *)PDE_DATA(file->f_inode);

	SYMSPI_CHECK_DEVICE_AND_PRIVATE("", return -ENODEV);

	const int BUFFER_SIZE = 2048;

	if (*ppos >= BUFFER_SIZE || *ppos > SIZE_MAX) {
		return 0;
	}

	char *buf = kmalloc(BUFFER_SIZE, GFP_KERNEL);

	if (IS_ERR_OR_NULL(buf)) {
		return -ENOMEM;
	}

	const struct symspi_info * const s = &symspi->p->info;
	size_t len = (size_t)snprintf(buf, BUFFER_SIZE
		     , "Statistics:\n"
		       "other side indicated errors:  %llu\n"
		       "other side no reaction errors:  %llu\n"
		       "xfers done OK:  %llu\n"
		       "their flag edges detected:  %llu\n"
		       "\n"
		       "Configuration:\n"
			   "max xfer size: "
			   macro_val_str(SYMSPI_XFER_SIZE_MAX_BYTES)
			   " bytes\n"
			   "our flag min inactive time: "
               macro_val_str(SYMSPI_OUR_FLAG_INACTIVE_STATE_MIN_TIME_USEC)
			   " us\n"
			   "their flag wait timeout: "
               macro_val_str(SYMSPI_THEIR_FLAG_WAIT_TIMEOUT_MSEC)
			   " ms\n"
			   "error recovery silence time: "
               macro_val_str(SYMSPI_ERROR_RECOVERY_SILENCE_TIME_MS)
			   " ms\n"
			   "workqueue mode: "macro_val_str(SYMSPI_WORKQUEUE_MODE)"\n"
			   "verbosity level: "macro_val_str(SYMSPI_VERBOSITY)"\n"
		       "\n"
		       "Note: statistical/monitoring"
		       " info is not expeted to be used in precise"
		       " measurements due to atomic selfconsistency"
		       " maintenance would put overhead in the driver.\n"
			, s->other_side_indicated_errors
			, s->other_side_no_reaction_errors
			, s->xfers_done_ok
			, s->their_flag_edges
		);
	len++;

	if (len > BUFFER_SIZE) {
		symspi_warning("statistics output was too big for buffer"
			      ", required length: %zu", len);
		len = BUFFER_SIZE;
		buf[BUFFER_SIZE - 1] = 0;
	}

	const unsigned long nbytes_to_copy
			= (len >= (size_t)(*ppos))
				?  min(len - (size_t)(*ppos), count)
				: 0;
	const unsigned long not_copied
			= copy_to_user(ubuf, buf + (size_t)(*ppos)
				       , nbytes_to_copy);
	kfree(buf);
	buf = NULL;
	*ppos += nbytes_to_copy - not_copied;

	return nbytes_to_copy - not_copied;
}


/* ----------------------- SPI CALLBACKS SECTION ----------------------- */

// Will be called upon the xfer finished by SPI driver
//
// CONTEXT:
//      SPI driver calls this callback from context which can't sleep.
//
// Is called in a context which can not sleep. See spi_async(...)
// description for more information.
static void symspi_spi_xfer_done_callback(void *context)
{
	struct symspi_dev *symspi = (struct symspi_dev*)context;

#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("No device provided.", return);
#endif

	// No one except us can exit the xfer state, even error
	// handling shall be postponed
	if (!SYMSPI_SWITCH_STRICT(XFER, POSTPROCESSING)) {
		__symspi_error_handle(SYMSPI_ERROR_LOGICAL, 0);
		return;
	}

	// Trigger postponed error handling if needed
	if (symspi->p->last_error != SYMSPI_SUCCESS) {
		__symspi_error_handle(symspi->p->last_error, 0);
		return;
	}

	// NOTE: SPI layer uses mostly (always?) standard
	//      Linux errors.
	// not all went fine =(
	if (symspi->p->spi_msg.status != 0) {
		__symspi_error_handle(SYMSPI_ERROR_SPI
				      , symspi->p->spi_msg.status);
		return;
	}

	// update overview info
	symspi->p->info.xfers_done_ok += 1;

	// all went fine
	// we'll shedule the data processing
	// cause we can not run potentially heavy
	// and unreliable routine within context
	// which can't sleep
	__symspi_schedule_work(symspi, &symspi->p->postprocessing_work);
}


/* ----------------------- ISR SECTION --------------------------------- */

// ISR: Handles their flag drop and set edges
//
// CONTEXT:
//      can not sleep (ISR)
//
// As long as it is impossible to register separate ISRs for raising
// and falling edge of the pin, then both are unified under the
// following ISR.
//
// TODO: the issue exists that in case of fast drop/raise or raise/drop
//      of other side flag, we will not be able to detect it cause
//      if this ISR we manually check the flag state AFTER the interrupt
//      happened, so if flag state was altered in following way:
//
//         OTHER FLAG                 EXECUTION
//
//       dropped state                some code
//
//           raise  ----------------> start of switch to the ISR context
//                                    still switching
//
//           drop   --(isr queued?)-- just switched, starting ISR execution
//
//                                    ISR checks the other flag state and
//                                    gets LO, while it should be first
//                                    HI, then (in next interupt), LO
//
//      this might be fixed if have separate handlers for GPIO raise and
//      drop events, but this functionality is not provided at GPIO
//      abstration layer, which allows only to register single ISR to
//      only drop, or only raise, or only to drop/raise events. Clarify
//      if hardware allows us to register one ISR to raise and other ISR
//      to drop.
static irqreturn_t symspi_their_flag_isr(int irq, void *symspi_device)
{
	struct symspi_dev *symspi = (struct symspi_dev *)symspi_device;

#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided.", return IRQ_HANDLED);
#endif
	symspi_trace("Their flag ISR.");

	if (symspi->p->state == SYMSPI_STATE_COLD) {
		return IRQ_HANDLED;
	}

	if (symspi_their_flag_is_set(symspi)) {
		symspi_their_flag_set_isr_sequence(symspi);
	} else {
		symspi_their_flag_drop_isr_sequence(symspi);
	}

	// track in info
	symspi->p->info.their_flag_edges += 1;

	return IRQ_HANDLED;
}

// Handles their flag drop edge: from ACTIVE to INACTIVE
// (falling edge of the flag means: "previous xfer was processed")
//
// CONTEXT:
//      can not sleep (ISR)
//
// \param symspi_dev is a pointer on our symspi data
//
static void symspi_their_flag_drop_isr_sequence(
		struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE_AND_PRIVATE("No device provided.", return);
#endif
	int *counter_ptr = &(symspi->p->their_flag_drop_counter);

	// NOTE: we could use the kernel provided function, but
	//          with current kernel version, all atomic operations
	//          except those for x86 arc are implemented there
	//          by blocking the interrupts, which is slow.
	// TODO: if we move to new kernel, use kernel-defined
	//       atomic operations, like:
	//          atomic_add(1, counter_ptr);
	//
	// For now will use GCC atomic builtin.

	const int counter = __atomic_add_fetch(counter_ptr, 1
					       , __ATOMIC_SEQ_CST);

	// ISR does nothing but counter management on SPI slave side
	if (counter == 1 && symspi->p->spi_master_mode) {
		symspi_try_leave_waiting_prev_sequence(symspi);
		return;
	}

	// we launch the recovery procedure only on crossing the border
	// between valid and invalid values to avoid overhead in case of
	// error indication from the other side
	if (counter >= 2) {
		__symspi_error_handle(SYMSPI_ERROR_OTHER_SIDE, 0);
		return;
	}

	if (counter <= 0) {
		symspi_err("Unexpected (<= 0) their flag drop counter value."
			   " Something is really broken.");
	}
}

// Handles their flag set edge: from INACTIVE to ACTIVE
// (raised flag means: "ready for xfer + [have data to send]")
//
// CONTEXT:
//      can not sleep (ISR)
//
// \param symspi_dev is a pointer on our symspi data
//
// On this interrupt we should initiate the xfer.
static void symspi_their_flag_set_isr_sequence(
		struct symspi_dev *symspi)
{
#ifdef SYMSPI_DEBUG
	SYMSPI_CHECK_DEVICE("No device provided.", return);
#endif
	// TODO: Use the softIRQs
	// see: https://notes.shichao.io/lkd/ch8/#implementing-softirqs
	// for detailed description
	//
	// Register them as ~network TX/RX level,
	// as long as ICCom which is a consumer of the is actually
	// the same inter process communication mechanics.

	// other side initiated xfer sequence
	if (SYMSPI_SWITCH_STRICT(IDLE, XFER_PREPARE)) {
		// no work queueing here, to decrease communication latency
		// as long as symspi_do_xfer() contains lightweight
		// determined operations (we don't require consumer data
		// here, it is already prepared)
		symspi_xfer_prepare_to_waiting_prev_sequence(symspi);
		return;
	}

	// in absence of SPI_RDY hardware support on SPI
	// master side, we have to trigger SPI xfer manually
	if (symspi->p->spi_master_mode && !symspi->p->hardware_spi_rdy) {
		symspi_try_leave_waiting_rdy_sequence(symspi);
	}
}

/* --------------------- EXTERNAL SECTION ------------------------------ */

// Allocates a new symspi device with default configuration,
// ready to be started.
//
// @spi {valid ptr to spi device} pointer to spi device to bound to.
// @symspi {valid ptr to memory area allocated for symspi_dev struct}
//      should point to area where to initialize symspi_dev struct.
//      This memory area is managed by consumer.
//
// RETURNS:
//      * negated error code on error (casted to pointer)
//      * 0 on success
static int symspi_make_default_device(
		struct spi_device *spi, struct symspi_dev *symspi)
{
	if (IS_ERR_OR_NULL(spi)) {
		symspi_err("No spi device provided.");
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(symspi)) {
		symspi_err("No memory area for symspi device provided.");
		return -EINVAL;
	}

	int fail_res;
	// TODO consider using managed devices resources
	//      * devm_kmalloc
	//      * devm_gipiod_get

	symspi->gpiod_our_flag = gpiod_get(&spi->dev, "symspi-hsk-out"
					   , GPIOD_OUT_LOW);
	if (IS_ERR(symspi->gpiod_our_flag)) {
		symspi_err("failed to get our flag GPIO device, err = %ld"
			   , PTR_ERR(symspi->gpiod_our_flag));
		fail_res = PTR_ERR(symspi->gpiod_our_flag);
		goto failure;
	}

	symspi->gpiod_their_flag = gpiod_get(&spi->dev, "symspi-hsk-in"
					     , GPIOD_IN);
	if (IS_ERR(symspi->gpiod_their_flag)) {
		symspi_err("failed to get their flag GPIO device, err = %ld"
			   , PTR_ERR(symspi->gpiod_their_flag));
		fail_res = PTR_ERR(symspi->gpiod_their_flag);
		goto free_our_flag_gpiod;
	}

	symspi->spi = spi;
	symspi->xfer_accepted_callback = NULL;
	symspi->p = NULL;

	return SYMSPI_SUCCESS;

free_our_flag_gpiod:
	gpiod_put(symspi->gpiod_our_flag);
	symspi->gpiod_our_flag = ERR_PTR(-ENODEV);
failure:
	return fail_res;
}

// Destroys the symspi device (frees all resources dedicated to
// symspi device). The symspi_dev structure memory itself is still managed
// by consumer. Will stop the device if not stopped.
//
// @ symspi the device structure to release.
//      NOTE: the structure memory is still to be managed by consumer
static void symspi_destroy_device(struct symspi_dev *symspi)
{
	if (IS_ERR_OR_NULL(symspi)) {
		return;
	}
	if (!IS_ERR_OR_NULL(symspi->p)
			&& symspi->p->state != SYMSPI_STATE_COLD) {
		symspi_close((void*)symspi);
	}

	symspi->xfer_accepted_callback = NULL;
	symspi->spi = NULL;

	if (!IS_ERR_OR_NULL(symspi->gpiod_our_flag)) {
		gpiod_put(symspi->gpiod_our_flag);
		symspi->gpiod_our_flag = ERR_PTR(-ENODEV);
	}

	if (!IS_ERR_OR_NULL(symspi->gpiod_their_flag)) {
		gpiod_put(symspi->gpiod_their_flag);
		symspi->gpiod_their_flag = ERR_PTR(-ENODEV);
	}
}

/* --------------------- MODULE HOUSEKEEPING SECTION ------------------- */

EXPORT_SYMBOL(symspi_get_global_device);
EXPORT_SYMBOL(symspi_data_xchange);
EXPORT_SYMBOL(symspi_default_data_update);
EXPORT_SYMBOL(symspi_init);
EXPORT_SYMBOL(symspi_close);
EXPORT_SYMBOL(symspi_is_running);
EXPORT_SYMBOL(symspi_reset);
EXPORT_SYMBOL(symspi_iface);

// The module is to be used via export symbols.
//
// Initialization sequence:
// 1. Upon the symspi compatible device (declared in device tree)
//    is found, kernel loads the SymSPI driver.
// 2. SymSPI driver makes initialization of own structures including
//    symspi_dev structure and keeps it in static global variable.
// 3. SymSPI driver exports symbols to access the symspi_dev structure
//    and all API function of SymSPI.
// 4. Other driver, like ICCom driver will call exported symbols to
//    retreive the SymSPI device data and work with available devices.
// 5. Upon work is done, SymSPI module frees all resources accociated
//    with it.
//
// See also: try_module_get()/module_put() include/linux/module.h



// TODO[Harald comment]: Driver data can be usually stored in a
// field of the spi_device struct itself by using
// spi_set_drvdata(spi, symspi_dev); In other functions you can get it by
// struct symspi_dev* pdata = spi_get_drvdata(spi);
//
//
// Allocates single global symspi device with default
// configuration. If fails all related resources are freed.
//
// NOTE:
//      even if probe function fails, then module will still be
//      loaded and available, so consumers of default global
//      device should check the global device pointer with IS_ERR.
//
// RETURNS:
//      * negative error number on error
//      * 0 on success
static int symspi_probe(struct spi_device *spi)
{
	// At the moment we'll make a global symspi device
	// which will work with the device stated in DT.

	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "module loading");
	symspi_info(SYMSPI_LOG_INFO_OPT_LEVEL, "spi device: %px", spi);

	int fail_res = 0;

	symspi_global_device_ptr = kmalloc(sizeof(struct symspi_dev)
					    , GFP_KERNEL);
	if (!symspi_global_device_ptr) {
		symspi_err("no memory to allocate global symspi device");
		fail_res = -ENOMEM;
		goto failure;
	}

	fail_res = symspi_make_default_device(spi, symspi_global_device_ptr);
	if (fail_res != 0) {
		symspi_err("could not create default symspi device, err = %d"
			   , fail_res);
		goto free_global_symspi_dev;
	}

	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL
		    , "created symspi global device at %px (at COLD state)"
		    , symspi_global_device_ptr);

	// Here the device is ready to be initialized and run.

	return SYMSPI_SUCCESS;

free_global_symspi_dev:
	kfree(symspi_global_device_ptr);
	symspi_global_device_ptr = ERR_PTR(fail_res);
failure:
	return fail_res;
}

static int symspi_remove(struct spi_device *spi)
{
	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL
		    , "module unloading: global dev ptr: %px"
		    , symspi_global_device_ptr);

	symspi_destroy_device(symspi_global_device_ptr);

	if (!IS_ERR_OR_NULL(symspi_global_device_ptr)) {
		kfree(symspi_global_device_ptr);
	}
	symspi_global_device_ptr = NULL;

	symspi_info(SYMSPI_LOG_INFO_KEY_LEVEL, "module unloaded");

	return SYMSPI_SUCCESS;
}

enum symspi_id {
	symspi1
};

static const struct spi_device_id symspi1_id[] = {
	{ "symspi1", symspi1 },
	{}
};
MODULE_DEVICE_TABLE(spi, symspi1_id);

static const struct of_device_id symspi1_dt_ids_match[] = {
	{ .compatible = "bosch,symspi1" },
	{},
};
MODULE_DEVICE_TABLE(of, symspi1_dt_ids_match);


static struct spi_driver symspi_spi_driver = {
	.driver = {
		.name = "symspi",
		.of_match_table = of_match_ptr(symspi1_dt_ids_match),
	},
	.probe = symspi_probe,
	.remove = symspi_remove,
	.id_table = symspi1_id,
};
module_spi_driver(symspi_spi_driver);

MODULE_DESCRIPTION("Module for maintain symmetrical SPI communication.");
MODULE_AUTHOR("Artem Gulyaev <Artem.Gulyaev@bosch.com>");
MODULE_LICENSE("GPL v2");
