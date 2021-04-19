/*
 * This file declares kernel API to SymSPI driver.
 *
 * Driver for the Symmetrical SPI (SymSPI) communication between independent
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

#include "../full_duplex_interface/full_duplex_interface.h"
#include <linux/spi/spi.h>
#include <linux/gpio.h>


/* -------------------- SYMSPI ERRORS ---------------------------------- */

// TODO move to the common Linux error codes when possible
// TODO consider to move to enum if reasonable


// NOTE: Keep updated if adding/removing error type
#define SYMSPI_ERROR_TYPES_COUNT 14


// no error code, keep it 0
#define SYMSPI_SUCCESS 0
// logic error (only used for debugging)
#define SYMSPI_ERROR_LOGICAL 1
// The new xfer size requested with independent data_xchange call
// differs from current xfer size. This situation is race condition.
// Xfer data size change only possible on previous xfer done event.
#define SYMSPI_ERROR_XFER_SIZE_MISMATCH 3
// New xfer size is zero (one should turn off the SPI then)
#define SYMSPI_ERROR_XFER_SIZE_ZERO 4
// We are out of memory
#define SYMSPI_ERROR_NO_MEMORY ENOMEM
// Other side error / sync loss
#define SYMSPI_ERROR_OTHER_SIDE EPIPE
// Trying to do something except error recovery while
// in ERROR state.
#define SYMSPI_ERROR_STATE 8
// consumer provided new xfer TX data which overlaps with
// current xfer TX data
#define SYMSPI_ERROR_OVERLAP 9
// underlying SPI layer returned with error
#define SYMSPI_ERROR_SPI 10
// no SPI device was provided
#define SYMSPI_ERROR_NO_SPI ENODEV
// no GPIO device was provided
#define SYMSPI_ERROR_NO_GPIO 12
// no xfer provided
#define SYMSPI_ERROR_NO_XFER 13
// symspi could not ackqure the irq
#define SYMSPI_ERROR_IRQ_ACQUISITION 14
// symspi could not install ISR
#define SYMSPI_ERROR_ISR_SETUP 15
// error waiting other side reaction
#define SYMSPI_ERROR_WAIT_OTHER_SIDE 16
// error trying to create private work queue
#define SYMSPI_ERROR_WORKQUEUE_INIT 17


/* --------------------- DATA STRUCTS SECTION ---------------------------*/

// Structure representing the SymSPI device.
//
// @xfer_accepted_callback {valid ptr || NULL}
//      Is called from xfer done routine to indicate that we
//      have processed new_xfer data (when provided) and consumer
//      may do whatever it whants with its xfer data. If NULL,
//      then ignored.
//      CONTEXT:
//          sleepable
// @spi {valid ptr} the SPI device to work with.
//      Consumer should keep the device alive and untouched while
//      SymSPI is not in COLD state.
//      OWNERSHIP:
//          consumer
// @gpiod_our_flag {valid ptr} the GPIO device (pin) to be used
//      to carry our flag. Consumer should keep the device alive
//      and untouched while SymSPI is not in COLD state.
//      OWNERSHIP:
//          consumer
// @gpiod_their_flag {valid ptr} the GPIO device (pin) to be used
//      to get their flag. Consumer should keep the device alive
//      and untouched while SymSPI is not in COLD state.
//      OWNERSHIP:
//          consumer
// @their_flag_wait_timeout_ms TODO: unused for now, so either use
//      either remove the filed. TODO: the maximum time we waiting the other
//      side to indicate its readiness via their flag raise (ms).
//      This time is measured between our flag raise (we raise our
//      flag as have prepared the data) and their indication of
//      readiness (other side should drop the flag after previous
//      xfer and then raise it again).
// @native_transfer_configuration_hook {NULL || valid function ptr}
//      this hook can be used by caller layer to define the native
//      transport details at the point when they are configured.
//      As long as SymSPI doesn't (and should not) know how to
//      configure the underlying transport configuration (this
//      should be defined by dedicated protocol driver), this function
//      will be called (if !NULL) when the native transport configuration
//      is created for the underlying transport device so tranport details
//      are defined for the transfer.
//      CONTEXT:
//          can not sleep
//      @xfer{valid ptr}  the current full-duplex transfer for which
//          the transport transfer is initialized
//      @native_transfer{valid ptr} pointer to the native transfer struct
//          to be configured (NOTE: the mandatory fields, like transfer
//          length will be configured after the hook invocation).
//      @native_transfer_struct_size{>0} size (in bytes) of the native
//          transfer struct.
// @p {NULL on initial struct creation} pointer to symspi_dev_private.
//      This pointer is not to be used (read/write) by consumer.
//      Managed by SymSPI internally. But it MUST be set to NULL
//      on initial struct creation (as NULL will be interpreted internally
//      as uninitialized symspi device, while !NULL will be interpreted
//      as initialized device, which contains already selfconsistent valid
//      data)
//
//      OWNERSHIP:
//          SymSPI
struct symspi_dev {

	void (*xfer_accepted_callback)(struct full_duplex_xfer *xfer);

	struct spi_device *spi;

	struct gpio_desc *gpiod_our_flag;
	struct gpio_desc *gpiod_their_flag;

	// TODO: IMPLEMENT
	int their_flag_wait_timeout_ms;

	void (*native_transfer_configuration_hook)(
		const struct full_duplex_xfer *const xfer
		, void *const native_transfer
		, const size_t native_transfer_struct_size);

	struct symspi_dev_private *p;
};


/* -------------------- API DECLARATIONS SECTION ------------------------*/
/* ---------------for documentation, see symspi.c file ------------------*/

int symspi_data_xchange(void __kernel *device
		, struct __kernel full_duplex_xfer *xfer
		, bool force_size_change);
int symspi_default_data_update(void __kernel *device
		, struct full_duplex_xfer *xfer
		, bool force_size_change);
int symspi_init(void __kernel *device
		, struct full_duplex_xfer *default_xfer);
int symspi_close(void __kernel *device);
bool symspi_is_running(void __kernel *device);
int symspi_reset(void __kernel *device
		, struct full_duplex_xfer *default_xfer);
const struct full_duplex_sym_iface *symspi_iface(void);

// not a part of general interface
struct symspi_dev *symspi_get_global_device(void);

