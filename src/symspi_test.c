/*
 * This file provides a linux kernel module to test the symmetrical SPI
 * driver (SymSPI).
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

/*
 * NOTE: to run this test one needs both sides to run the testing code
 *
 * The typical usage from Bash is the following:
 *
 *      # Bash:
 *      insmod spi-imx.ko; sleep 0.2; # inserting iMX SPI driver
 *      insmod symspi.ko; sleep 0.2;  # inserting SymSPI driver
 *      insmod symspi_test.ko && echo "PASSED" || {
 *          echo "Failed! See dmesg output for details"
 *          dmesg
 *      }
 *
 * If one needs to run the test repeatedly and ensure previous
 * modules are unloaded use the following script:
 *
 *      # Bash
 *      rmmod symspi_test; sleep 0.2;
 *      rmmod symspi; sleep 0.2;
 *      rmmod spi-imx; sleep 0.2;
 *      insmod spi-imx.ko; sleep 0.2;
 *      insmod symspi.ko; sleep 0.2;
 *      while true; do {
 *          rmmod symspi_test; sleep 1;
 *          insmod symspi_test.ko || {
 *              echo "Failed! See dmesg output for details"
 *              dmesg
 *              exit 1
 *          }
 *      }; done;
 *
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include "./symspi.h"

// DEV STACK
//
// BACKLOG:
//
// * White box testing
//
// This test module will simulate the other side GPIO
// activity and mock SPI master to simulate a full data
// exchange cycle.
//
//   +--------------------+            +------------+             +------------+
//   |      SymSPI test   |            |            |             |            |
//   |         driver     | Tx.Data()  |            | Xfer data() | +--------+ |
//   |                    +------------>            +--------------->        | |
//   +-------------+      |            |   SymSPI   |             | | Mock   | |
//---->OUR TX FILE |      |            |   driver   |             | | SPI    | |
//   |-------------+      | Xfer done  |            |  Xfer done  | | Driver | |
//----|OUR RX FILE |      <------------+            <---------------+        | |
//   +-------------+      |   callb.   |            |   callb.    | +--------+ |
//   |                    |            +---+----+---+             |            |
//   |   GPIO readings    |       GPIO     |    ^                 |            |
//   |   and writings     |       write    |    | Simulate other  |            |
//   |   of SymSPI        |       (call    |    | side flag raise |            |
//   |   driver are to be |       replaced |    | fall.           |            |
//   |   redirected to    |                |    | Via ISR call.   |            |
//   |   SymSPI test      +----------------v----+-----------------+            |
//   |   driver.          |                                                    |
//   |                                                                         |
//   |      BASE PART     |            ADVANCED PART                           |
//   |                                                                         |
//   |- - - - - - - - - - |                                                    |
//   |                                                                         |
//   +---------------+              Maintains RX consumer data buffers         |
//---->THEIR TX FILE |              on both sides.                             |
//   |---------------+                                                         |
//----|THEIR RX FILE |                                                         |
//   +---------------+                                                         |
//   |                                                                         |
//   +-------------------------------------------------------------------------+
//


/*------------------- FORWARD DECLARATIONS -------------------------*/

struct full_duplex_xfer * symspi_test_xfer_done_default_callback(
                        const struct full_duplex_xfer __kernel *done_xfer
                        , const int next_xfer_id
                        , bool __kernel *start_immediately__out
                        , void *consumer_data);


/*------------------- STATIC MODULE VARS ---------------------------*/

// The correct empty data package, with 2 ID
//
// 2 byte payload len = 0x00 0x00
// 1 byte package ID = 0x02
// ----- PACKET -----
// 57 bytes of empty space = 0xFF
// 4 byte control checksum
static const char symspi_test_correct_empty_data_package[] = {
        0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0x99, 0xFA, 0xD8, 0x0D
};

// 2 byte payload len = 0x00 0x06
// 1 byte package ID = 0x01
// ----- PACKET -----
// 2 byte payload len = 0x00 0x01
// 1 byte LUN = 0x00
// 1 byte CID + complete = 0x80
// 1 byte mock packet data = 0xEE
// 52 bytes of empty space = 0xFF (52 times)
// 4 byte control checksum
static const char symspi_test_init_debug_mode_xfer_data[] = {
        0x00, 0x06, 0x01, 0x00, 0x01, 0x00, 0x80, 0xEE,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF3, 0x7E, 0x11
};
struct full_duplex_xfer symspi_test_start_dbg_xfer = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_init_debug_mode_xfer_data
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = &symspi_test_xfer_done_default_callback
        , .fail_callback = NULL
};

static const char symspi_test_64_byte_xfer_data[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,

        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,

        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,

        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
};
struct full_duplex_xfer symspi_test_64_byte_xfer= {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_64_byte_xfer_data
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = &symspi_test_xfer_done_default_callback
        , .fail_callback = NULL
};


static const char symspi_test_1_byte_xfer_data[] = {
        0xAE
};
struct full_duplex_xfer symspi_test_1_byte_xfer = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_1_byte_xfer_data
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = &symspi_test_xfer_done_default_callback
        , .fail_callback = NULL
};

static const char symspi_test_1_byte_xfer_data_alt[] = {
        0x42
};
struct full_duplex_xfer symspi_test_1_byte_xfer_alt = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_1_byte_xfer_data_alt
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = &symspi_test_xfer_done_default_callback
        , .fail_callback = NULL
};

/*-------------------- TESTS DATA SECTION --------------------------*/

// Fallback xfer: one zero byte, used as default for majority of tests
static const char symspi_test_zeroed_1b_xfer_data[] = {
        0x00
};
struct full_duplex_xfer symspi_test_zeroed_1b_xfer = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_zeroed_1b_xfer_data
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// Fallback xfer: 64 zero byte
static const char symspi_test_zeroed_64b_xfer_data[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
struct full_duplex_xfer symspi_test_zeroed_64b_xfer = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_zeroed_64b_xfer_data
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// default 1 byte xfer
static const char symspi_test_xfer_data_1b_default[] = {
        0x5E
};
struct full_duplex_xfer symspi_test_xfer_1b_default = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_xfer_data_1b_default
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// default 64 byte xfer
static const char symspi_test_xfer_data_64b_default[] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xFF
};
struct full_duplex_xfer symspi_test_xfer_64b_default = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_default
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// switch to 1 byte mode for next 1 xfer
static const char symspi_test_xfer_data_64b_to_1b_xfer_once[] = {
        0x12, 0x34, 0x56, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x29, 0xA4, 0xB1, 0x8C, 0xFD, 0xFE, 0xAA, 0xA8,
        0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer symspi_test_xfer_64b_to_1b_xfer_once = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_to_1b_xfer_once
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// switch to 1 byte mode permanently
static const char symspi_test_xfer_data_64b_to_1b_xfer[] = {
        0x12, 0x34, 0x56, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0x44, 0x33, 0x64, 0xAA, 0xFF, 0xFF,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x29, 0xA4, 0xB1, 0x8C, 0xFD, 0xFE, 0xAA, 0xA8,
        0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer symspi_test_xfer_64b_to_1b_xfer = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_to_1b_xfer
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// request xfer, 64 bytes size
static const char symspi_test_xfer_data_64b_request[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,

        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,

        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,

        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
};
struct full_duplex_xfer symspi_test_xfer_64b_request = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_request
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// request to initiate next 64 byte xfer (64 bytes)
static const char symspi_test_xfer_data_64b_init_xfer_request[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer symspi_test_xfer_64b_init_xfer_request = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_init_xfer_request
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// request to initiate sequence of 1000 their initiated
// with random delay 64 byte xfer (64 bytes)
static const char symspi_test_xfer_data_64b_init_1000x_64b_xfer_rq[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer
symspi_test_xfer_64b_init_1000x_64b_xfer_request = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_init_1000x_64b_xfer_rq
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// 1000 of 64 bytes xfers sequence initiated by other side is done,
// next xfer is initiated in standard way
static const char symspi_test_xfer_data_64b_done_1000x_64b_xfer[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,

        0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer symspi_test_xfer_64b_done_1000x_64b_xfer = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_done_1000x_64b_xfer
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// request to initiate next 1 byte xfer (64 bytes)
static const char symspi_test_xfer_data_64b_init_1b_xfer_request[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
struct full_duplex_xfer symspi_test_xfer_64b_init_1b_xfer_request = {
        .size_bytes = 64
        , .data_tx = (char*)&symspi_test_xfer_data_64b_init_1b_xfer_request
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// answer xfer, 64 bytes size
static const char symspi_test_xfer_data_64b_answer[] = {
        0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
        0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,

        0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
        0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,

        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,

        0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
        0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
};

// request xfer, 1 byte size
static const char symspi_test_xfer_data_1b_request[] = {
        0x52
};
struct full_duplex_xfer symspi_test_xfer_1b_request = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_xfer_data_1b_request
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};

// answer xfer, 1 byte size
static const char symspi_test_xfer_data_1b_answer[] = {
        0x5B
};

// switch back to 64 bytes xfer request, 1 byte
static const char symspi_test_xfer_data_1b_to_64b_xfer[] = {
        0x91
};
struct full_duplex_xfer symspi_test_xfer_1b_to_64b_xfer = {
        .size_bytes = 1
        , .data_tx = (char*)&symspi_test_xfer_data_1b_to_64b_xfer
        , .data_rx_buf = NULL
        , .consumer_data = NULL
        , .done_callback = NULL
        , .fail_callback = NULL
};


/*-------------------- TESTS UTILS SECTION -------------------------*/



#define SYMSPI_TEST_LOG_PREFIX "SYMSPI_TEST: "

#define SYMSPI_TEST_LOG_TEST_PREFIX(test_number)                     \
        "SYMSPI_TEST: test "#test_number": "

#define symspi_test_err(fmt, ...)                                    \
        pr_err(SYMSPI_TEST_LOG_PREFIX"%s: at %d line: "fmt"\n"       \
               , __func__, __LINE__, ##__VA_ARGS__)
#define symspi_test_warn(fmt, ...)                                   \
        pr_warn(SYMSPI_TEST_LOG_PREFIX"%s: at %d line: "fmt"\n"      \
                , __func__, __LINE__, ##__VA_ARGS__)
#define symspi_test_info(fmt, ...)                                   \
        pr_info(SYMSPI_TEST_LOG_PREFIX"%s: at %d line: "fmt"\n"      \
                , __func__, __LINE__, ##__VA_ARGS__)

#define symspi_test_err_raw(fmt, ...)                                \
        pr_err(SYMSPI_TEST_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#define symspi_test_warn_raw(fmt, ...)                               \
        pr_warn(SYMSPI_TEST_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)
#define symspi_test_info_raw(fmt, ...)                               \
        pr_info(SYMSPI_TEST_LOG_PREFIX""fmt"\n", ##__VA_ARGS__)

#define SYMSPI_TEST_DEFINE_TEST(test_number)                         \
        struct completion symspi_test_##test_number##__completion;   \
        bool symspi_test_##test_number##__result;                    \
        int symspi_test_##test_number##__xfer_counter;


#define SYMSPI_TEST_INITIATE_TEST_ACTIONS(test_id)                   \
        pr_info("SYMSPI_TEST: "#test_id": starting\n.");             \
                                                                     \
        symspi_test_##test_id##__xfer_counter = 1;                   \
                                                                     \
        int res = symspi_test_run_symspi(symspi);                    \
        if (res != 0) {                                              \
                pr_info("SYMSPI_TEST: "#test_id": starting of"       \
                        "symspi failed. Error: %d\n."                \
                        , res);                                      \
                return false;                                        \
        }                                                            \
                                                                     \
        init_completion(&symspi_test_##test_id##__completion);       \
        symspi_test_##test_id##__result = true;

#define SYMSPI_TEST_FINALIZE(test_id, xfers_count)                   \
        int timeout_ms = symspi_test_get_timeout_ms(xfers_count);    \
        unsigned long time_left = wait_for_completion_timeout(       \
                        &symspi_test_##test_id##__completion         \
                        , msecs_to_jiffies(timeout_ms));             \
                                                                     \
        if (time_left == 0) {                                        \
            pr_err("SYMSPI_TEST: TEST "#test_id                      \
                   ": FAIL! (timeout).\n");                          \
            return false;                                            \
        }                                                            \
                                                                     \
        if (symspi_test_##test_id##__result) {                       \
            pr_info("SYMSPI_TEST: TEST "#test_id": OK.\n");          \
        } else {                                                     \
            pr_err("SYMSPI_TEST: TEST "#test_id": FAIL!.\n");        \
        }                                                            \
        return symspi_test_##test_id##__result;

#define SYMSPI_TEST_FINISH_SEQ(test_num, xfer_name)                  \
        *start_immediately__out = false;                             \
        SYMSPI_TEST_UNBIND_XFER(xfer_name, test_num);                \
        complete(&symspi_test_##test_num##__completion);             \
        return &symspi_test_xfer_##xfer_name

#define SYMSPI_TEST_FAIL_FINISH_SEQ(test_num, xfer_name, msg, ...)   \
        *start_immediately__out = false;                             \
        symspi_test_##test_num##__result = false;                    \
        SYMSPI_TEST_UNBIND_XFER(xfer_name, test_num);                \
        symspi_test_err("test error: "msg, ##__VA_ARGS__);           \
        complete(&symspi_test_##test_num##__completion);             \
        return &symspi_test_xfer_##xfer_name

#define SYMSPI_TEST_VERIFY_RX(data_name, test_number, fail_xfer_name)\
        equal = symspi_test_packages_equal(done_xfer->data_rx_buf    \
                        , done_xfer->size_bytes                      \
                        , symspi_test_xfer_data_##data_name          \
                        , sizeof(symspi_test_xfer_data_##data_name));\
                                                                     \
        symspi_test_##test_number##__result &= equal;                \
                                                                     \
        if (!equal) {                                                \
                symspi_test_err("FAILED! See details:");             \
                symspi_test_printout_xfer(done_xfer);                \
                print_hex_dump(KERN_DEBUG                            \
                        , SYMSPI_TEST_LOG_TEST_PREFIX(test_number)   \
                          "FAIL: expected RX: "                      \
                        , 0, 16, 1                                   \
                        , symspi_test_xfer_data_##data_name          \
                        , sizeof(symspi_test_xfer_data_##data_name)  \
                        , true);                                     \
                                                                     \
                SYMSPI_TEST_FINISH_SEQ(test_number, fail_xfer_name); \
        }

#define SYMSPI_TEST_BIND_TO_TEST(xfer_name, test_number)             \
        symspi_test_info("using xfer: "#xfer_name"; TX data: %px"    \
                         ", size: %d"                                \
                         , symspi_test_xfer_##xfer_name.data_tx      \
                         , (int)(symspi_test_xfer_##xfer_name        \
                                        .size_bytes));               \
        symspi_test_xfer_##xfer_name.done_callback                   \
                = symspi_test_##test_number##__xfer_done_callback;

#define SYMSPI_TEST_UNBIND_XFER(xfer_name, test_number)              \
        symspi_test_xfer_##xfer_name.done_callback = NULL

// TODO: verify that xfer is bound to somewhere
#define SYMSPI_TEST_JNEXT_XFER(test_num, phase, xfer_name)           \
        if (symspi_test_##test_num##__xfer_counter  == phase) {      \
                *start_immediately__out = true;                      \
                symspi_test_##test_num##__xfer_counter++;            \
                return &symspi_test_xfer_##xfer_name;                \
        }

#define SYMSPI_TEST_JNEXT_PASSIVE(test, phase, xfer)                 \
        if (symspi_test_##test##__xfer_counter                       \
                        == phase) {                                  \
                *start_immediately__out = false;                     \
                symspi_test_##test##__xfer_counter++;                \
                return &symspi_test_xfer_##xfer;                     \
        }

#define SYMSPI_TEST_VERIFY_JNEXT(test, phase, next_xfer, exp_data, fail_xfer) \
        if (symspi_test_##test##__xfer_counter == phase) {           \
                SYMSPI_TEST_VERIFY_RX(exp_data, test, fail_xfer);    \
                *start_immediately__out = true;                      \
                symspi_test_##test##__xfer_counter++;                \
                return &symspi_test_xfer_##next_xfer;                \
        }

#define SYMSPI_TEST_VERIFY_JNEXT_PASSIVE(test, phase, next_xfer, exp_data, fail_xfer) \
        if (symspi_test_##test##__xfer_counter == phase) {           \
                SYMSPI_TEST_VERIFY_RX(exp_data, test, fail_xfer);    \
                *start_immediately__out = false;                     \
                symspi_test_##test##__xfer_counter++;                \
                return &symspi_test_xfer_##next_xfer;                \
        }

#define SYMSPI_TEST_STANDARD_CALLBACK(test_num)                      \
        struct full_duplex_xfer *                                    \
                symspi_test_##test_num##__xfer_done_callback(        \
                        const struct full_duplex_xfer __kernel *done_xfer \
                        , const int next_xfer_id                     \
                        , bool __kernel *start_immediately__out      \
                        , void *consumer_data)

int symspi_test_run_symspi(struct symspi_dev *symspi)
{
        int res = symspi_init((void*)symspi, &symspi_test_zeroed_1b_xfer);
        if (res < 0) {
                symspi_test_err("Starting of symspi failed. Error %d."
                                , res);
                return -ENODEV;
        }

        return 0;
}

// TODO: extract to independent source
void symspi_test_printout_xfer(const struct full_duplex_xfer *xfer)
{
        symspi_test_info_raw("TX data:");
        print_hex_dump(KERN_DEBUG, "TX data: ", 0, 16, 1
                       , xfer->data_tx, xfer->size_bytes, true);
        symspi_test_info_raw("RX data:");
        print_hex_dump(KERN_DEBUG, "RX data: ", 0, 16, 1
                       , xfer->data_rx_buf, xfer->size_bytes, true);
}

// Returns true if two data blocks are equal, otherwise false
bool symspi_test_packages_equal(const void *data_a, int length_a
                                , const void *data_b, int length_b)
{
        if (length_a != length_b) {
                return false;
        }

        int i;
        for (i = 0; i < length_a; i++) {
                if (*((unsigned char*)data_a)
                                != *((unsigned char*)data_b)) {
                        return false;
                }
        }
        return true;
}

// Returns the timeout to wait for given number of xfers
int symspi_test_get_timeout_ms(int xfers_count)
{
    // every xfer takes not longer than 30 ms
    // + 1000 ms on scheduler delays
    return 30 * xfers_count + 1000;
}


/*-------------------- TESTS SECTIOON ------------------------------*/

//////////////////////////////////////////////////////////////////////
// NOTE: the detection of the test packages is to be disabled in    //
//      production mode                                             //
//////////////////////////////////////////////////////////////////////


// TEST 1
// 64 byte request-answer sequence.
//      * XFER 1
//          * TX: predefined 64 byte request package
//          * RX: irrelevant
//      * XFER 2
//          * TX: default 64 byte package
//          * RX: predefined 1 byte answer package
// if 2nd xfer RX data equals to expected (predefined), then test
// is passed
#define SYMSPI_TEST_1 1

SYMSPI_TEST_DEFINE_TEST(1)
SYMSPI_TEST_STANDARD_CALLBACK(1)
{
        bool equal;

        // we have xfered our request
        SYMSPI_TEST_JNEXT_XFER(1, 1, 64b_default);

        // we have done the answer xfer, verifying the answer
        SYMSPI_TEST_VERIFY_RX(64b_answer, 1, 64b_default);

        SYMSPI_TEST_FINISH_SEQ(1, 64b_default);
}

bool symspi_test_1(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(1);

        SYMSPI_TEST_BIND_TO_TEST(64b_default, 1);
        SYMSPI_TEST_BIND_TO_TEST(64b_request, 1);

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_request
                            , true);

        SYMSPI_TEST_FINALIZE(1, 2);
}



// TEST 2
// 1 byte request-answer sequence.
//      * XFER 1
//          * TX: predefined 64 byte package to request
//                to 1 byte xfer at next step
//          * RX: irrelevant
//      * XFER 2
//          * TX: predefined 1 byte request package
//          * RX: predefined 1 byte answer package
//      * XFER 3
//          * TX: default 64 byte package (irrelevant)
//          * RX: predefined 64 byte answer package:
//                  if at previous step other side got everything
//                  correctly, otherwise 64 byte bad receive package
// if 2nd xfer RX data equals to expected (predefined), then test
// is passed
#define SYMSPI_TEST_2 2

SYMSPI_TEST_DEFINE_TEST(2)
SYMSPI_TEST_STANDARD_CALLBACK(2)
{
        bool equal;

        // we have xfered our 64 byte request to switch to 1 byte xfer
        SYMSPI_TEST_JNEXT_XFER(2, 1, 1b_request);
        // we have xfered our 1 byte request
        SYMSPI_TEST_VERIFY_JNEXT(2, 2, 64b_default, 1b_answer, 64b_default);
        // we have xfered default 64 byte xfer, expecting 64 byte answer
        SYMSPI_TEST_VERIFY_RX(64b_answer, 2, 64b_default);
        SYMSPI_TEST_FINISH_SEQ(2, 64b_default);
}

bool symspi_test_2(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(2);

        SYMSPI_TEST_BIND_TO_TEST(64b_to_1b_xfer_once, 2);
        SYMSPI_TEST_BIND_TO_TEST(1b_request, 2);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 2);

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_to_1b_xfer_once
                            , true);

        SYMSPI_TEST_FINALIZE(2, 3);
}

// TEST 3
// 1000 of TEST 1 (64 byte request - answer)  in heavy load mode
// OK if none of TEST 1 stages failed
#define SYMSPI_TEST_3 3

int symspi_test_3__iterations_counter;
const int symspi_test_3__iterations_total = 1000;

SYMSPI_TEST_DEFINE_TEST(3)
SYMSPI_TEST_STANDARD_CALLBACK(3)
{
        bool equal;

        // we have xfered our request
        SYMSPI_TEST_JNEXT_XFER(3, 1, 64b_default);

        // we have done the answer xfer, verifying the answer
        SYMSPI_TEST_VERIFY_RX(64b_answer, 3, 64b_default);

        if (symspi_test_3__iterations_counter
                        >= symspi_test_3__iterations_total) {
                SYMSPI_TEST_FINISH_SEQ(3, 64b_default);
        }

        // new iteration
        symspi_test_3__iterations_counter++;
        symspi_test_3__xfer_counter = 0;
        SYMSPI_TEST_JNEXT_XFER(3, 0, 64b_request);

        SYMSPI_TEST_FAIL_FINISH_SEQ(3, 64b_default, "test logical error");
}

bool symspi_test_3(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(3);

        SYMSPI_TEST_BIND_TO_TEST(64b_default, 3);
        SYMSPI_TEST_BIND_TO_TEST(64b_request, 3);

        symspi_test_3__iterations_counter = 1;

        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_request
                            , true);

        SYMSPI_TEST_FINALIZE(3, 2 * symspi_test_3__iterations_total);
}

// TEST 4
// 1000 of TEST 2 (1 byte request-answer) in heavy load mode
// OK if none of TEST 2 stages failed
#define SYMSPI_TEST_4 4

int symspi_test_4__iterations_counter;
const int symspi_test_4__iterations_total = 1000;

SYMSPI_TEST_DEFINE_TEST(4)
SYMSPI_TEST_STANDARD_CALLBACK(4)
{
        bool equal;

        // we have xfered our 64 byte request to switch to 1 byte xfer
        SYMSPI_TEST_JNEXT_XFER(4, 1, 1b_request);
        // we have xfered our 1 byte request
        SYMSPI_TEST_VERIFY_JNEXT(4, 2, 64b_default
                                 , 1b_answer, 64b_default);
        // we have xfered default 64 byte xfer, expecting 64 byte answer
        SYMSPI_TEST_VERIFY_RX(64b_answer, 4, 64b_default);

        if (symspi_test_4__iterations_counter
                        >= symspi_test_4__iterations_total) {
                SYMSPI_TEST_FINISH_SEQ(4, 64b_default);
        }

        symspi_test_4__iterations_counter++;
        symspi_test_4__xfer_counter = 0;

        SYMSPI_TEST_JNEXT_XFER(4, 0, 64b_to_1b_xfer_once);

        SYMSPI_TEST_FAIL_FINISH_SEQ(4, 64b_default
                                    , "test logical error");
}

bool symspi_test_4(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(4);

        SYMSPI_TEST_BIND_TO_TEST(64b_to_1b_xfer_once, 4);
        SYMSPI_TEST_BIND_TO_TEST(1b_request, 4);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 4);

        symspi_test_4__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_to_1b_xfer_once
                            , true);

        SYMSPI_TEST_FINALIZE(4, 3 * symspi_test_4__iterations_total);
}

// TEST 5
// (bulk 1 byte xfers, without restoring 64 bytes after each iteration)
//      * switch to permanent 1 byte mode
//          64 byte request ->
//          <- any
//      * iterations count times:
//          * 1 byte default ->
//            <- 1 byte answer (on prev request)
//          * 1 byte request ->
//            <- 1 byte default
//      * restore 64 byte mode
//          restore 64 byte mode ->
//          <- 1 byte answer (on prev request)
//      * final verification
//          64 byte default ->
//          <- 64 byte ack
// OK if none of steps failed
#define SYMSPI_TEST_5 5

int symspi_test_5__iterations_counter;
const int symspi_test_5__iterations_total = 1000;

SYMSPI_TEST_DEFINE_TEST(5)
SYMSPI_TEST_STANDARD_CALLBACK(5)
{
        bool equal;

        SYMSPI_TEST_JNEXT_XFER(5, 1, 1b_default);

        if (symspi_test_5__xfer_counter > 3) {
                if (symspi_test_5__iterations_counter
                                >= symspi_test_5__iterations_total) {
                        // finishing
                        SYMSPI_TEST_VERIFY_JNEXT(5, 4, 1b_to_64b_xfer
                                                 , 1b_answer, 64b_default);
                        SYMSPI_TEST_VERIFY_JNEXT(5, 5, 64b_default
                                                 , 1b_default, 64b_default);
                        SYMSPI_TEST_VERIFY_JNEXT(5, 6, 64b_default
                                                 , 64b_answer, 64b_default);
                        SYMSPI_TEST_FINISH_SEQ(5, 64b_default);
                }

                symspi_test_5__iterations_counter++;
                symspi_test_5__xfer_counter = 2;
        }

        SYMSPI_TEST_VERIFY_JNEXT(5, 2, 1b_request, 1b_answer, 64b_default);
        SYMSPI_TEST_VERIFY_JNEXT(5, 3, 1b_default, 1b_default, 64b_default);

        SYMSPI_TEST_FAIL_FINISH_SEQ(5, 64b_default, "test logical error");
}

bool symspi_test_5(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(5);

        SYMSPI_TEST_BIND_TO_TEST(64b_to_1b_xfer, 5);
        SYMSPI_TEST_BIND_TO_TEST(1b_default, 5);
        SYMSPI_TEST_BIND_TO_TEST(1b_request, 5);
        SYMSPI_TEST_BIND_TO_TEST(1b_to_64b_xfer, 5);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 5);

        symspi_test_5__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_to_1b_xfer
                            , true);

        SYMSPI_TEST_FINALIZE(5, 3 + 2 * symspi_test_5__iterations_total);
}

// TODO:

// TEST 6
// random package size test (from 1 to 64 byte) with data
// approval
//
// * switch to random size request (64 byte) ->
//   <- any (64 byte)
// * random data of chosen size (chosen at RT bytes) ->
//   <- random data of chosen size (chosen at RT bytes)
// * every byte incremented by 1, every 4-bytes-integer incremented by 10,
//      previous random data package (chosen at RT bytes)->
//   <- every byte incremented by 1, every 4-bytes-integer incremented by 10,
//      previous random data package (chosen at RT bytes)
// * if match: answer 64 byte package ->
//   <- if match: answer 64 byte package
// OK if none of steps failed

//#define SYMSPI_TEST_6 6

//int symspi_test_6__iterations_counter;
//const int symspi_test_6__iterations_total = 1000;

//SYMSPI_TEST_DEFINE_TEST(6)
//SYMSPI_TEST_STANDARD_CALLBACK(6)
//{
//}

//bool symspi_test_6(struct symspi_dev *symspi)
//{
//}


// TEST 7
// * request to initiate next 64 byte xfer (64 bytes)->
//   <- any
// * [other side initiated]
//   default 64 byte package ->
//   <- 64 byte answer
// OK if none of steps failed
#define SYMSPI_TEST_7 7

SYMSPI_TEST_DEFINE_TEST(7)
SYMSPI_TEST_STANDARD_CALLBACK(7)
{
        bool equal;

        SYMSPI_TEST_JNEXT_PASSIVE(7, 1, 64b_default);

        // other-side-initiated xfer is done
        SYMSPI_TEST_VERIFY_RX(64b_answer, 7, 64b_default);

        SYMSPI_TEST_FINISH_SEQ(7, 64b_default);
}



bool symspi_test_7(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(7);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_xfer_request, 7);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 7);

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_init_xfer_request
                            , true);

        SYMSPI_TEST_FINALIZE(7, 2);
}


// TEST 8
// 1000x of TEST 7, with no wait from our side
// OK if none of steps failed
#define SYMSPI_TEST_8 8

int symspi_test_8__iterations_counter;
const int symspi_test_8__iterations_total = 1000;

SYMSPI_TEST_DEFINE_TEST(8)
SYMSPI_TEST_STANDARD_CALLBACK(8)
{
        bool equal;

        SYMSPI_TEST_JNEXT_PASSIVE(8, 1, 64b_default);

        if (symspi_test_8__iterations_counter
                        < symspi_test_8__iterations_total) {
                symspi_test_8__xfer_counter = 0;
                symspi_test_8__iterations_counter++;

                SYMSPI_TEST_VERIFY_JNEXT(8, 0, 64b_init_xfer_request
                                         , 64b_answer, 64b_default);
        }

        SYMSPI_TEST_FINISH_SEQ(8, 64b_default);
}

bool symspi_test_8(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(8);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_xfer_request, 8);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 8);

        symspi_test_8__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_init_xfer_request
                            , true);

        SYMSPI_TEST_FINALIZE(8, 2 * symspi_test_8__iterations_total);
}

// TEST 9
// * request to initiate next 1 byte xfer (64 bytes)->
//   <- any
// * [other side initiated]
//   default 1 byte package ->
//   <- 1 byte answer
// OK if none of steps failed
#define SYMSPI_TEST_9 9

SYMSPI_TEST_DEFINE_TEST(9)
SYMSPI_TEST_STANDARD_CALLBACK(9)
{
        bool equal;

        SYMSPI_TEST_JNEXT_PASSIVE(9, 1, 1b_default);

        // other-side-initiated xfer is done
        SYMSPI_TEST_VERIFY_RX(1b_answer, 9, 64b_default);

        SYMSPI_TEST_FINISH_SEQ(9, 64b_default);
}

bool symspi_test_9(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(9);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_1b_xfer_request, 9);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 9);
        SYMSPI_TEST_BIND_TO_TEST(1b_default, 9);

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_init_1b_xfer_request
                            , true);

        SYMSPI_TEST_FINALIZE(9, 2);
}

// TEST 10
// 1000x of TEST 9, with no wait from our side
// OK if none of steps failed
#define SYMSPI_TEST_10 10

int symspi_test_10__iterations_counter;
const int symspi_test_10__iterations_total = 1000;

SYMSPI_TEST_DEFINE_TEST(10)
SYMSPI_TEST_STANDARD_CALLBACK(10)
{
        bool equal;

        SYMSPI_TEST_JNEXT_PASSIVE(10, 1, 1b_default);

        if (symspi_test_10__iterations_counter
                        < symspi_test_10__iterations_total) {
                symspi_test_10__xfer_counter = 0;
                symspi_test_10__iterations_counter++;

                SYMSPI_TEST_VERIFY_JNEXT(10, 0, 64b_init_1b_xfer_request
                                         , 1b_answer, 64b_default);
        }

        SYMSPI_TEST_FINISH_SEQ(10, 64b_default);
}

bool symspi_test_10(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(10);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_1b_xfer_request, 10);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 10);
        SYMSPI_TEST_BIND_TO_TEST(1b_default, 10);

        symspi_test_10__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                            , &symspi_test_xfer_64b_init_1b_xfer_request
                            , true);

        SYMSPI_TEST_FINALIZE(10, 2 * symspi_test_10__iterations_total);
}


// TEST 11
// Switch to their initiated sequential 1000 of 64 xfers with random
// delay times on other side (all xfers after request are initiated only
// by other side).
//
// * Switch to 1000x their initiated xfers (64 bytes, our init)->
//   <- any
// * 500x
//   * 64byte default (64 bytes, their init)->
//     <- 64byte answer
//   * 64byte request  (64 bytes, their init)->
//     <- 64byte default
// * 64byte default  (64 bytes, our init)->
//   <- 64byte 1000x sequence done
// OK if none of steps failed
#define SYMSPI_TEST_11 11

int symspi_test_11__iterations_counter;
const int symspi_test_11__iterations_total = 500;

SYMSPI_TEST_DEFINE_TEST(11)
SYMSPI_TEST_STANDARD_CALLBACK(11)
{
        bool equal;

        // initial request done
        SYMSPI_TEST_JNEXT_PASSIVE(11, 1, 64b_default);

        // 1 done
        SYMSPI_TEST_VERIFY_JNEXT_PASSIVE(11, 2, 64b_request
                                         , 64b_answer, 64b_default);

        // 2 done
        SYMSPI_TEST_VERIFY_JNEXT_PASSIVE(11, 3, 64b_default
                                         , 64b_default, 64b_default);
        // 3 done
        SYMSPI_TEST_VERIFY_JNEXT_PASSIVE(11, 4, 64b_request
                                         , 64b_answer, 64b_default);
        // 4 done
        symspi_test_11__iterations_counter++;

        if (symspi_test_11__iterations_counter
                        >= symspi_test_11__iterations_total) {
                SYMSPI_TEST_VERIFY_JNEXT(11, 5, 64b_default
                                         , 64b_default, 64b_default);

                SYMSPI_TEST_VERIFY_RX(64b_done_1000x_64b_xfer
                                      , 11, 64b_default);
                SYMSPI_TEST_FINISH_SEQ(11, 64b_default);
        } else {
                symspi_test_11__xfer_counter = 3;

                SYMSPI_TEST_VERIFY_JNEXT_PASSIVE(11, 3, 64b_default
                                         , 64b_default, 64b_default);
        }

        SYMSPI_TEST_FAIL_FINISH_SEQ(11, 64b_default, "logical error");
}

bool symspi_test_11(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(11);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_1000x_64b_xfer_request, 11);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 11);
        SYMSPI_TEST_BIND_TO_TEST(64b_request, 11);

        symspi_test_11__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                , &symspi_test_xfer_64b_init_1000x_64b_xfer_request
                , true);

        SYMSPI_TEST_FINALIZE(11, 2 + 2 * symspi_test_11__iterations_total);
}



// TEST 12
// The same as TEST 11, but our side also initiates xfers with random
// delay, so we will have random initialised xfers with random delay
// times of initialization requests.
#define SYMSPI_TEST_12 12

int symspi_test_12__iterations_counter;
const int symspi_test_12__iterations_total = 500;

SYMSPI_TEST_DEFINE_TEST(12)
SYMSPI_TEST_STANDARD_CALLBACK(12)
{
        bool equal;

        // initial request done
        SYMSPI_TEST_JNEXT_XFER(12, 1, 64b_default);

        // 1 done
        SYMSPI_TEST_VERIFY_JNEXT(12, 2, 64b_request
                                 , 64b_answer, 64b_default);

        // 2 done
        SYMSPI_TEST_VERIFY_JNEXT(12, 3, 64b_default
                                 , 64b_default, 64b_default);
        // 3 done
        SYMSPI_TEST_VERIFY_JNEXT(12, 4, 64b_request
                                 , 64b_answer, 64b_default);
        // 4 done
        symspi_test_12__iterations_counter++;

        if (symspi_test_12__iterations_counter
                        >= symspi_test_12__iterations_total) {
                SYMSPI_TEST_VERIFY_JNEXT(12, 5, 64b_default
                                         , 64b_default, 64b_default);

                SYMSPI_TEST_VERIFY_RX(64b_done_1000x_64b_xfer
                                      , 12, 64b_default);
                SYMSPI_TEST_FINISH_SEQ(12, 64b_default);
        } else {
                symspi_test_12__xfer_counter = 3;

                SYMSPI_TEST_VERIFY_JNEXT(12, 3, 64b_default
                                         , 64b_default, 64b_default);
        }

        SYMSPI_TEST_FAIL_FINISH_SEQ(12, 64b_default, "logical error");
}

bool symspi_test_12(struct symspi_dev *symspi)
{
        SYMSPI_TEST_INITIATE_TEST_ACTIONS(12);

        SYMSPI_TEST_BIND_TO_TEST(64b_init_1000x_64b_xfer_request, 12);
        SYMSPI_TEST_BIND_TO_TEST(64b_default, 12);
        SYMSPI_TEST_BIND_TO_TEST(64b_request, 12);

        symspi_test_12__iterations_counter = 1;

        // TODO: may fail due to xfer requested from other side
        // FIXME
        symspi_data_xchange((void*)symspi
                , &symspi_test_xfer_64b_init_1000x_64b_xfer_request
                , true);

        SYMSPI_TEST_FINALIZE(12, 2 + 2 * symspi_test_12__iterations_total);
}


/*----------------------------- MAIN -------------------------------*/

static bool symspi_test_exiting = false;

struct full_duplex_xfer * symspi_test_xfer_done_default_callback(
                        const struct full_duplex_xfer __kernel *done_xfer
                        , const int next_xfer_id
                        , bool __kernel *start_immediately__out
                        , void *consumer_data)
{
        symspi_test_info_raw("default callback: xfer done");
        symspi_test_printout_xfer(done_xfer);

        // gracefull shutdown path
        if (symspi_test_exiting) {
                return ERR_PTR(-ENODATA);
        }

        return NULL;
}


/*-------------------- MODULE HOUSEKEEPING -------------------------*/

extern struct symspi_dev *symspi_get_global_device(void);

extern int symspi_data_xchange(void __kernel *device
                , struct __kernel full_duplex_xfer *xfer
                , bool force_size_change);
extern int symspi_default_data_update(void __kernel *device
                , struct full_duplex_xfer *xfer
                , bool force_size_change);
extern int symspi_init(void __kernel *device
                , struct full_duplex_xfer *default_xfer);
extern int symspi_close(void __kernel *device);
extern bool symspi_is_running(void __kernel *device);
extern int symspi_reset(void __kernel *device
                , struct full_duplex_xfer *default_xfer);


struct symspi_test_test {
        bool (*routine)(struct symspi_dev *);
        char *name;
        bool result;
};

// If you add a new test, add it here
static struct symspi_test_test symspi_test_tests[] = {
        { symspi_test_1, "TEST 1: 64 byte request-answer", false }
        , { symspi_test_2, "TEST 2: 64 byte - 1 byte - 64 byte", false }
        , { symspi_test_3, "TEST 3: 1000x heavy load of:"
                           " 64 byte request-answer", false }
        , { symspi_test_4, "TEST 4: 1000x heavy load of:"
                           " 64 byte - 1 byte - 64 byte", false }
        , { symspi_test_5, "TEST 5: switch to 1 byte mode"
                           ", 1000x of: 1 byte request-answer"
                           ", switch back to 64 byte mode", false }
        , { symspi_test_7, "TEST 7: single xfer initiated by other"
                           " side (upon our previous request)", false }
        , { symspi_test_8, "TEST 8: 1000x of other side initiated xfers"
                           " (no wait from our side)", false }
        , { symspi_test_9, "TEST 9: single 1 byte xfer initiated by other"
                           " side (upon our previous request)", false }
        , { symspi_test_10
            , "TEST 10: 1000x of 1 byte xfer initiated by other"
              " side (upon our previous request every time)"
            , false }
        , { symspi_test_11
            , "TEST 11: 1000x of 64 byte xfer initiated by other"
              " side (upon our single request)", false }
        , { symspi_test_12
            , "TEST 12: 1000x of 64 byte xfer initiated together"
              " (both sides run to trigger initialization)"
            , false }
};

static void symspi_test_configure_symspi(struct symspi_dev *symspi)
{
        // adjusting SPI mode

        // sets the single brust size in bits (amount of bits to be
        // xfered at once between SS raise and SS drop);
        // NOTE: right now works only up to 32 bits (probably as long
        //      as fifo on iMX is of 32bits width); (depth of fifo is
        //      64 entries);
        // WARNING: the spi device adjusts the bit order according to
        //      the word size, so if you set 32 bit word size, then
        //      it will send the most significant of 32 bits first
        //      then less, less and so on to the less significant one.
        //      Which will cause the byte order inversion in groups
        //      of 4 bytes.
        //symspi->spi->bits_per_word = 32;
        //
        // spi_device
        //#define	SPI_CPHA	0x01			/* clock phase */
        //#define	SPI_CPOL	0x02			/* clock polarity */


        //  * when SS disabled, clock is HI
        //  * data is valid on clock change from LO -> HI
        symspi->spi->bits_per_word = 8;
        symspi->spi->mode |= (SPI_CPOL | SPI_CPHA);
        symspi->spi->master->setup(symspi->spi);
}

static void symspi_test_print_results(void)
{
        symspi_test_info_raw("========= SYMSPI TEST RESULTS =========");

        int tests_count = ARRAY_SIZE(symspi_test_tests);
        int failed_count = 0;
        int i;
        for (i = 0; i < tests_count; i++) {
                if (symspi_test_tests[i].result) {
                        symspi_test_info_raw("test[%d]:     OK: %s", i
                                             , symspi_test_tests[i].name);
                } else {
                        symspi_test_err_raw("test[%d]:     FAILED!: %s"
                                            , i, symspi_test_tests[i].name);
                        failed_count++;
                }
        }

        if (failed_count == 0) {
                symspi_test_info_raw("ALL TESTS PASSED.");
        } else {
                symspi_test_err_raw("%d/%d TESTS FAILED", failed_count
                                    , tests_count);
        }
}

// RETURNS:
//      0 if all tests passed,
//      or negated first failed test index
static int __init symspi_test_module_init(void)
{
        symspi_test_info("loading module");

        struct symspi_dev *symspi = symspi_get_global_device();

        if (IS_ERR_OR_NULL(symspi)) {
                symspi_test_err("global symspi device unavailable"
                                ", err: %ld" , PTR_ERR(symspi));
                return -ENODEV;
        }
        symspi_test_info_raw("symspi device: %px", symspi);

        symspi_test_configure_symspi(symspi);

        if (symspi_test_run_symspi(symspi) != 0) {
                symspi_test_err("Symspi loading failed. Abort.");
                return -ENODEV;
        }

        symspi_test_info("symspi inited");
        symspi_test_info("starting tests...");

        if (sizeof(symspi_test_tests) == 0) {
                symspi_test_err("No tests to run!");
                return -1;
        }

        // run all decladed tests
        int i;
        int tests_count = ARRAY_SIZE(symspi_test_tests);
        int failed_count = 0;
        for (i = 0; i < tests_count ; i++) {
                symspi_test_info("starting test: [%d]", (i + 1));
                if (!(symspi_test_tests[i].routine(symspi))) {
                        symspi_test_err("test %d failed, see dmesg"
                                        " for details.", (i + 1));
                        symspi_test_tests[i].result = false;
                        failed_count++;
                } else {
                        symspi_test_tests[i].result = true;
                }
                // to distinguish between tests in logic diagram
                msleep(500);
        }

        symspi_test_print_results();

        return failed_count > 0 ? -failed_count : 0;
}

static void __exit symspi_test_module_exit(void)
{
        symspi_test_info_raw("module unloaded");
}

module_init(symspi_test_module_init);
module_exit(symspi_test_module_exit);

MODULE_DESCRIPTION("Module for testing symmetrical SPI communication.");
MODULE_AUTHOR("Artem Gulyaev <Artem.Gulyaev@bosch.com>");
MODULE_LICENSE("GPL v2");
