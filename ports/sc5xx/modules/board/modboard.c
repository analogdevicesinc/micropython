/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// This module relies on machine module

#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/objtuple.h"
#include "py/objstr.h"
#include "lib/timeutils/timeutils.h"

#include <sys/platform.h>
#include <sys/adi_core.h>

#include "py/mphal.h"
#include "bm_adau_device.h"
#include "bm_sru.h"
#include "mphalport.h"

BM_ADAU_DEVICE adau1761_local;

void board_init(void) {
    BM_ADAU_RESULT adau1761_result;

    // Configure the DAI / SRU to use the ADAU1761 as an I2S clock/FS master to SPORT 0 and A2B to SPORT 1
    sru_config_sharc_sam_a2b_master();

    // Configure SPDIF to connect to SPORT2.  Divide the fs
    sru_config_spdif(4);

    // Initialize ADAU1761
    if ((adau1761_result = adau_initialize(&adau1761_local,
                                           TWI0,
                                           SAM_ADAU1761_I2C_ADDR,
                                           &adau1761_8ch_i2s_master,
                                           ADAU1761_ADDR_BYTES) != ADAU_SUCCESS)) {

        if (adau1761_result == ADAU_CORRUPT_INIT_FILE) {
            mp_hal_stdout_tx_str("ADAU1761 failed to initialize properly due to a corrupt I2C initialization file\r\n");
        }
        else if (adau1761_result == ADAU_TWI_TIMEOUT_ERROR) {
            mp_hal_stdout_tx_str("ADAU1761 failed to initialize due to an I2C timeout during initialization\r\n");
        }
        else if (adau1761_result == ADAU_PLL_LOCK_TIMEOUT_ERROR) {
            mp_hal_stdout_tx_str("ADAU1761 failed to initialize because its PLL failed to lock\r\n");
        }
        else if (adau1761_result == ADAU_SIMPLE_ERROR) {
            mp_hal_stdout_tx_str("ADAU1761 failed to initialize because an initialization error occurred\r\n");
        }
    }
    else {
        mp_hal_stdout_tx_str("ADAU1761 successfully initialized over I2C\r\n");
    }

    // Confirm that the ADAU1761 is running (a good indication that it has been initialized properly)
    uint8_t sigmadspRunning;
    adau_read_ctrl_reg(&adau1761_local, ADAU1761_REG_DSP_RUN, &sigmadspRunning);
    if (!(sigmadspRunning & 0x1)) {
        mp_hal_stdout_tx_str("The SigmaDSP core inside the ADAU1761 is not running\r\n");
    }
}

void board_deinit(void) {
    // Nothing.
}

STATIC const mp_rom_map_elem_t board_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_board) },
};

STATIC MP_DEFINE_CONST_DICT(board_module_globals, board_module_globals_table);

const mp_obj_module_t mp_module_board = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&board_module_globals,
};
