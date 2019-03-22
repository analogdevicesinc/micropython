/*
 * Copyright (c) 2018 Analog Devices, Inc.  All rights reserved.
 */

#include "../bm_adau_device.h"

// ADAU1761 Master I2S/50% 8 Channel Configuration
uint8_t ADAU1761_8ch_i2s_master_TxBuffer[] = {
#include "ss_schematics/ADAU1761_ADAU1761_8ch_i2s_master/Exported_init_files/TxBuffer_ADAU1761.dat"
};

uint16_t ADAU1761_8ch_i2s_master_NumBytes[] = {
#include "ss_schematics/ADAU1761_ADAU1761_8ch_i2s_master/Exported_init_files/NumBytes_ADAU1761.dat"
};

BM_ADAU_DEVICE_INIT_DATA adau1761_8ch_i2s_master = {.data_tx_buffer = ADAU1761_8ch_i2s_master_TxBuffer,
                                                    .data_num_bytes = ADAU1761_8ch_i2s_master_NumBytes,
                                                    .total_lines = sizeof(ADAU1761_8ch_i2s_master_NumBytes) / sizeof(uint16_t),
                                                    .ignore_first_byte_of_init_file = false};
