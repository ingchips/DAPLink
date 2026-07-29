/**
 * @file    flash_decoder.c
 * @brief   Implementation of flash_decoder.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys_init.h"
#include "flash_decoder.h"
#include "util.h"
//#include "daplink.h"
#include "flash_manager.h"
//#include "target_config.h"  // for target_device
//#include "settings.h"       // for config_get_automation_allowed
#include "validation.h"
//#include "target_board.h"
#include "cmsis_compiler.h"
#include "eflash.h"

// Set to 1 to enable debugging
#define DEBUG_FLASH_DECODER     0

#if DEBUG_FLASH_DECODER
#include "daplink_debug.h"
#define flash_decoder_printf    debug_msg
#else
#define flash_decoder_printf(...)
#endif

typedef enum {
    DECODER_STATE_CLOSED,
    DECODER_STATE_OPEN,
    DECODER_STATE_ERROR
} decoder_state_t;

static decoder_state_t state = DECODER_STATE_CLOSED;
static flash_decoder_type_t flash_type;
static bool data_written;

#define APP_FLASH_SECTOR_COUNT ((APP_END_ADDR - APP_ADDR) / EFLASH_ERASABLE_SIZE)
static uint8_t erased_sectors[(APP_FLASH_SECTOR_COUNT + 7U) / 8U];

__WEAK uint8_t board_detect_incompatible_image(const uint8_t *data, uint32_t size)
{
    (void)data;
    (void)size;
    return 0;   // Return 0 if image is compatible
}

uint8_t validate_bin_nvic(const uint8_t *buf)
{
    uint32_t initial_sp;
    uint32_t reset_handler;
    uint32_t reset_addr;

    memcpy(&initial_sp, buf, sizeof(initial_sp));
    memcpy(&reset_handler, buf + sizeof(initial_sp), sizeof(reset_handler));
    reset_addr = reset_handler & ~1UL;

    return ((initial_sp & 0x7UL) == 0U) &&
           (initial_sp >= APP_RAM_START) && (initial_sp <= APP_RAM_END) &&
           ((reset_handler & 1UL) != 0U) &&
           (reset_addr >= APP_ADDR) && (reset_addr < APP_END_ADDR);
}

uint8_t validate_hexfile(const uint8_t *buf)
{
    // look here for known hex records
    // add hex identifier b[0] == ':' && b[8] == {'0', '2', '3', '4', '5'}
    return ((buf[0] == ':') && ((buf[8] == '0') || (buf[8] == '2') || (buf[8] == '3') || (buf[8] == '4') || (buf[8] == '5'))) ? 1 : 0;
}

flash_decoder_type_t flash_decoder_detect_type(const uint8_t *data, uint32_t size, uint32_t addr, bool addr_valid)
{
    (void)addr;
    (void)addr_valid;

    if ((data == NULL) || (size < 16U)) {
        return FLASH_DECODER_TYPE_UNKNOWN;
    }

    // Check if a valid vector table for the target can be found
    if (validate_bin_nvic(data)) {
        return FLASH_DECODER_TYPE_TARGET;
    }
    
//    if (validate_hexfile(data)) {
//        if(!addr_valid){ //binary is a bin type
//            flash_type_target_bin = true;
//        }
//        return FLASH_DECODER_TYPE_TARGET;
//    }

    return FLASH_DECODER_TYPE_UNKNOWN;
}
error_t flash_decoder_get_flash(flash_decoder_type_t type, uint32_t addr, bool addr_valid, uint32_t *start_addr, const flash_intf_t **flash_intf)
{
    error_t status = ERROR_SUCCESS;
    (void)type;
    (void)addr;
    (void)addr_valid;

    *start_addr = APP_ADDR;
    *flash_intf = NULL;

    return status;
}

error_t flash_decoder_open(void)
{
    flash_decoder_printf("flash_decoder_open()\r\n");

    // Stream must not be open already
    if (state != DECODER_STATE_CLOSED) {
        // util_assert(0);
        return ERROR_INTERNAL;
    }

    memset(erased_sectors, 0, sizeof(erased_sectors));
    state = DECODER_STATE_OPEN;
    flash_type = FLASH_DECODER_TYPE_UNKNOWN;
    data_written = false;
    return ERROR_SUCCESS;
}

error_t flash_decoder_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t prg_size;
    uint32_t current_addr = addr;
    const uint8_t *current_data = data;
    uint32_t size_left = size;

    if (state != DECODER_STATE_OPEN) {
        return ERROR_INTERNAL;
    }
    if ((data == NULL) || (size == 0U) || (addr < APP_ADDR) ||
            (addr >= APP_END_ADDR) || (size > (APP_END_ADDR - addr))) {
        state = DECODER_STATE_ERROR;
        return ERROR_FILE_BOUNDS;
    }

    while (size_left > 0U) {
        uint32_t sector_addr = current_addr & ~(EFLASH_ERASABLE_SIZE - 1U);
        uint32_t sector_index = (sector_addr - APP_ADDR) / EFLASH_ERASABLE_SIZE;
        uint8_t sector_mask = (uint8_t)(1U << (sector_index & 7U));

        prg_size = EFLASH_PAGE_SIZE - (current_addr & (EFLASH_PAGE_SIZE - 1U));
        prg_size = MIN(prg_size, size_left);

        if ((erased_sectors[sector_index >> 3] & sector_mask) == 0U) {
            if (erase_flash_sector(sector_addr) != 0) {
                state = DECODER_STATE_ERROR;
                return ERROR_ERASE_SECTOR;
            }
            erased_sectors[sector_index >> 3] |= sector_mask;
        }

        {
            int write_status = write_flash(current_addr, current_data, prg_size);
            if (write_status != 0) {
                state = DECODER_STATE_ERROR;
                return (write_status == 2) ? ERROR_WRITE_VERIFY : ERROR_WRITE;
            }
        }

        current_addr += prg_size;
        current_data += prg_size;
        size_left -= prg_size;
    }

    flash_type = FLASH_DECODER_TYPE_TARGET;
    data_written = true;
    return ERROR_SUCCESS;
}

error_t flash_decoder_close(void)
{
    error_t status = ERROR_SUCCESS;
    decoder_state_t prev_state = state;
    flash_decoder_printf("flash_decoder_close()\r\n");

    if (DECODER_STATE_CLOSED == state) {
        // util_assert(0);
        return ERROR_INTERNAL;
    }

    state = DECODER_STATE_CLOSED;

    if ((DECODER_STATE_OPEN != prev_state) || !data_written ||
            (flash_type != FLASH_DECODER_TYPE_TARGET)) {
        status = ERROR_IAP_UPDT_INCOMPLETE;
    }

    return status;
}
