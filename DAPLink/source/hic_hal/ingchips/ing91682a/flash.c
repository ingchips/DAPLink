/**
 * @file    flash.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2017-2017, ARM Limited, All Rights Reserved
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

#include <string.h>
#include "flash_hal.h"
#include "util.h"
#include "eflash.h"


uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    return 0; // No init needed
}

uint32_t UnInit(uint32_t fnc)
{
    return 0; // No init needed
}

uint32_t EraseChip(void)
{
    return (1); // IAP not supported
}

uint32_t EraseSector(uint32_t adr)
{
    return erase_flash_sector(adr);
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    return program_flash(adr, (const uint8_t*)buf, sz);
}

