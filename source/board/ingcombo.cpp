/**
 * @file    ingcombo.c
 * @brief   Virtual "combo" board information for INGCHIPS ING918xx/ING916xx dev boards
 *
 * Copyright (c) 2017-2019, ARM Limited, All Rights Reserved
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

extern "C"
{
#include "target_family.h"
#include "target_board.h"

extern void ingchips_combo_targets_select(int index);
}

namespace ing916xx
{
    #include "ing916xx.c"

    extern target_cfg_t target_device;
}

namespace ing918xx
{
    #include "ing918xx.c"

    extern target_cfg_t target_device;
}

static const board_info_t def_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "0000",
    .family_id = kStub_SWVectReset_FamilyID,
};

extern "C" void ingchips_combo_boards_select(int index, uint32_t app_addr)
{
    board_info_t *p_board = (board_info_t *)&g_board_info;
    switch (index)
    {
    case 0:
        ingchips_combo_targets_select(0);
        *p_board = ing918xx::g_board_info;
        break;
    case 1:
        ingchips_combo_targets_select(1);
        *p_board = ing916xx::g_board_info;
        break;
    default:
        *p_board = def_board_info;
        break;
    }
}

// g_board_info must be in RAM
extern "C" const board_info_t __attribute__((section("ram_func"))) g_board_info = 
{
};
