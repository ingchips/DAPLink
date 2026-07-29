/**
 * @file    ing916xx.c
 * @brief   board information for INGCHIPS ING918xx dev board
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

#include "target_family.h"
#include "target_board.h"


const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = {'0', 'x', '2', '4', '0'},
    .family_id = kIngchips_Ing916_FamilyID,
    .flags = kEnablePageErase|kEnableUnderResetConnect,
    .daplink_url_name = {'I', 'N', 'G', '9', '1', '6', 
        ' ', ' ',  'H', 'T', 'M'},
    .daplink_drive_name = {'I', 'N', 'G', '9', '1', '6', 
        ' ', ' ', ' ', ' ', ' '},
    .daplink_target_url = "https://ingchips.github.io",
    .target_cfg = &target_device,
    .board_vendor = "INGCHIPS",
    .board_name = "ING916xx-Dev",
};
