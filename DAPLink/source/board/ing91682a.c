/**
 * @file    xDot.c
 * @brief   board ID for the MultiTech xDot
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
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

extern target_cfg_t target_device_ing916;

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "0x240",
    .family_id = kIngchips_Ing916_FamilyID,
    .flags = kEnablePageErase|kEnableUnderResetConnect,
    .daplink_url_name = "INGCHIPSHTM",
    .daplink_drive_name = "INGCHIPS",
    .daplink_target_url = "https://ingchips.github.io",
    .target_cfg = &target_device_ing916,
    .board_vendor = "INGCHIPS",
    .board_name = "ING916",
};
