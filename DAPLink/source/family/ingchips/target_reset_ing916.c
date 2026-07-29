/**
 * @file    target_reset_ing916.c
 * @brief   Target reset for the ing916
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

#include "DAP_config.h"
#include "target_family.h"
#include "target_board.h"
#include "swd_host.h"

#if 0
static void target_before_init_debug(void)
{
    // any target specific sequences needed before attaching
    //	to the DAP across JTAG or SWD
    return;
}

static uint8_t target_unlock_sequence(void)
{
    // if the device can secure the flash and there is a way to
    //	erase all it should be implemented here.
    return 1;
}

static uint8_t target_set_state(target_state_t state)
{
    // if a custom state machine is needed to set the TARGET_RESET_STATE state
    return 1;
}

static uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
{
    // if there are security bits in the programmable flash region
    //	a check should be performed. This method is used when programming
    //	by drag-n-drop and should refuse to program an image requesting
    //	to set the device security. This can be performed with the debug channel
    //	if needed.
    return 0;
}
#endif
const target_family_descriptor_t g_ingchips_ing916 = {
    .family_id = kIngchips_Ing916_FamilyID,
    .default_reset_type = kSoftwareReset,
    .soft_reset_type = VECTRESET,
};

/*
The target family api is located in `source/target/target_family.h` and target_reset file can customize the function apis according to the family specification. 
Family id is a combination of vendor id and an incrementing id. There are predefined family id stubs that can be used for generic reset types; 
`kStub_HWReset_FamilyID`, `kStub_SWVectReset_FamilyID`, and `kStub_SWSysReset_FamilyID`.
*/
