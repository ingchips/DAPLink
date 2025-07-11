/**
 * @file    target_reset.c
 * @brief   Target reset for the new target
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

#include "target_family.h"
#include "swd_host.h"

static uint8_t validate_bin_nvic(const uint8_t *buf)
{
    return 1;
}

static const target_family_descriptor_t ing918xx_family = {
    .family_id = 0,
    .default_reset_type = kSoftwareReset,
    .soft_reset_type = SYSRESETREQ,
    .validate_bin_nvic = validate_bin_nvic,
};

const target_family_descriptor_t *g_target_family = &ing918xx_family;
