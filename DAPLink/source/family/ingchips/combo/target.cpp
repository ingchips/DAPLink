/**
 * @file    target.c
 * @brief   Target information for the nRF52 Family
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2021, Arm Limited, All Rights Reserved
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
#include "target_config.h"
}

namespace ing916xx
{
    #include "../ing916xx/target.c"
}

namespace ing918xx
{
    #include "../ing918xx/target.c"
}

// target information for model with 32 KB RAM
extern "C" target_cfg_t target_device = ing916xx::target_device;

extern "C" void ingchips_combo_targets_select(int index)
{
    switch (index)
    {
    case 0:
        target_device = ing918xx::target_device;
        break;
    case 1:
        target_device = ing916xx::target_device;
        break;
    default:
        break;
    }
}
