/**
 * @file    ing91682a.h
 * @brief
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

#ifndef ING91682A_H
#define ING91682A_H

#include <stdio.h>
#include <string.h>
#include "ingsoc.h"

#define BSP_DEBUG_BSP_EVENT_ISR     ((uint32_t)0xF0 << 24)
#define BSP_DEBUG_BSP_EP0_STATE     ((uint32_t)0xF1 << 24)
#define USBD_CORE_DEBUG_EP0_EVENT   ((uint32_t)0xF2 << 24)
#define USBD_CORE_DEBUG_DATA_IN     ((uint32_t)0xF3 << 24)
#define BSP_DEBUG_BSP_WRITE_EP      ((uint32_t)0xF4 << 24)
#define BSP_DEBUG_BSP_SET_CONFIG    ((uint32_t)0xF5 << 24)
#define BSP_DEBUG_ADDR_4BYTE        ((uint32_t)0xF6 << 24)
#define BSP_DEBUG_BSP_READ_EP       ((uint32_t)0xF7 << 24)
#define USBD_MSC_OUT                ((uint32_t)0xF8 << 24)
#define USBD_MSC_IN                 ((uint32_t)0xF9 << 24)
#define BSP_DEBUG_BSP_STALL         ((uint32_t)0xFA << 24)
#define USBD_ASSERT                 ((uint32_t)0xFB << 24)
#define USB_DRIVER_ISR              ((uint32_t)0xFC << 24)

#define EP0_IN_INT                  ((uint32_t)0xE0 << 24)
#define EP0_OUT_INT                 ((uint32_t)0xE1 << 24)
#define USBD_CDC_IN                 ((uint32_t)0xE2 << 24)
#define USBD_CDC_OUT                ((uint32_t)0xE3 << 24)
#define USBD_CDC_ACM_DATASEND       ((uint32_t)0xE4 << 24)

extern uint32_t SystemCoreClock; 

extern void BSP_DEBUG_HISTORY(uint32_t data, uint32_t mask);

#endif
