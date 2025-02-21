/**
 * @file    IO_Config.h
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


#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__


#include "ingsoc.h"
#include "ing91682a.h"
#include "daplink.h"
#include "soc.h"

// This GPIO configuration is only valid for the ING91682A HIC
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_ING91682A);

//SWD
#define SWCLK_PIN                    GIO_GPIO_7
#define SWDIO_PIN                    GIO_GPIO_8
//the reset pin muset be defined if you have hardware reset pin
//if you donet have reset pin please set software reset commond
#define nRESET_PIN                   GIO_GPIO_9

// Debug Unit LEDs

// Connected LED
#define LED_CONNECTED_PIN            GIO_GPIO_10

// Target Running LED
//#define LED_RUNNING_PIN              0

// HID LED
#define LED_HID_PIN           GIO_GPIO_13

// MSC LED
#define LED_MSC_PIN           GIO_GPIO_14

// CDC LED
#define LED_CDC_PIN           GIO_GPIO_15

// POWER LED
#define LED_PWR_PIN           GIO_GPIO_18

#define PIN_UART_RX   GIO_GPIO_11
#define PIN_UART_TX   GIO_GPIO_12

#define CDC_UART_BASE  APB_UART1
#define CDC_UART_IRQ   IRQn_Uart1

#define PIN_UART_RTS   GIO_GPIO_19
#define PIN_UART_DTR   GIO_GPIO_20

#endif
