/**
 * @file    gpio.c
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

#include "gpio.h"
#include "compiler.h"
#include "IO_Config.h"
#include "settings.h"

#if 0
static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}
#endif
void gpio_init(void)
{
     SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ITEM_APB_SysCtrl)
                                    | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO1)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO0));
                                

    PINCTRL_SetPadMux(LED_HID_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_HID_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_HID_PIN, 0);
    
    PINCTRL_SetPadMux(LED_MSC_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_MSC_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_MSC_PIN, 0);
    
    PINCTRL_SetPadMux(LED_CDC_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_CDC_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_CDC_PIN, 0);
    
    PINCTRL_SetPadMux(LED_PWR_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_PWR_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_PWR_PIN, 0);
    
    /////////////////
    PINCTRL_SetPadMux(SWCLK_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(SWCLK_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(SWCLK_PIN, 1);
    
    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(SWDIO_PIN, GIO_DIR_BOTH);
    PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);
    GIO_WriteValue(SWDIO_PIN, 1);
    
    PINCTRL_SetPadMux(LED_CONNECTED_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_CONNECTED_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_CONNECTED_PIN, 0);
    
    #if LED_RUNNING_PIN
    PINCTRL_SetPadMux(LED_RUNNING_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_RUNNING_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_RUNNING_PIN, 0);
    #endif
    /////////////////
    
    #ifdef nRESET_PIN
    PINCTRL_SetPadMux(nRESET_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(nRESET_PIN, GIO_DIR_BOTH);
    PINCTRL_Pull(nRESET_PIN, PINCTRL_PULL_UP);
    GIO_WriteValue(nRESET_PIN, 1);
    #endif

}

void gpio_set_debug_line(uint8_t val)
{
    #if 0
    GIO_WriteValue((GIO_Index_t)LED_PWR_PIN,val);
    #endif
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    GIO_WriteValue(LED_HID_PIN, state);
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    GIO_WriteValue(LED_CDC_PIN, state);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    GIO_WriteValue(LED_MSC_PIN, state);
}

uint8_t gpio_get_reset_btn_no_fwrd()
{
    return 0;
}

uint8_t gpio_get_reset_btn_fwrd()
{
    #ifdef nRESET_PIN
    return GIO_ReadValue(nRESET_PIN) ? 0 : 1;
    #else
    return 0;
    #endif
}

void gpio_set_board_power(bool powerEnabled)
{
    if (powerEnabled) {
        // enable power switch
        GIO_WriteValue(LED_PWR_PIN, 1);
    }
    else {
        // disable power switch
        GIO_WriteValue(LED_PWR_PIN, 0);
    }
}
