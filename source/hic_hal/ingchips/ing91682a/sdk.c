/**
 * @file    sdk.c
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

#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "util.h"
#include "common.h"
#include "ingsoc.h"
#include "eflash.h"

uint32_t SystemCoreClock;

void NVIC_SetVectorTable(int NVIC_VectTab, int Offset)
{ 
    /* Check the parameters */ 
    SCB->VTOR = NVIC_VectTab | (Offset & (int)0xFFFFFF80);
}

#define PRINT_PORT    APB_UART0

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(PRINT_PORT, &config, 0);
}

void SysInit(void)
{
  if(aon2_ctrl_reg->pwr_ctrl_status0.f.boot_power_up == 0x1){
    aon1_ctrl_reg->aon1_reg3.f.reg_boot_pin_clr = 0x1;
    aon1_ctrl_reg->aon1_boot.r = ((0x1 << 0 ) | //BootConfig Enable
                                  (0x1 << 1 ) | //Pll Enable
                                  (0x1 << 2 ) | //Pll wait time or lock, 0:time, 1:lock
                                  (0x0 << 3 ) | //pll time, 110us/130us/150us/170us
                                  (80  << 5 ) | //pll div loop reg
                                  (0x1 << 13) | //hclk sel
                                  (4   << 14) | //hclk div denom
                                  (0x1 << 18) | //flash clk sel
                                  (2   << 19) | //flash div denom
                                  (0x1 << 23) | //flash 4line
                                  (0x2 << 24) | //flash sample delay
                                  (0x1 << 27) | //cache enable
                                  (0x0 << 28) | //wdt enable
                                  (7UL << 29)   //other, must be 0x7
                                 );
    NVIC_SystemReset();
  }
  
  //open clock
  //timer0
  sys_ctrl_reg->cgu_cfg3.f.apb_clk_timer0_gate = 0x1;
  sys_ctrl_reg->cgu_cfg5.f.fclk_timer0_gate = 0x1;

  //iomux & gpio
  sys_ctrl_reg->cgu_cfg3.f.apb_clk_iomux_gate = 0x1;
  sys_ctrl_reg->cgu_cfg3.f.apb_clk_gpio0_gate = 0x1;
  sys_ctrl_reg->cgu_cfg3.f.apb_clk_gpio1_gate = 0x1;

  extern uint32_t __Vectors;
  NVIC_SetVectorTable(0x0, (uint32_t)&__Vectors);

  SystemCoreClock = SYSCTRL_GetPLLClk();

  config_uart(OSC_CLK_FREQ, 921600);
  printf("starting up: %u\n", SYSCTRL_GetHClk());
}
__WEAK
void ingchips_combo_boards_select(int index, uint32_t app_addr)
{
}

extern int uart_detect_target(
  uint8_t *platform_version_found,
  uint16_t *ver_major, uint8_t *ver_minor, uint8_t *ver_patch,
  uint32_t *app_addr);

static uint8_t platform_version_found = 0;
static uint16_t ver_major = 0;
static uint8_t ver_minor = 0;
static uint8_t ver_patch = 0;
static uint32_t app_addr = 0;

#include "DAP_config.h"

void debug_gpio_init(void);
void sdk_init()
{
//    NVIC_SetPriority(SysTick_IRQn, 0);
    SysInit();
    flash_prepare_factory_data();

    int target_id = uart_detect_target(&platform_version_found, &ver_major, &ver_minor, &ver_patch, &app_addr);

    if (platform_version_found)
        printf("Target version: v%d.%d.%d", ver_major, ver_minor, ver_patch);

    if (target_id < 0) {
      target_id = 1;
      app_addr = 0x2002000;
    }
    ingchips_combo_boards_select(target_id, app_addr);
//    debug_gpio_init();
}

void debug_gpio_init(void)
{
    SYSCTRL_ClearClkGateMulti( (1 << SYSCTRL_ITEM_APB_GPIO1)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO0));
    
    
    PINCTRL_SetPadMux(GIO_GPIO_28, IO_SOURCE_GENERAL);
    GIO_SetDirection(GIO_GPIO_28, GIO_DIR_OUTPUT);
    GIO_WriteValue(GIO_GPIO_28, 0);
    PINCTRL_SetPadMux(GIO_GPIO_27, IO_SOURCE_GENERAL);
    GIO_SetDirection(GIO_GPIO_27, GIO_DIR_OUTPUT);
    GIO_WriteValue(GIO_GPIO_27, 0);
}


void debug_di(void)
{
    GIO_WriteValue(GIO_GPIO_28,1);
    GIO_WriteValue(GIO_GPIO_28,0);
}

void debug_do(void)
{
    GIO_WriteValue(GIO_GPIO_27,1);
    GIO_WriteValue(GIO_GPIO_27,0);
}
