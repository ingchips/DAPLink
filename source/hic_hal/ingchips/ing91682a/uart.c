/**
 * @file    uart.c
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

#include "uart.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"
#include "settings.h"
#include "cortex_m.h"
#include <stdio.h>
#include "cmsis_os2.h"
#define  UART_DEBUG

#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define READ_BUFFER_SIZE         (1024*4)
#define WRITE_BUFFER_SIZE         (1024*4)

circ_buf_t write_buffer;
uint8_t write_buffer_data[WRITE_BUFFER_SIZE];
circ_buf_t read_buffer;
uint8_t read_buffer_data[READ_BUFFER_SIZE];
UART_sStateStruct u_config;
uint8_t g_data_bits = 0, g_stop_bits = 0, g_parity = 0;
uint32_t g_baudrate = 0;
uint8_t g_int_process = 0;
static uint8_t control_line_active = 0;
#define TX_FIFO_SIZE (32)
struct {
    // Number of bytes pending to be transferred. This is 0 if there is no
    // ongoing transfer and the uart_handler processed the last transfer.
    volatile uint32_t tx_size;

    uint8_t rx;
} cb_buf;

static uint8_t flow_control_enabled = 1;


static void config_comm_uart()
{
    #if 1
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ClkGate_APB_UART1)| (1 << SYSCTRL_ClkGate_APB_PinCtrl)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO1)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO0)
                                    | (1 << SYSCTRL_ITEM_APB_TMR1)
                                    | (1 << SYSCTRL_ITEM_APB_TMR2));

    PINCTRL_SelUartIn(UART_PORT_1, PIN_UART_RX, IO_NOT_A_PIN);
    PINCTRL_SetPadMux(PIN_UART_TX, IO_SOURCE_UART1_TXD);
    
    PINCTRL_SetPadMux(PIN_UART_RTS, IO_SOURCE_GPIO);
    GIO_SetDirection(PIN_UART_RTS, GIO_DIR_OUTPUT);
    GIO_WriteValue(PIN_UART_RTS, 1);

    GIO_WriteValue(PIN_UART_RTS, 0);
    
    PINCTRL_SetPadMux(PIN_UART_DTR, IO_SOURCE_GPIO);
    GIO_SetDirection(PIN_UART_DTR, GIO_DIR_OUTPUT);
    GIO_WriteValue(PIN_UART_DTR, 1);
    PINCTRL_Pull(PIN_UART_RX, PINCTRL_PULL_UP);

    GIO_WriteValue(PIN_UART_RTS, 0);
    #endif
    
    // setup channel 0 of timer 2: 100Hz
    TMR_SetOpMode(APB_TMR2, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_APB, 0);
    TMR_SetReload(APB_TMR2, 0, TMR_GetClk(APB_TMR2, 0)/17);
    TMR_IntEnable(APB_TMR2, 0, 0xf);

    // setup channel 0 of timer 1: 1Hz
    TMR_SetOpMode(APB_TMR1, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_APB, 0);
    TMR_SetReload(APB_TMR1, 0, TMR_GetClk(APB_TMR1, 0));
    TMR_IntEnable(APB_TMR1, 0, 0xf);
    
    #if 0
    extern void SetUart1PIN(uint8_t TxPIN, uint8_t RxPIN);
    
    sys_ctrl_reg->cgu_cfg3.f.apb_clk_gpio0_gate = 1;
    sys_ctrl_reg->cgu_cfg3.f.apb_clk_iomux_gate = 1;
    sys_ctrl_reg->cgu_cfg3.f.apb_clk_uart1_gate = 1;
    sys_ctrl_reg->cgu_cfg5.f.fclk_uart1_gate = 1;
    SetUart1PIN(PIN_UART_TX,PIN_UART_RX); 
    #endif
    
}

static void activate_control_line()
{
    if (control_line_active)
    {
        TMR_IntClr(APB_TMR1, 0, 0xf);
        TMR_Enable(APB_TMR1, 0, 0x0);
    }
    else
    {
        control_line_active = 1;
        GIO_WriteValue(PIN_UART_RTS, 0);
        GIO_SetDirection(PIN_UART_RTS, GIO_DIR_OUTPUT);

        GIO_WriteValue(PIN_UART_DTR, 1);
        GIO_SetDirection(PIN_UART_DTR, GIO_DIR_OUTPUT);
    }

    TMR_Enable(APB_TMR1, 0, 0xf);
}

void IRQHandler_Timer1(void)
{
    TMR_IntClr(APB_TMR1, 0, 0xf);
    TMR_Enable(APB_TMR1, 0, 0x0);
    control_line_active = 0;

    GIO_SetDirection(PIN_UART_RTS, GIO_DIR_NONE);
    GIO_SetDirection(PIN_UART_DTR, GIO_DIR_NONE);
}

void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t uart_initialize(void)
{
    NVIC_DisableIRQ(CDC_UART_IRQ);
    config_comm_uart();
    CDC_UART_BASE->Control &= (~(1<<bsUART_RECEIVE_ENABLE));
    CDC_UART_BASE->Control &= (~(1<<bsUART_TRANSMIT_ENABLE));
    cb_buf.tx_size = 0;
    clear_buffers();
    NVIC_ClearPendingIRQ(CDC_UART_IRQ);
//    NVIC_SetPriority(CDC_UART_IRQ,14);
    //wait until cdc driver is loaded
    NVIC_EnableIRQ(CDC_UART_IRQ);
    NVIC_EnableIRQ(IRQn_Timer1);
    NVIC_EnableIRQ(IRQn_Timer2);

    return 1;
}

int32_t uart_uninitialize(void)
{
    // disable interrupt
    apUART_Disable_RECEIVE_INT(CDC_UART_BASE);
    // disable clk
    // SYSCTRL_SetClkGateMulti((1 << SYSCTRL_ClkGate_APB_UART1));
    NVIC_ClearPendingIRQ(CDC_UART_IRQ);
    NVIC_DisableIRQ(CDC_UART_IRQ);
    cb_buf.tx_size = 0;
    clear_buffers();

    return 1;
}

int32_t uart_reset(void)
{
    // disable interrupt
    NVIC_DisableIRQ(CDC_UART_IRQ);
    printf("reset pin\n");
    // reset uart
    cb_buf.tx_size = 0;
    apUART_uart_reset(CDC_UART_BASE);
    // enable interrupt
    NVIC_EnableIRQ(CDC_UART_IRQ);
    return 1;
}

static void renable_uart(void)
{
    CDC_UART_BASE->Control &= (~(1<<bsUART_ENABLE));
    NVIC_DisableIRQ(CDC_UART_IRQ);
    TMR_Enable(APB_TMR2, 0, 0xf);
}

void IRQHandler_Timer2(void)
{
    uint32_t status;
    status = CDC_UART_BASE->IntRaw;

    CDC_UART_BASE->IntClear = status;
    TMR_IntClr(APB_TMR2, 0, 0xf);
    TMR_Enable(APB_TMR2, 0, 0x0);
    NVIC_ClearPendingIRQ(CDC_UART_IRQ);
    CDC_UART_BASE->Control |= (1<<bsUART_ENABLE);
    NVIC_EnableIRQ(CDC_UART_IRQ);
}

static int32_t uart_setup(const UART_Configuration *config, uint8_t enable_int)
{
    printf("init uart\n");
    uint8_t cts_en = 0, rts_en = 0;
    uint32_t state;
    UART_ePARITY parity;

    // disable interrupt
    NVIC_DisableIRQ(CDC_UART_IRQ);
    uint32_t status = CDC_UART_BASE->IntRaw;
    CDC_UART_BASE->IntClear = status;
    
    printf("baud:%d\n",config->Baudrate);
    
    g_data_bits = 8;
    g_stop_bits = 1;
    g_parity = config->Parity;
    g_baudrate = config->Baudrate;
    
    switch (config->Parity) {
        case UART_PARITY_ODD:
            parity = UART_PARITY_ODD_PARITY;
            break;     // Parity Odd

        case UART_PARITY_EVEN:
            parity = UART_PARITY_EVEN_PARITY;
            break;    // Parity Even

        case UART_PARITY_MARK:
            parity = UART_PARITY_FIX_ONE;
            break;    // Parity Mark

        case UART_PARITY_SPACE:
            parity = UART_PARITY_FIX_ZERO;
            break;   // Parity Space

        case UART_PARITY_NONE:                          // Parity None
        default:
            parity = UART_PARITY_NOT_CHECK;
            break;
    }
    
    if (flow_control_enabled) {
        cts_en = 1;
        rts_en = 1;
    }else
    {
        cts_en = 0;
        rts_en = 0;
    }
    
    u_config.word_length       = UART_WLEN_8_BITS;
    u_config.parity            = parity;
    u_config.fifo_enable       = 1;
    u_config.two_stop_bits     = 0;
    u_config.receive_en        = 1;
    u_config.transmit_en       = 1;
    u_config.UART_en           = 1;
    u_config.cts_en            = cts_en;
    u_config.rts_en            = rts_en;
    u_config.rxfifo_waterlevel = 1;
    u_config.txfifo_waterlevel = 1;
    u_config.ClockFrequency    = OSC_CLK_FREQ;
    u_config.BaudRate          = config->Baudrate;

    state = cortex_int_get_and_disable();

    
    clear_buffers();
//    CDC_UART_BASE->Control = 0;
    apUART_uart_reset(CDC_UART_BASE);
    
    apUART_Initialize(CDC_UART_BASE, &u_config,
        enable_int ? (1 << bsUART_RECEIVE_INTENAB) : 0);
    cortex_int_restore(state);
    
    // Enable UART interrupt
    NVIC_EnableIRQ(CDC_UART_IRQ);
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    
    return uart_setup(config, 1);
}

void uart_enable_after_linecoding(void)
{
  NVIC_EnableIRQ(CDC_UART_IRQ);
}

int32_t uart_get_configuration(UART_Configuration *config)
{

    config->Baudrate = g_baudrate;
    config->DataBits = UART_DATA_BITS_8;
    config->Parity = (UART_Parity)g_parity;
    config->StopBits = UART_STOP_BITS_1;

    // get flow control
    if (flow_control_enabled) {
        config->FlowControl = UART_FLOW_CONTROL_RTS_CTS;
    }
    else {
        config->FlowControl = UART_FLOW_CONTROL_NONE;
    }
    
    return 1;
}
uint32_t line_ctr;
#define CDC_EP_POINT          4
void uart_set_control_line_state(uint16_t ctrl_bmp)
{
    uint32_t state;
/*
D15..D2
    RESERVED (Reset to zero)
D1
    Carrier control for half duplex modems. This signal corresponds to V.24 signal 105 and RS-232 signal RTS. 0 - Deactivate carrier 1 - Activate carrier
    The device ignores the value of this bit when operating in full duplex mode.
D0
    Indicates to DCE if DTE is present or not. This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR. 0 - Not Present 1 - Present
*/
    #ifdef UART_DEBUG
    printf("set control line %x \n", ctrl_bmp);
    #endif
    // for(uint32_t i=0; i<5;i++) {
    //     state = AHB_USB->UsbDOxConfig[i].DOCtrlx;
    //     printf("enable OUT state EP%d : %d \n",i, (state>>15)&0x1);
    // }
    // for(uint32_t i=0; i<5;i++) {
    //     state = AHB_USB->UsbDIxConfig[i].DICtrlx;
    //     printf("enable IN state  EP%d: %d \n",i,(state>>15)&0x1);
    // }
    
   // default set to high level
   if(ctrl_bmp & (1 << 1))
   {
       GIO_WriteValue(PIN_UART_RTS, 0);
   } else {
       GIO_WriteValue(PIN_UART_RTS, 1);
   }
   
   // default set to high level
   if(ctrl_bmp & (1 << 0))
   {
       GIO_WriteValue(PIN_UART_DTR, 0);
   } else {
       GIO_WriteValue(PIN_UART_DTR, 1);
   }
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

uint32_t uart_driver_send_data(uint8_t *c, uint16_t len)
{
  uint16_t i;
  for(i = 0; i < len; i++)
  {
    while (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 1);
    UART_SendData(CDC_UART_BASE, (uint8_t)*(c+i));
  }
  return 0;
}

uint32_t send_len = 0;
uint32_t uart_driver_send_data_fifo_scheme(uint8_t *c, volatile uint32_t *len)
{
  uint16_t i;
  
  for(i = 0; i < *len; i++)
  {
    if(apUART_Check_TXFIFO_FULL(CDC_UART_BASE)){break;};
    UART_SendData(CDC_UART_BASE, (uint8_t)*(c+i));
  }
  
  *len -= i;
  send_len = i;
  return 0;
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt;
    cortex_int_state_t state;
    if (size == 0) {
        return 0;
    }
//    printf("uart_write_data %d\n", size);
//    printf("write data\n");
    cnt = circ_buf_write(&write_buffer, data, size);
    
    state = cortex_int_get_and_disable();
    if (circ_buf_count_used(&write_buffer) > 0 && g_int_process == 0) {
        if ((apUART_Get_INT_Mask(CDC_UART_BASE)&(1<<bsUART_TRANSMIT_INTENAB)) == 0) {
            
            size = circ_buf_count_used(&write_buffer);
            if (size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            }
            
            if(size < 4) {
                while (size) {
                    CDC_UART_BASE->DataRead = circ_buf_pop(&write_buffer);
                    size--;
                }
                cortex_int_restore(state);
                return cnt;
            }
            g_int_process = 1;
              while (size - 1) {
                  CDC_UART_BASE->DataRead = circ_buf_pop(&write_buffer);
                  size--;
              }
                
            /* Enable Tx Empty Interrupt. (Trigger first one) */
            apUART_Enable_TRANSMIT_INT(CDC_UART_BASE);
        }
    }
    cortex_int_restore(state);
    return cnt;
}


int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void uart_enable_flow_control(bool enabled)
{
    flow_control_enabled = (uint8_t)enabled;
}

uint32_t IRQHandler_Uart1(void *user_data)
{
    uint32_t status, size = 0;
    uint32_t i;
    static uint32_t err_cnt = 0;
    status = CDC_UART_BASE->IntRaw;

    CDC_UART_BASE->IntClear = status;
    
    if (status & (1 << bsUART_FRAME_INTENAB)) {
//        printf("errot faild frame\n");
        err_cnt++;
        if(err_cnt > 5)
            renable_uart();
//        printf("e\n");
//        cb_buf.rx = APB_UART1->DataRead;
        CDC_UART_BASE->StatusClear = 1;
    }
    
    // tx int
    if (status & (1 << bsUART_TRANSMIT_INTENAB)) {
        /* Fill the Tx FIFO */
        size = circ_buf_count_used(&write_buffer);
        if (size > 4) {
            if (size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            } 
            while (size) {
                CDC_UART_BASE->DataRead = circ_buf_pop(&write_buffer);
                size--;
            }
        } else {
            g_int_process = 0;
//            printf("end tx\n");
            /* No more data, just stop Tx (Stop work) */
            CDC_UART_BASE->IntMask &= ~(1<<bsUART_TRANSMIT_INTENAB);
            for(i=0;i<size;i++) {
                CDC_UART_BASE->DataRead = circ_buf_pop(&write_buffer);
            }
            
        }
    }
    
    // rx int
    if (status & (1 << bsUART_RECEIVE_INTENAB)) {
        while (apUART_Check_RXFIFO_EMPTY(APB_UART1) != 1) {
            cb_buf.rx = APB_UART1->DataRead;
            uint32_t free = circ_buf_count_free(&read_buffer);
            if (free > RX_OVRF_MSG_SIZE) {
                circ_buf_push(&read_buffer, cb_buf.rx);
            } else if ((RX_OVRF_MSG_SIZE == free) && config_get_overflow_detect()) {
                circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
            } else {
//                break;
                // Drop character
            }
        }
    }

    return 0;
}

static void uart_delay_ms(uint32_t t)
{
    uint32_t cnt = (uint32_t)((uint64_t)TMR_GetClk(APB_TMR2, 0) * t / 1000);

    TMR_SetOpMode(APB_TMR2, 0, TMR_CTL_OP_MODE_32BIT_TIMER_x1, TMR_CLK_MODE_APB, 0);
    TMR_SetReload(APB_TMR2, 0, cnt);
    TMR_IntEnable(APB_TMR2, 0, 0xf);
    TMR_IntClr(APB_TMR2, 0, 0xf);
    TMR_Enable(APB_TMR2, 0, 0x1);
    while (TMR_IntHappened(APB_TMR2, 0) == 0) ;
    TMR_Enable(APB_TMR2, 0, 0x0);
    TMR_IntClr(APB_TMR2, 0, 0xf);
}

static int uart_extract_all_rx(UART_TypeDef *dev, uint8_t *buffer, int max_len)
{
    memset(buffer, 0, max_len);
    int r = 0;
    while ((r < max_len) && (apUART_Check_RXFIFO_EMPTY(dev) != 1))
        buffer[r++] = (uint8_t)CDC_UART_BASE->DataRead;
    return r;
}

int uart_detect_target(
    uint8_t *platform_version_found,
    uint16_t *ver_major, uint8_t *ver_minor, uint8_t *ver_patch,
    uint32_t *app_addr)
{
    int r = -1;
    uint32_t ver_location;

    UART_Configuration config =
    {
        .Baudrate = 115200,
        .DataBits = UART_DATA_BITS_8,
        .Parity = UART_PARITY_NONE,
        .StopBits = UART_STOP_BITS_1,
        .FlowControl = UART_FLOW_CONTROL_NONE,
    };

    config_comm_uart();
    uart_setup(&config, 0);

    NVIC_DisableIRQ(CDC_UART_IRQ);

    uint8_t buffer[32];
    int len;

    *platform_version_found = 0;

    uart_set_control_line_state(0x2);
    (void)uart_extract_all_rx(CDC_UART_BASE, buffer, sizeof(buffer));
    uart_delay_ms(2);
    uart_set_control_line_state(0x3);
    uart_delay_ms(50);
    len = uart_extract_all_rx(CDC_UART_BASE, buffer, sizeof(buffer));

    IRQHandler_Timer1();

    if (strcmp((char *)buffer, "UartBurnStart916\n") == 0)
        r = INGCHIPS_FAMILY_916;
    else if (strcmp((char *)buffer, "UartBurnStart\n") == 0)
        r = INGCHIPS_FAMILY_918;
    else
        goto exit;

    printf("target: %d\n", r);

    uart_driver_send_data("#$state", 7);
    uart_delay_ms(2);
    len = uart_extract_all_rx(CDC_UART_BASE, buffer, sizeof(buffer));

    if (strcmp((char *)buffer, "#$ulk\n"))
        goto exit;

    switch (r)
    {
    case INGCHIPS_FAMILY_918:
        uart_driver_send_data("#$readd", 7);
        ver_location = 0x000040b0;
        break;
    case INGCHIPS_FAMILY_916:
        uart_driver_send_data("#$fsh2u", 7);
        ver_location = 0x020020fc;
        break;
    }

    uart_driver_send_data((uint8_t *)&ver_location, sizeof(ver_location));
    uart_delay_ms(2);
    len = uart_extract_all_rx(CDC_UART_BASE, buffer, 8);

    if (len >= 8)
    {
        *platform_version_found = 1;

        *ver_major = *(uint16_t *)(buffer + 0);
        *ver_minor = *(uint8_t  *)(buffer + 2);
        *ver_patch = *(uint8_t  *)(buffer + 3);

        *app_addr = *(uint32_t *)(buffer + 4);
    }

exit:

    return r;
}
