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
#include "peripheral_ssp.h"
#include "DAP_config.h"
#include "daplink.h"
static void setup_peripherals_spi_module(void);
static void setup_peripherals_spi_pin(void);
void test_spi_swd(void);
uint8_t JTAG2SWD(void);
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
                                    | (1 << SYSCTRL_ITEM_APB_GPIO0)
                                    | (1 << SYSCTRL_ITEM_APB_SPI1));
                                

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
    
    #if (!SPI_ENABLE)
    PINCTRL_SetPadMux(SWCLK_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(SWCLK_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(SWCLK_PIN, 1);
    
    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(SWDIO_PIN, GIO_DIR_BOTH);
    PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);
    GIO_WriteValue(SWDIO_PIN, 1);
    #else
    setup_peripherals_spi_pin();
    setup_peripherals_spi_module();
    #endif
    
//    JTAG2SWD();
//    test_spi_swd();

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

void init_memory(void)
{
    SYSCTRL_CacheControl(SYSCTRL_MEM_BLOCK_AS_CACHE, SYSCTRL_MEM_BLOCK_AS_SYS_MEM);
}

#define MASTER
// #define SLAVE 
#define SPI_MIC_CLK         GIO_GPIO_7

#define SPI_MIC_MOSI        GIO_GPIO_8
#define SPI_MIC_MISO        GIO_GPIO_11
#define SPI_MIC_CS          GIO_GPIO_12

#define SPI_MIC_WP          GIO_GPIO_13
#define SPI_MIC_HOLD        GIO_GPIO_14

#define SPI_MODE            AHB_SSP0
#define SPI_IRQ             QSPI_IRQ
#define SPI_IRQ_HANDER      QSPI_IRQHandler
#define SPI_CLK             SYSCTRL_ITEM_AHB_SPI0
#define SPI_PORT            SPI_PORT_0
//#define SPI_MODE            APB_SSP1
//#define SPI_IRQ             APBSPI_IRQ
//#define SPI_IRQ_HANDER      APBSPI_IRQHandler
//#define SPI_CLK             SYSCTRL_ITEM_APB_SPI1
//#define SPI_PORT            SPI_PORT_1

void setuo_spi_clk(uint32_t clk)
{
    apSSP_SetTimingSclkDiv(SPI_MODE, ((10000000/clk - 2)&0xff));
}


static void setup_peripherals_spi_module(void)
{
    apSSP_sDeviceControlBlock pParam;

    //速率选择
//    pParam.eSclkDiv = setup_peripherals_spi_0_high_speed_interface_clk(21000000);
    pParam.eSclkDiv = 0xff;
    pParam.eSCLKPolarity = SPI_CPOL_SCLK_HIGH_IN_IDLE_STATES;
    pParam.eSCLKPhase = SPI_CPHA_EVEN_SCLK_EDGES;
    pParam.eLsbMsbOrder = SPI_LSB_LEAST_SIGNIFICANT_BIT_FIRST;
    pParam.eDataSize = 4;//SPI_DATALEN_8_BITS;
    #ifdef MASTER
    pParam.eMasterSlaveMode = SPI_SLVMODE_MASTER_MODE;//主从模式
    #else
    pParam.eMasterSlaveMode = SPI_SLVMODE_SLAVE_MODE;
    #endif
    pParam.eReadWriteMode = SPI_TRANSMODE_WRITE_ONLY;
    pParam.eQuadMode = SPI_DUALQUAD_REGULAR_MODE;//设置spi传输使用的io模式两线三线四线
    //单次发送数据大小 fifo满
    pParam.eWriteTransCnt = 0;
    pParam.eReadTransCnt = 0;
    pParam.eAddrEn = SPI_ADDREN_DISABLE;//地址使能
    pParam.eCmdEn = SPI_CMDEN_DISABLE;//命令使能
    //中断触发fifo阈值 半满中断
    pParam.RxThres = 4;
    pParam.TxThres = 4;
    //仅数据模式，该模式仅从机模式有效，必须处于全双工模式
    pParam.SlaveDataOnly = SPI_SLVDATAONLY_DISABLE;
    pParam.eAddrLen = SPI_ADDRLEN_1_BYTE;
    pParam.eInterruptMask = 0;//(1 << bsSPI_INTREN_ENDINTEN);
    pParam.eMOSI_Dir =SPI_MOSI_BI_DIR_MODE; //SPI_MOSI_BI_DIR_MODE;//SPI_MOSI_UNI_DIR_MODE;

    apSSP_DeviceParametersSet(SPI_MODE, &pParam);
}


static void setup_peripherals_spi_pin(void)
{
    SYSCTRL_SelectSpiClk(SPI_PORT_0,0);
    SYSCTRL_ClearClkGateMulti((1 << SPI_CLK)
                                  | (1 << SYSCTRL_ITEM_APB_PinCtrl));
    //该函数可直接选择iomox输入和输出双向配置
    PINCTRL_SelSpiIn(SPI_PORT, SPI_MIC_CLK, IO_NOT_A_PIN, IO_NOT_A_PIN,
                     IO_NOT_A_PIN, IO_NOT_A_PIN, SPI_MIC_MOSI);//SPI_MIC_MISO
    GIO_SetDirection(SPI_MIC_MOSI, GIO_DIR_BOTH);
    PINCTRL_SetDriveStrength(SPI_MIC_MOSI,PINCTRL_DRIVE_12mA);
    PINCTRL_SetDriveStrength(SPI_MIC_CLK,PINCTRL_DRIVE_12mA);
    PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);

}

#define io_out_mux  APB_PINCTRL->OUT_CTRL[2] &= 0xFFFFFF80;   \
                    APB_PINCTRL->IN_CTRL[1] |= 0x7c00


#define io_spi_mux  APB_PINCTRL->OUT_CTRL[2] |= 0x8;   \
                    APB_PINCTRL->IN_CTRL[1] &= ~(0x7c00);       \
                    APB_PINCTRL->IN_CTRL[1] |= 0x8<<10


#define spi_dir_write(data_bit,data_vale) \
        SPI_MODE->TransCtrl &= (~((BW2M(4) << bsSPI_TRANSCTRL_TRANSMODE)|(BW2M(bwSPI_TRANSCTRL_WRTRANCNT) << bsSPI_TRANSCTRL_WRTRANCNT))); \
        SPI_MODE->TransCtrl |= ((SPI_TRANSMODE_WRITE_ONLY << bsSPI_TRANSCTRL_TRANSMODE)); \
        SPI_MODE->TransFmt &= (~(0x1f<<bsSPI_TRANSFMT_DATALEN));  \
        SPI_MODE->TransFmt |= ((data_bit-1)<<bsSPI_TRANSFMT_DATALEN);  \
        SPI_MODE->Cmd = 0; \
        SPI_MODE->Data = data_vale;  \
        while((SPI_MODE->Status & 1)) \
        
#define spi_fast_write_bit(data_bit,data_vale) \
        SPI_MODE->TransCtrl &= (~((BW2M(bwSPI_TRANSCTRL_WRTRANCNT) << bsSPI_TRANSCTRL_WRTRANCNT))); \
        SPI_MODE->TransFmt &= (~(0x1f<<bsSPI_TRANSFMT_DATALEN));  \
        SPI_MODE->TransFmt |= ((data_bit-1)<<bsSPI_TRANSFMT_DATALEN);  \
        SPI_MODE->Cmd = 0; \
        SPI_MODE->Data = data_vale;  \
        while((SPI_MODE->Status & 1)) \
            
#define spi_dir_read(data_bit,data_vale) \
        SPI_MODE->TransCtrl &= (~((BW2M(4) << bsSPI_TRANSCTRL_TRANSMODE)|(BW2M(bwSPI_TRANSCTRL_RDTRANCNT) << bsSPI_TRANSCTRL_RDTRANCNT))); \
        SPI_MODE->TransCtrl |= ((SPI_TRANSMODE_READ_ONLY << bsSPI_TRANSCTRL_TRANSMODE)); \
        SPI_MODE->TransFmt &= (~(0x1f<<bsSPI_TRANSFMT_DATALEN));  \
        SPI_MODE->TransFmt |= ((data_bit-1)<<bsSPI_TRANSFMT_DATALEN);  \
        SPI_MODE->Cmd = 0; \
        data_vale = SPI_MODE->Data;  \
        while((SPI_MODE->Status & 1))

#define spi_fast_read_bit(data_bit,data_vale) \
        SPI_MODE->TransFmt &= (~(0x1f<<bsSPI_TRANSFMT_DATALEN));  \
        SPI_MODE->TransFmt |= ((data_bit-1)<<bsSPI_TRANSFMT_DATALEN);  \
        SPI_MODE->Cmd = 0; \
        data_vale = SPI_MODE->Data;  \
        while((SPI_MODE->Status & 1))
#if SPI_ENABLE
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data)
{
    uint32_t parity;
    uint32_t ack; 
    uint32_t rw_data;
    uint32_t read_data;
    uint32_t read_parity;
    uint32_t i;
    uint32_t bytes; 
    uint32_t rem_bits;
    
    parity= 0;
    rw_data = 0;
    ack = 0;
    // 请求阶段，8bit写
    parity = __builtin_parity(request&0xf);
    rw_data = 0x81|(parity<<5)|((request&0xf)<<1);
    spi_dir_write(8,rw_data);
    // 响应阶段，3+trd
    spi_dir_read(3+DAP_Data.swd_conf.turnaround,ack);
    ack = (ack>>DAP_Data.swd_conf.turnaround)&0x7;
    
    

    // // 处理不同响应类型
    if (ack ==  DAP_TRANSFER_OK) {
        if (request & DAP_TRANSFER_RnW) {
            // 读数据阶段，16+8+10bit读
            rw_data = 0;
            parity = 0;
            spi_dir_read(16,rw_data);
            read_data = rw_data;
            spi_fast_read_bit(17+DAP_Data.swd_conf.turnaround,rw_data);
            read_data |= ((rw_data & (0xffff)) << 16);
            parity = __builtin_parity(read_data);
            read_parity = (rw_data>>16) & 0x1;
            if ((parity ^ read_parity) & 1U) {
                ack = DAP_TRANSFER_ERROR;
            }
//            printf("read\n");
            if (data) { *data = read_data; }
        } else {
            rw_data = 0;
            parity = 0;
            // 写数据阶段，16+8+9bit写
            rw_data = *data;
            spi_dir_write(16 + DAP_Data.swd_conf.turnaround,rw_data<<DAP_Data.swd_conf.turnaround);
            parity = __builtin_parity(rw_data);
            spi_fast_write_bit(17,rw_data>>16|(parity<<16));
//            printf("write\n");
        }
        /* Capture Timestamp */                                                     
        if (request & DAP_TRANSFER_TIMESTAMP) {                                     
            DAP_Data.timestamp = TIMESTAMP_GET();                                     
        }
        if(DAP_Data.transfer.idle_cycles) {
//            printf("idle\n");
//        rw_data = 0;
//        bytes = DAP_Data.transfer.idle_cycles/32;
//        rem_bits = DAP_Data.transfer.idle_cycles%32;
//        if(bytes) {
//            spi_dir_write(32,rw_data);
//            bytes--;
//            for(i=0;i<bytes;i++) {
//                spi_fast_write_bit(32,rw_data);
//            }
//        }
        
//        for(i=0;i<rem_bits;i++) {
//            rw_data |= (1<<i);
//        }
//        spi_dir_write(DAP_Data.transfer.idle_cycles,rw_data);
        }
        return ((uint8_t)ack);
    }

    if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {
//        printf("wait\n");
        
        if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) {
            spi_dir_read(16,rw_data);
            spi_fast_read_bit(17,rw_data);
        }
        spi_dir_read(DAP_Data.swd_conf.turnaround,rw_data);
        if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) {
            spi_dir_write(16,0);
            spi_fast_write_bit(17,0);
        }
        return ((uint8_t)ack);
    }
    /* Protocol error */  
    spi_dir_read(16,rw_data);
    spi_fast_read_bit(17+DAP_Data.swd_conf.turnaround,rw_data);

    return ((uint8_t)ack); 
}

void SWJ_Sequence(uint32_t count, const uint8_t *data) {
    uint32_t bytes = count / 8;    
    uint32_t rem_bits = count % 8;
    if (bytes) {
        spi_dir_write(8, *data++);
        bytes--;
        while (bytes--) {
            spi_fast_write_bit(8, *data++);
        }
    }
    if (rem_bits) {
        spi_dir_write(rem_bits, *data);
    }
}

void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
    uint32_t val;
    uint32_t bit_len;

    bit_len = info & SWD_SEQUENCE_CLK;
    if (bit_len == 0) {
        bit_len = 64U;
    }

    if (info & SWD_SEQUENCE_DIN) {
        if (bit_len/8) {
            spi_dir_read(8,val);
            *swdi++ = (uint8_t)val;
            bit_len -= 8U;
            while (bit_len/8U) {
                spi_fast_write_bit(8,val);
                *swdi++ = (uint8_t)val;
                bit_len -= 8U;
            }
        }
    } else {
        if (bit_len/8) {
            spi_dir_write(8,*swdo++);
            bit_len -= 8U;
            while (bit_len/8U) {
                spi_fast_write_bit(8,*swdo++);
                bit_len -= 8U;
            }
        }
    }
}
#endif

void test_spi_swd(void)
{
    uint32_t swd_data = 1;
    SWD_Transfer(0,&swd_data);
}

