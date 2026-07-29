// ----------------------------------------------------------------------------
// Copyright Message                                                           
// ----------------------------------------------------------------------------
//                                                                             
// INGCHIPS confidential and proprietary.                                      
// COPYRIGHT (c) 2018 by INGCHIPS                                              
//                                                                             
// All rights are reserved. Reproduction in whole or in part is                
// prohibited without the written consent of the copyright owner.              
//                                                                             
// ----------------------------------------------------------------------------
// Design Information                                                          
// ----------------------------------------------------------------------------
//                                                                             
// File         : soc.h
//                                                                             
// Author       : Lujie JI
//                                                                             
// Organisation : INGCHIPS                                                     
//                                                                             
// Date         : 2018-06-15 14:27:20
//                                                                             
// Revision     : v0.1                                                         
//                                                                             
// Project      : ING91800                                                     
//                                                                             
// Description  :                                                              
//                                                                             
// ----------------------------------------------------------------------------
// Revision History                                                            
// ----------------------------------------------------------------------------
//                                                                             
// Date       Author          Revision  Change Description                     
// ========== =============== ========= =======================================
// 2018/06/15 Lujie JI        v0.1      Creat
//                                                                             
// ----------------------------------------------------------------------------
#ifndef SOC__H
#define SOC__H

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/******************************************************************************/
/*                               Common                                       */
/******************************************************************************/
typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef unsigned int        uint32_t;
typedef unsigned long long  uint64_t;

#define INGSIMD_BYTE_1K   ((uint32_t)0x400)
#define INGSIMD_BYTE_2K   ((uint32_t)0x800)
#define INGSIMD_BYTE_4K   ((uint32_t)0x1000)
#define INGSIMD_BYTE_8K   ((uint32_t)0x2000)
#define INGSIMD_BYTE_16K  ((uint32_t)0x4000)
#define INGSIMD_BYTE_32K  ((uint32_t)0x8000)
#define INGSIMD_BYTE_48K  ((uint32_t)0xc000)
#define INGSIMD_BYTE_64K  ((uint32_t)0x10000)
#define INGSIMD_BYTE_96K  ((uint32_t)0x18000)
#define INGSIMD_BYTE_128K ((uint32_t)0x20000)
#define INGSIMD_BYTE_256K ((uint32_t)0x40000)
#define INGSIMD_BYTE_512K ((uint32_t)0x80000)
#define INGSIMD_BYTE_1M   ((uint32_t)0x100000)
#define INGSIMD_BYTE_2M   ((uint32_t)0x200000)

/******************************************************************************/
/*                            SYSTEM CLOCK                                    */
/******************************************************************************/
#define KHz                     1000
#define MHz                     1000000

/******************************************************************************/
/*                              REGS H                                        */
/******************************************************************************/
#include "ahbaes_reg.h"
#include "cache_ctrl_reg.h"
#include "dma_reg.h"
#include "flash_spi_ctrl_reg.h"
#include "io_mux_ctrl_reg.h"
#include "sys_ctrl_reg.h"
#include "gpio_reg.h"
#include "i2c_reg.h"
#include "i2s_reg.h"
#include "ir_reg.h"
#include "pwm_reg.h"
#include "rtc_reg.h"
#include "qspi_reg.h"
#include "apbspi_reg.h"
#include "timer_reg.h"
#include "trng_reg.h"
#include "uart_reg.h"
#include "wdt_reg.h"
#include "key_scanner_reg.h"
#include "lle_top_reg.h"
#include "sadc_reg.h"
#include "efuse_reg.h"
#include "aon2_ctrl_reg.h"
#include "aon1_ctrl_reg.h"
#include "pwm_reg.h"
#include "pdm_reg.h"
#include "qdec_reg.h"
#include "cmsis_compiler.h"

/******************************************************************************/
/*                            Memory Map                                      */
/******************************************************************************/
//AHB
#ifndef INGSIMD_SW_BUS_REMAP
#define INGSIMD_A_ROM_BASE            ((uint32_t)0x00000000UL)
#define INGSIMD_A_FLASH_BASE          ((uint32_t)0x02000000UL)
#else
#define INGSIMD_A_ROM_BASE            ((uint32_t)0x02000000UL)
#define INGSIMD_A_FLASH_BASE          ((uint32_t)0x00000000UL)
#endif

#define INGSIMD_A_QSPI_MEM_BASE       ((uint32_t)0x04000000UL)
#define INGSIMD_A_SYS_MEM_BASE        ((uint32_t)0x20000000UL)
#define INGSIMD_A_SYS_APB_BASE        ((uint32_t)0x40000000UL)
#define INGSIMD_A_AON_APB_BASE        ((uint32_t)0x40100000UL)
#define INGSIMD_A_LLE_SUBSYS_BASE     ((uint32_t)0x40110000UL)
#define INGSIMD_A_AHBAES_BASE         ((uint32_t)0x40130000UL)
#define INGSIMD_A_ICACHE_CTRL_BASE    ((uint32_t)0x40140000UL)
#define INGSIMD_A_DCACHE_CTRL_BASE    ((uint32_t)0x40141000UL)
#define INGSIMD_A_FLASH_SPI_CTRL_BASE ((uint32_t)0x40150000UL)
#define INGSIMD_A_QSPI_BASE           ((uint32_t)0x40160000UL)

//SYS APB
#define INGSIMD_A_SYS_CTRL_BASE    (INGSIMD_A_SYS_APB_BASE + 0x0000 )
#define INGSIMD_A_WDT_BASE         (INGSIMD_A_SYS_APB_BASE + 0x1000 )
#define INGSIMD_A_TIMER0_BASE      (INGSIMD_A_SYS_APB_BASE + 0x2000 )
#define INGSIMD_A_TIMER1_BASE      (INGSIMD_A_SYS_APB_BASE + 0x3000 )
#define INGSIMD_A_TIMER2_BASE      (INGSIMD_A_SYS_APB_BASE + 0x4000 )
#define INGSIMD_A_PWM_BASE         (INGSIMD_A_SYS_APB_BASE + 0x5000 )
#define INGSIMD_A_IO_MUX_CTRL_BASE (INGSIMD_A_SYS_APB_BASE + 0x6000 )
#define INGSIMD_A_TRNG_BASE        (INGSIMD_A_SYS_APB_BASE + 0x7000 )
#define INGSIMD_A_PDM_BASE         (INGSIMD_A_SYS_APB_BASE + 0x8000 )
#define INGSIMD_A_QDEC_BASE        (INGSIMD_A_SYS_APB_BASE + 0x9000 )
#define INGSIMD_A_KEY_SCANNER_BASE (INGSIMD_A_SYS_APB_BASE + 0xA000 )
#define INGSIMD_A_IR_BASE          (INGSIMD_A_SYS_APB_BASE + 0xB000 )
#define INGSIMD_A_DMA_BASE         (INGSIMD_A_SYS_APB_BASE + 0xC000 )
#define INGSIMD_A_APBSPI_BASE      (INGSIMD_A_SYS_APB_BASE + 0xE000 )
#define INGSIMD_A_SADC_BASE        (INGSIMD_A_SYS_APB_BASE + 0xF000 )
#define INGSIMD_A_I2S_BASE         (INGSIMD_A_SYS_APB_BASE + 0x10000)
#define INGSIMD_A_UART0_BASE       (INGSIMD_A_SYS_APB_BASE + 0x11000)
#define INGSIMD_A_UART1_BASE       (INGSIMD_A_SYS_APB_BASE + 0x12000)
#define INGSIMD_A_I2C0_BASE        (INGSIMD_A_SYS_APB_BASE + 0x13000)
#define INGSIMD_A_I2C1_BASE        (INGSIMD_A_SYS_APB_BASE + 0x14000)
#define INGSIMD_A_GPIO0_BASE       (INGSIMD_A_SYS_APB_BASE + 0x15000)
#define INGSIMD_A_GPIO1_BASE       (INGSIMD_A_SYS_APB_BASE + 0x16000)
#define INGSIMD_A_EFUSE_BASE       (INGSIMD_A_SYS_APB_BASE + 0x17000)

//AON APB
#define INGSIMD_A_AON2_CTRL_BASE   (INGSIMD_A_AON_APB_BASE + 0x0000 )
#define INGSIMD_A_RTC_BASE         (INGSIMD_A_AON_APB_BASE + 0x1000 )
#define INGSIMD_A_AON1_CTRL_BASE   (INGSIMD_A_AON_APB_BASE + 0x2000 )

//LLE SUBSYS
#define INGSIMD_A_LLE_REG_BASE   (INGSIMD_A_LLE_SUBSYS_BASE + 0x0000 )
#define INGSIMD_A_LLE_MEM_BASE   (INGSIMD_A_LLE_SUBSYS_BASE + 0x10000)

//Memory
#define INGSIMD_ROM_SIZE        INGSIMD_BYTE_16K

#define INGSIMD_SYS_MEM00_SIZE    INGSIMD_BYTE_16K
#define INGSIMD_SYS_MEM10_SIZE    INGSIMD_BYTE_16K
#define INGSIMD_SYS_MEM20_SIZE    (INGSIMD_BYTE_16K+INGSIMD_BYTE_8K+4*INGSIMD_BYTE_2K+4*INGSIMD_BYTE_2K)

#define INGSIMD_LLE_MEM0_SIZE     INGSIMD_BYTE_8K
#define INGSIMD_LLE_MEM1_SIZE     INGSIMD_BYTE_16K
#define INGSIMD_LLE_MEM2_SIZE     INGSIMD_BYTE_8K
#define INGSIMD_LLE_MEM3_SIZE     INGSIMD_BYTE_16K

#define INGSIMD_A_SYS_MEM00_BASE  (INGSIMD_A_SYS_MEM_BASE)
#define INGSIMD_A_SYS_MEM10_BASE  (INGSIMD_A_SYS_MEM_BASE+INGSIMD_SYS_MEM00_SIZE)
#define INGSIMD_A_SYS_MEM20_BASE  (INGSIMD_A_SYS_MEM_BASE+INGSIMD_SYS_MEM00_SIZE+INGSIMD_SYS_MEM10_SIZE)

#define INGSIMD_A_SYS_MEM0_BASE   INGSIMD_A_SYS_MEM00_BASE
#define INGSIMD_A_SYS_MEM1_BASE   INGSIMD_A_SYS_MEM10_BASE
#define INGSIMD_A_SYS_MEM2_BASE   INGSIMD_A_SYS_MEM20_BASE

#define INGSIMD_A_LLE_MEM0_BASE   (INGSIMD_A_LLE_MEM_BASE)
#define INGSIMD_A_LLE_MEM2_BASE   (INGSIMD_A_LLE_MEM_BASE+INGSIMD_LLE_MEM0_SIZE)
#define INGSIMD_A_LLE_MEM1_BASE   (INGSIMD_A_LLE_MEM_BASE+INGSIMD_LLE_MEM0_SIZE+INGSIMD_LLE_MEM2_SIZE)
#define INGSIMD_A_LLE_MEM3_BASE   (INGSIMD_A_LLE_MEM_BASE+INGSIMD_LLE_MEM0_SIZE+INGSIMD_LLE_MEM2_SIZE+INGSIMD_LLE_MEM1_SIZE)

#define INGSIMD_FLASH_PAGE_SIZE    INGSIMD_BYTE_256
#define INGSIMD_FLASH_BLOCK32_SIZE INGSIMD_BYTE_32K
#define INGSIMD_FLASH_BLOCK64_SIZE INGSIMD_BYTE_64K

#define INGSIMD_FLASH_SIZE      INGSIMD_BYTE_512K

/******************************************************************************/
/*                            Register                                        */
/******************************************************************************/
//SYS APB
#define sys_ctrl_reg       ((volatile sys_ctrl_REG_H      *) INGSIMD_A_SYS_CTRL_BASE       )
#define wdt_reg            ((volatile WDT_REG_H           *) INGSIMD_A_WDT_BASE            )
#define timer0_reg         ((volatile TIMER_REG_H         *) INGSIMD_A_TIMER0_BASE         )
#define timer1_reg         ((volatile TIMER_REG_H         *) INGSIMD_A_TIMER1_BASE         )
#define timer2_reg         ((volatile TIMER_REG_H         *) INGSIMD_A_TIMER2_BASE         )
#define pwm_reg            ((volatile pwm_REG_H           *) INGSIMD_A_PWM_BASE            )
#define io_mux_ctrl_reg    ((volatile io_mux_ctrl_REG_H   *) INGSIMD_A_IO_MUX_CTRL_BASE    )
#define trng_reg           ((volatile TRNG_REG_H          *) INGSIMD_A_TRNG_BASE           )
#define pdm_reg            ((volatile PDM_REG_H           *) INGSIMD_A_PDM_BASE            )
#define qdec_reg           ((volatile qdec_REG_H          *) INGSIMD_A_QDEC_BASE           )
#define key_scanner_reg    ((volatile key_scanner_REG_H   *) INGSIMD_A_KEY_SCANNER_BASE    )
#define ir_reg             ((volatile IR_REG_H            *) INGSIMD_A_IR_BASE             )
#define dma_reg            ((volatile DMA_REG_H           *) INGSIMD_A_DMA_BASE            )
#define qspi_reg           ((volatile QSPI_REG_H          *) INGSIMD_A_QSPI_BASE           )
#define apbspi_reg         ((volatile APBSPI_REG_H        *) INGSIMD_A_APBSPI_BASE         )
#define sadc_reg           ((volatile sadc_REG_H          *) INGSIMD_A_SADC_BASE           )
#define i2s_reg            ((volatile I2S_REG_H           *) INGSIMD_A_I2S_BASE            )
#define uart0_reg          ((volatile UART_REG_H          *) INGSIMD_A_UART0_BASE          )
#define uart1_reg          ((volatile UART_REG_H          *) INGSIMD_A_UART1_BASE          )
#define i2c0_reg           ((volatile I2C_REG_H           *) INGSIMD_A_I2C0_BASE           )
#define i2c1_reg           ((volatile I2C_REG_H           *) INGSIMD_A_I2C1_BASE           )
#define gpio0_reg          ((volatile GPIO_REG_H          *) INGSIMD_A_GPIO0_BASE          )
#define gpio1_reg          ((volatile GPIO_REG_H          *) INGSIMD_A_GPIO1_BASE          )
#define efuse_reg          ((volatile efuse_REG_H         *) INGSIMD_A_EFUSE_BASE          )

//AON APB
#define aon2_ctrl_reg      ((volatile aon2_ctrl_REG_H     *) INGSIMD_A_AON2_CTRL_BASE      )
#define rtc_reg            ((volatile RTC_REG_H           *) INGSIMD_A_RTC_BASE            )
#define aon1_ctrl_reg      ((volatile aon1_ctrl_REG_H     *) INGSIMD_A_AON1_CTRL_BASE      )

//LLE SUBSYS
#define lle_reg            ((volatile LLE_TOP_REG_H       *) INGSIMD_A_LLE_REG_BASE        )

//AHB
#define ahbaes_reg         ((volatile AHBAES_REG_H        *) INGSIMD_A_AHBAES_BASE         )
#define icache_ctrl_reg    ((volatile CACHE_CTRL_REG_H    *) INGSIMD_A_ICACHE_CTRL_BASE    )
#define dcache_ctrl_reg    ((volatile CACHE_CTRL_REG_H    *) INGSIMD_A_DCACHE_CTRL_BASE    )
#define flash_spi_ctrl_reg ((volatile FLASH_SPI_CTRL_REG_H*) INGSIMD_A_FLASH_SPI_CTRL_BASE )

/******************************************************************************/
/*                               IRQ                                          */
/******************************************************************************/
#define IRQHandler_CacheI           n00_IRQHandler
#define IRQHandler_CacheD           n01_IRQHandler
#define IRQHandler_RTCCnt           n02_IRQHandler
#define IRQHandler_GPIO1            n03_IRQHandler
#define IRQHandler_GPIO0            n04_IRQHandler
#define IRQHandler_Timer2           n05_IRQHandler
#define IRQHandler_Timer1           n06_IRQHandler
#define IRQHandler_Timer0           n07_IRQHandler
#define IRQHandler_WDT              n08_IRQHandler
#define IRQHandler_PDM              n09_IRQHandler
#define IRQHandler_AHBAES           n10_IRQHandler
#define IRQHandler_LLEErr           n11_IRQHandler
#define IRQHandler_LLEFun           n12_IRQHandler
#define IRQHandler_SPIFlash         n13_IRQHandler
#define IRQHandler_APBSPI           n14_IRQHandler
#define IRQHandler_QSPI             n15_IRQHandler
#define IRQHandler_SADC             n16_IRQHandler
#define IRQHandler_I2S              n17_IRQHandler
#define IRQHandler_Uart1            n18_IRQHandler
#define IRQHandler_Uart0            n19_IRQHandler
#define IRQHandler_I2C1             n20_IRQHandler
#define IRQHandler_I2C0             n21_IRQHandler
#define IRQHandler_DMA              n22_IRQHandler
#define IRQHandler_KeyScan          n23_IRQHandler
#define IRQHandler_c0_int_pwm       n24_IRQHandler
#define IRQHandler_TRNG             n25_IRQHandler
#define IRQHandler_IR_INT           n26_IRQHandler
#define IRQHandler_IR_WKUP          n27_IRQHandler
#define IRQHandler_QDEC2            n28_IRQHandler
#define IRQHandler_QDEC1            n29_IRQHandler
#define IRQHandler_QDEC0            n30_IRQHandler
#define IRQHandler_usb              n31_IRQHandler
#define IRQHandler_lpc_pos          n32_IRQHandler
#define IRQHandler_lpc_neg          n33_IRQHandler
#define IRQHandler_rcmfd_trim_done  n34_IRQHandler
#define IRQHandler_pmu_pvd          n35_IRQHandler
#define IRQHandler_pmu_pdr          n36_IRQHandler
#define IRQHandler_c1_int_pwm       n37_IRQHandler
#define IRQHandler_c2_int_pwm       n38_IRQHandler
#define IRQHandler_c0_ppi_int       n39_IRQHandler
#define IRQHandler_c1_ppi_int       n40_IRQHandler
#define IRQHandler_c2_ppi_int       n41_IRQHandler
#define IRQHandler_c3_ppi_int       n42_IRQHandler
#define IRQHandler_spi2ahb          n43_IRQHandler
#define IRQHandler_xo_ready         n44_IRQHandler
#define IRQHandler_lle_domain_reg   n45_IRQHandler
#define IRQHandler_actcnt_32k       n46_IRQHandler

#define IRQn_CacheI             n00_IRQn
#define IRQn_CacheD             n01_IRQn
#define IRQn_RTCCnt             n02_IRQn
#define IRQn_GPIO1              n03_IRQn
#define IRQn_GPIO0              n04_IRQn
#define IRQn_Timer2             n05_IRQn
#define IRQn_Timer1             n06_IRQn
#define IRQn_Timer0             n07_IRQn
#define IRQn_WDT                n08_IRQn
#define IRQn_PDM                n09_IRQn
#define IRQn_AHBAES             n10_IRQn
#define IRQn_LLEErr             n11_IRQn
#define IRQn_LLEFun             n12_IRQn
#define IRQn_SPIFlash           n13_IRQn
#define IRQn_APBSPI             n14_IRQn
#define IRQn_QSPI               n15_IRQn
#define IRQn_SADC               n16_IRQn
#define IRQn_I2S                n17_IRQn
#define IRQn_Uart1              n18_IRQn
#define IRQn_Uart0              n19_IRQn
#define IRQn_I2C1               n20_IRQn
#define IRQn_I2C0               n21_IRQn
#define IRQn_DMA                n22_IRQn
#define IRQn_KeyScan            n23_IRQn
#define IRQn_c0_int_pwm         n24_IRQn
#define IRQn_TRNG               n25_IRQn
#define IRQn_IR_INT             n26_IRQn
#define IRQn_IR_WKUP            n27_IRQn
#define IRQn_QDEC2              n28_IRQn
#define IRQn_QDEC1              n29_IRQn
#define IRQn_QDEC0              n30_IRQn
#define IRQn_usb                n31_IRQn
#define IRQn_lpc_pos            n32_IRQn
#define IRQn_lpc_neg            n33_IRQn
#define IRQn_rcmfd_trim_done    n34_IRQn
#define IRQn_pmu_pvd            n35_IRQn
#define IRQn_pmu_pdr            n36_IRQn
#define IRQn_c1_int_pwm         n37_IRQn
#define IRQn_c2_int_pwm         n38_IRQn
#define IRQn_c0_ppi_int         n39_IRQn
#define IRQn_c1_ppi_int         n40_IRQn
#define IRQn_c2_ppi_int         n41_IRQn
#define IRQn_c3_ppi_int         n42_IRQn
#define IRQn_spi2ahb            n43_IRQn
#define IRQn_xo_ready           n44_IRQn
#define IRQn_lle_domain_reg     n45_IRQn
#define IRQn_actcnt_32k         n46_IRQn

/******************************************************************************/
/*                               Misc                                         */
/******************************************************************************/
//write & read
#define w32(a,b) *(volatile uint32_t*)(a) = (uint32_t)(b);
#define w16(a,b) *(volatile uint16_t*)(a) = (uint16_t)(b);
#define w8(a,b)  *(volatile uint8_t* )(a) = (uint8_t )(b);
#define r32(a)   *(volatile uint32_t*)(a)
#define r16(a)   *(volatile uint16_t*)(a)
#define r8(a)    *(volatile uint8_t*)(a)

typedef enum IOMUXFuncSelEnum {
    IOMUXFuncSel_gpio                             = 0,
    IOMUXFuncSel_dbg                              = 1,
    IOMUXFuncSel_swdo                             = 2,
    IOMUXFuncSel_spi0_clk_out                     = 3,
    IOMUXFuncSel_spi0_csn_out                     = 4,
    IOMUXFuncSel_spi0_hold_out                    = 5,
    IOMUXFuncSel_spi0_wp_out                      = 6,
    IOMUXFuncSel_spi0_miso_out                    = 7,
    IOMUXFuncSel_spi0_mosi_out                    = 8,
    IOMUXFuncSel_spi1_clk_out                     = 9,
    IOMUXFuncSel_spi1_csn_out                     = 10,
    IOMUXFuncSel_spi1_miso_out                    = 11,
    IOMUXFuncSel_spi1_mosi_out                    = 12,
    IOMUXFuncSel_spi1_hold_out                    = 13,
    IOMUXFuncSel_spi1_wp_out                      = 14,
    IOMUXFuncSel_ir_wakeup                        = 15,
    IOMUXFuncSel_ir_datout                        = 16,
    IOMUXFuncSel_i2s_bclk_o                       = 17,
    IOMUXFuncSel_i2s_lrclk_o                      = 18,
    IOMUXFuncSel_i2s_dout                         = 19,
    IOMUXFuncSel_uart0_txd                        = 20,
    IOMUXFuncSel_uart0_rts                        = 21,
    IOMUXFuncSel_uart1_txd                        = 22,
    IOMUXFuncSel_uart1_rts                        = 23,
    IOMUXFuncSel_i2c0_scl_o                       = 24,
    IOMUXFuncSel_i2c0_sda_o                       = 25,
    IOMUXFuncSel_i2c1_scl_o                       = 26,
    IOMUXFuncSel_i2c1_sda_o                       = 27,
    IOMUXFuncSel_pwm_0a                           = 28,
    IOMUXFuncSel_pwm_0b                           = 29,
    IOMUXFuncSel_pwm_1a                           = 30,
    IOMUXFuncSel_pwm_1b                           = 31,
    IOMUXFuncSel_pwm_2a                           = 32,
    IOMUXFuncSel_pwm_2b                           = 33,
    IOMUXFuncSel_pwm_3a                           = 34,
    IOMUXFuncSel_pwm_3b                           = 35,
    IOMUXFuncSel_pwm_4a                           = 36,
    IOMUXFuncSel_pwm_4b                           = 37,
    IOMUXFuncSel_pwm_5a                           = 38,
    IOMUXFuncSel_pwm_5b                           = 39,
    IOMUXFuncSel_pwm_6a                           = 40,
    IOMUXFuncSel_pwm_6b                           = 41,
    IOMUXFuncSel_pwm_7a                           = 42,
    IOMUXFuncSel_pwm_7b                           = 43,
    IOMUXFuncSel_pwm_8a                           = 44,
    IOMUXFuncSel_pwm_8b                           = 45,
    IOMUXFuncSel_lle_ant_sw0                      = 46,
    IOMUXFuncSel_lle_ant_sw1                      = 47,
    IOMUXFuncSel_lle_ant_sw2                      = 48,
    IOMUXFuncSel_lle_ant_sw3                      = 49,
    IOMUXFuncSel_lle_ant_sw4                      = 50,
    IOMUXFuncSel_lle_ant_sw5                      = 51,
    IOMUXFuncSel_lle_ant_sw6                      = 52,
    IOMUXFuncSel_lle_ant_sw7                      = 53,
    IOMUXFuncSel_lle_pa_txen                      = 54,
    IOMUXFuncSel_lle_pa_rxen                      = 55,
    IOMUXFuncSel_pdm_dmic_mclk                    = 56,
    IOMUXFuncSel_key_out_row_0                    = 57,
    IOMUXFuncSel_key_out_row_1                    = 58,
    IOMUXFuncSel_key_out_row_2                    = 59,
    IOMUXFuncSel_key_out_row_3                    = 60,
    IOMUXFuncSel_key_out_row_4                    = 61,
    IOMUXFuncSel_key_out_row_5                    = 62,
    IOMUXFuncSel_key_out_row_6                    = 63,
    IOMUXFuncSel_key_out_row_7                    = 64,
    IOMUXFuncSel_key_out_row_8                    = 65,
    IOMUXFuncSel_key_out_row_9                    = 66,
    IOMUXFuncSel_key_out_row_10                   = 67,
    IOMUXFuncSel_key_out_row_11                   = 68,
    IOMUXFuncSel_key_out_row_12                   = 69,
    IOMUXFuncSel_key_out_row_13                   = 70,
    IOMUXFuncSel_key_out_row_14                   = 71,
    IOMUXFuncSel_key_out_row_15                   = 72,
    IOMUXFuncSel_key_out_row_16                   = 73,
    IOMUXFuncSel_key_out_row_17                   = 74,
    IOMUXFuncSel_key_out_row_18                   = 75,
    IOMUXFuncSel_key_out_row_19                   = 76,
    IOMUXFuncSel_spi2ahb_do                       = 77,
    IOMUXFuncSel_qdec_timer_ext_out_a0            = 78,
    IOMUXFuncSel_qdec_timer_ext_out_a1            = 79,
    IOMUXFuncSel_qdec_timer_ext_out_a2            = 80,
    IOMUXFuncSel_qdec_timer_ext_out_b0            = 81,
    IOMUXFuncSel_qdec_timer_ext_out_b1            = 82,
    IOMUXFuncSel_qdec_timer_ext_out_b2            = 83,
    IOMUXFuncSel_sw_tms                           = 84,
    IOMUXFuncSel_sw_tck                           = 85,
    IOMUXFuncSel_spi0_clk_in                      = 86,
    IOMUXFuncSel_spi0_csn_in                      = 87,
    IOMUXFuncSel_spi0_hold_in                     = 88,
    IOMUXFuncSel_spi0_wp_in                       = 89,
    IOMUXFuncSel_spi0_miso_in                     = 90,
    IOMUXFuncSel_spi0_mosi_in                     = 91,
    IOMUXFuncSel_spi1_clk_in                      = 92,
    IOMUXFuncSel_spi1_csn_in                      = 93,
    IOMUXFuncSel_spi1_miso_in                     = 94,
    IOMUXFuncSel_spi1_mosi_in                     = 95,
    IOMUXFuncSel_spi1_hold_in                     = 96,
    IOMUXFuncSel_spi1_wp_in                       = 97,
    IOMUXFuncSel_ir_datin                         = 98,
    IOMUXFuncSel_i2s_bclk_i                       = 99,
    IOMUXFuncSel_i2s_lrclk_i                      = 100,
    IOMUXFuncSel_i2s_din                          = 101,
    IOMUXFuncSel_uart0_rxd                        = 102,
    IOMUXFuncSel_uart0_cts                        = 103,
    IOMUXFuncSel_uart1_rxd                        = 104,
    IOMUXFuncSel_uart1_cts                        = 105,
    IOMUXFuncSel_i2c0_scl_i                       = 106,
    IOMUXFuncSel_i2c0_sda_i                       = 107,
    IOMUXFuncSel_i2c1_scl_i                       = 108,
    IOMUXFuncSel_i2c1_sda_i                       = 109,
    IOMUXFuncSel_pdm_dmic_in                      = 110,
    IOMUXFuncSel_key_in_col_0                     = 111,
    IOMUXFuncSel_key_in_col_1                     = 112,
    IOMUXFuncSel_key_in_col_2                     = 113,
    IOMUXFuncSel_key_in_col_3                     = 114,
    IOMUXFuncSel_key_in_col_4                     = 115,
    IOMUXFuncSel_key_in_col_5                     = 116,
    IOMUXFuncSel_key_in_col_6                     = 117,
    IOMUXFuncSel_key_in_col_7                     = 118,
    IOMUXFuncSel_key_in_col_8                     = 119,
    IOMUXFuncSel_key_in_col_9                     = 120,
    IOMUXFuncSel_key_in_col_10                    = 121,
    IOMUXFuncSel_key_in_col_11                    = 122,
    IOMUXFuncSel_key_in_col_12                    = 123,
    IOMUXFuncSel_key_in_col_13                    = 124,
    IOMUXFuncSel_key_in_col_14                    = 125,
    IOMUXFuncSel_key_in_col_15                    = 126,
    IOMUXFuncSel_key_in_col_16                    = 127,
    IOMUXFuncSel_key_in_col_17                    = 128,
    IOMUXFuncSel_key_in_col_18                    = 129,
    IOMUXFuncSel_key_in_col_19                    = 130,
    IOMUXFuncSel_spi2ahb_sclk                     = 131,
    IOMUXFuncSel_spi2ahb_cs                       = 132,
    IOMUXFuncSel_spi2ahb_di                       = 133,
    IOMUXFuncSel_qdec_phasea                      = 134,
    IOMUXFuncSel_qdec_phaseb                      = 135,
    IOMUXFuncSel_qdec_index                       = 136,
    IOMUXFuncSel_qdec_ext_in_clk                  = 137,
    IOMUXFuncSel_qdec_timer_ext_in_a1             = 138,
    IOMUXFuncSel_qdec_timer_ext_in_a2             = 139,
    IOMUXFuncSel_qdec_timer_ext_in_b2             = 140,
    IOMUXFuncSel_p_cap_in0                        = 141,
    IOMUXFuncSel_p_cap_in1                        = 142,
    IOMUXFuncSel_p_cap_in2                        = 143,
    IOMUXFuncSel_p_cap_in3                        = 144,
    IOMUXFuncSel_p_cap_in4                        = 145,
    IOMUXFuncSel_p_cap_in5                        = 146 
} E_IOMUXFuncSel;

#endif
