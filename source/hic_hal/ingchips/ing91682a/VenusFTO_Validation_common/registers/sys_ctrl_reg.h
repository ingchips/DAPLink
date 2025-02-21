// ----------------------------------------------------------------------------
// Copyright Message                                                           
// ----------------------------------------------------------------------------
//                                                                             
// INGCHIPS confidential and proprietary.                                      
// COPYRIGHT (c) 2022 by INGCHIPS
//                                                                             
// All rights are reserved. Reproduction in whole or in part is                
// prohibited without the written consent of the copyright owner.              
//                                                                             
// ----------------------------------------------------------------------------
// Design Information                                                          
// ----------------------------------------------------------------------------
//                                                                             
// File         : sys_ctrl_reg.h
//                                                                             
// Author       : INGCHIPS                                                     
//                                                                             
// Organisation : INGCHIPS                                                     
//                                                                             
// Date         : 2022/4/26 
//                                                                             
// Revision     : v0.1                                                         
//                                                                             
// Project      :                                                              
//                                                                             
// Description  :                                                              
//                                                                             
// ----------------------------------------------------------------------------
// Revision History                                                            
// ----------------------------------------------------------------------------
//                                                                             
// Date       Author          Revision  Change Description                     
// ========== =============== ========= =======================================
// 2022/4/26  INGCHIPS        v0.1      Create
//                                                                             
// ----------------------------------------------------------------------------
#ifndef SYS_CTRL_REG__H
#define SYS_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t div_hclk_denom                   : 4 ;
        volatile uint32_t div_pclk_denom                   : 4 ;
        volatile uint32_t div_aon_denom                    : 4 ;
        volatile uint32_t div_i2s_denom                    : 4 ;
        volatile uint32_t div_flash_denom                  : 4 ;
        volatile uint32_t div_qspi_denom                   : 4 ;
        volatile uint32_t div_ks_denom                     : 4 ;
        volatile uint32_t div_hclk_update                  : 1 ;
        volatile uint32_t div_pclk_update                  : 1 ;
        volatile uint32_t div_aon_update                   : 1 ;
        volatile uint32_t div_i2s_update                   : 1 ;
    }f;
}sys_ctrl_REG_cgu_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused0                          : 10;
        volatile uint32_t div_32k_update                   : 1 ;
        volatile uint32_t en_clk_32k_div                   : 1 ;
        volatile uint32_t sel_clk_32k                      : 1 ;
        volatile uint32_t sel_clk_ks                       : 1 ;
        volatile uint32_t sel_hclk                         : 1 ;
        volatile uint32_t sel_clk_timer0                   : 1 ;
        volatile uint32_t sel_clk_timer1                   : 1 ;
        volatile uint32_t sel_clk_timer2                   : 1 ;
        volatile uint32_t sel_clk_flash                    : 1 ;
        volatile uint32_t sel_clk_uart0                    : 1 ;
        volatile uint32_t sel_clk_uart1                    : 1 ;
        volatile uint32_t sel_clk_qspi                     : 1 ;
        volatile uint32_t sel_clk_spi                      : 1 ;
        volatile uint32_t sel_clk_i2s                      : 1 ;
        volatile uint32_t sel_clk_lle_sram1                : 1 ;
        volatile uint32_t auto_cg_ram0                     : 1 ;
        volatile uint32_t auto_cg_ram1                     : 1 ;
        volatile uint32_t auto_cg_ram2                     : 1 ;
        volatile uint32_t auto_cg_rom                      : 1 ;
        volatile uint32_t div_flash_update                 : 1 ;
        volatile uint32_t div_qspi_update                  : 1 ;
        volatile uint32_t div_ks_update                    : 1 ;
    }f;
}sys_ctrl_REG_cgu_cfg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ahb_clk_dma_gate                 : 1 ;
        volatile uint32_t ahb_clk_lle_gate                 : 1 ;
        volatile uint32_t ahb_clk_cpu_gate                 : 1 ;
        volatile uint32_t ahb_clk_cache_gate               : 1 ;
        volatile uint32_t ahb_clk_rom_gate                 : 1 ;
        volatile uint32_t ahb_clk_sm0_gate                 : 1 ;
        volatile uint32_t ahb_clk_sm1_gate                 : 1 ;
        volatile uint32_t ahb_clk_sm2_gate                 : 1 ;
        volatile uint32_t ahb_clk_aon_gate                 : 1 ;
        volatile uint32_t ahb_clk_aes_gate                 : 1 ;
        volatile uint32_t ahb_clk_flash_data_gate          : 1 ;
        volatile uint32_t ahb_clk_flash_cfg_gate           : 1 ;
        volatile uint32_t ahb_clk_qspi_gate                : 1 ;
        volatile uint32_t ahb_clk_spi2ahb_gate             : 1 ;
        volatile uint32_t clk_aes_gate                     : 1 ;
        volatile uint32_t ahb_clk_dma_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_lle_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_cpu_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_cache_gate_dr            : 1 ;
        volatile uint32_t ahb_clk_rom_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_sm0_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_sm1_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_sm2_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_aon_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_aes_gate_dr              : 1 ;
        volatile uint32_t ahb_clk_flash_data_gate_dr       : 1 ;
        volatile uint32_t ahb_clk_flash_cfg_gate_dr        : 1 ;
        volatile uint32_t ahb_clk_qspi_gate_dr             : 1 ;
        volatile uint32_t ahb_clk_spi2ahb_gate_dr          : 1 ;
        volatile uint32_t clk_aes_gate_dr                  : 1 ;
        volatile uint32_t unused1                          : 2 ;
    }f;
}sys_ctrl_REG_cgu_cfg2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t apb_clk_sysctrl_gate             : 1 ;
        volatile uint32_t apb_clk_wdt_gate                 : 1 ;
        volatile uint32_t apb_clk_timer0_gate              : 1 ;
        volatile uint32_t apb_clk_timer1_gate              : 1 ;
        volatile uint32_t apb_clk_timer2_gate              : 1 ;
        volatile uint32_t apb_clk_pwm_gate                 : 1 ;
        volatile uint32_t apb_clk_iomux_gate               : 1 ;
        volatile uint32_t apb_clk_trng_gate                : 1 ;
        volatile uint32_t apb_clk_pdm_gate                 : 1 ;
        volatile uint32_t apb_clk_qdec_gate                : 1 ;
        volatile uint32_t apb_clk_key_scanner_gate         : 1 ;
        volatile uint32_t apb_clk_ir_gate                  : 1 ;
        volatile uint32_t apb_clk_dma_gate                 : 1 ;
        volatile uint32_t apb_clk_qspi_gate                : 1 ;
        volatile uint32_t apb_clk_spi1_gate                : 1 ;
        volatile uint32_t apb_clk_sar_adc_gate             : 1 ;
        volatile uint32_t apb_clk_i2s_gate                 : 1 ;
        volatile uint32_t apb_clk_uart0_gate               : 1 ;
        volatile uint32_t apb_clk_uart1_gate               : 1 ;
        volatile uint32_t apb_clk_i2c0_gate                : 1 ;
        volatile uint32_t apb_clk_i2c1_gate                : 1 ;
        volatile uint32_t apb_clk_gpio0_gate               : 1 ;
        volatile uint32_t apb_clk_gpio1_gate               : 1 ;
        volatile uint32_t apb_clk_efuse_gate               : 1 ;
        volatile uint32_t apb_clk_aon_ctrl_gate            : 1 ;
        volatile uint32_t apb_clk_lpc_gate                 : 1 ;
        volatile uint32_t unused2                          : 6 ;
    }f;
}sys_ctrl_REG_cgu_cfg3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t apb_clk_sysctrl_gate_dr          : 1 ;
        volatile uint32_t apb_clk_wdt_gate_dr              : 1 ;
        volatile uint32_t apb_clk_timer0_gate_dr           : 1 ;
        volatile uint32_t apb_clk_timer1_gate_dr           : 1 ;
        volatile uint32_t apb_clk_timer2_gate_dr           : 1 ;
        volatile uint32_t apb_clk_pwm_gate_dr              : 1 ;
        volatile uint32_t apb_clk_iomux_gate_dr            : 1 ;
        volatile uint32_t apb_clk_trng_gate_dr             : 1 ;
        volatile uint32_t apb_clk_pdm_gate_dr              : 1 ;
        volatile uint32_t apb_clk_qdec_gate_dr             : 1 ;
        volatile uint32_t apb_clk_key_scanner_gate_dr      : 1 ;
        volatile uint32_t apb_clk_ir_gate_dr               : 1 ;
        volatile uint32_t apb_clk_dma_gate_dr              : 1 ;
        volatile uint32_t apb_clk_qspi_gate_dr             : 1 ;
        volatile uint32_t apb_clk_spi1_gate_dr             : 1 ;
        volatile uint32_t apb_clk_sar_adc_gate_dr          : 1 ;
        volatile uint32_t apb_clk_i2s_gate_dr              : 1 ;
        volatile uint32_t apb_clk_uart0_gate_dr            : 1 ;
        volatile uint32_t apb_clk_uart1_gate_dr            : 1 ;
        volatile uint32_t apb_clk_i2c0_gate_dr             : 1 ;
        volatile uint32_t apb_clk_i2c1_gate_dr             : 1 ;
        volatile uint32_t apb_clk_gpio0_gate_dr            : 1 ;
        volatile uint32_t apb_clk_gpio1_gate_dr            : 1 ;
        volatile uint32_t apb_clk_efuse_gate_dr            : 1 ;
        volatile uint32_t apb_clk_aon_ctrl_gate_dr         : 1 ;
        volatile uint32_t apb_clk_lpc_gate_dr              : 1 ;
        volatile uint32_t unused3                          : 6 ;
    }f;
}sys_ctrl_REG_cgu_cfg4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t fclk_timer0_gate                 : 1 ;
        volatile uint32_t fclk_timer1_gate                 : 1 ;
        volatile uint32_t fclk_timer2_gate                 : 1 ;
        volatile uint32_t fclk_flash_gate                  : 1 ;
        volatile uint32_t fclk_uart0_gate                  : 1 ;
        volatile uint32_t fclk_uart1_gate                  : 1 ;
        volatile uint32_t fclk_qspi_gate                   : 1 ;
        volatile uint32_t fclk_spi_gate                    : 1 ;
        volatile uint32_t fclk_i2s_gate                    : 1 ;
        volatile uint32_t fclk_pdm_32m_gate                : 1 ;
        volatile uint32_t fclk_pwm_32m_gate                : 1 ;
        volatile uint32_t fclk_ir_32m_gate                 : 1 ;
        volatile uint32_t fclk_sar_adc_32m_gate            : 1 ;
        volatile uint32_t fclk_lle_main_32m_gate           : 1 ;
        volatile uint32_t fclk_efuse_32m_gate              : 1 ;
        volatile uint32_t fclk_cpu_32k_gate                : 1 ;
        volatile uint32_t fclk_ir_32k_gate                 : 1 ;
        volatile uint32_t fclk_wdt_32k_gate                : 1 ;
        volatile uint32_t fclk_gpio_32k_gate               : 1 ;
        volatile uint32_t fclk_ks_gate                     : 1 ;
        volatile uint32_t fclk_ks_32k_gate                 : 1 ;
        volatile uint32_t unused4                          : 11;
    }f;
}sys_ctrl_REG_cgu_cfg5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t fclk_timer0_gate_dr              : 1 ;
        volatile uint32_t fclk_timer1_gate_dr              : 1 ;
        volatile uint32_t fclk_timer2_gate_dr              : 1 ;
        volatile uint32_t fclk_flash_gate_dr               : 1 ;
        volatile uint32_t fclk_uart0_gate_dr               : 1 ;
        volatile uint32_t fclk_uart1_gate_dr               : 1 ;
        volatile uint32_t fclk_qspi_gate_dr                : 1 ;
        volatile uint32_t fclk_spi_gate_dr                 : 1 ;
        volatile uint32_t fclk_i2s_gate_dr                 : 1 ;
        volatile uint32_t fclk_pdm_32m_gate_dr             : 1 ;
        volatile uint32_t fclk_pwm_32m_gate_dr             : 1 ;
        volatile uint32_t fclk_ir_32m_gate_dr              : 1 ;
        volatile uint32_t fclk_sar_adc_32m_gate_dr         : 1 ;
        volatile uint32_t fclk_lle_main_32m_gate_dr        : 1 ;
        volatile uint32_t fclk_efuse_32m_gate_dr           : 1 ;
        volatile uint32_t fclk_cpu_32k_gate_dr             : 1 ;
        volatile uint32_t fclk_ir_32k_gate_dr              : 1 ;
        volatile uint32_t fclk_wdt_32k_gate_dr             : 1 ;
        volatile uint32_t fclk_gpio_32k_gate_dr            : 1 ;
        volatile uint32_t fclk_ks_gate_dr                  : 1 ;
        volatile uint32_t fclk_ks_32k_gate_dr              : 1 ;
        volatile uint32_t unused5                          : 11;
    }f;
}sys_ctrl_REG_cgu_cfg6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lle_hclk_gate                    : 1 ;
        volatile uint32_t lle_cc1_clk_gate                 : 1 ;
        volatile uint32_t lle_cc2_clk_gate                 : 1 ;
        volatile uint32_t lle_aes_clk_gate                 : 1 ;
        volatile uint32_t lle_rfctrl_clk_gate              : 1 ;
        volatile uint32_t lle_decoder_hclk_gate            : 1 ;
        volatile uint32_t lle_encoder_hclk_gate            : 1 ;
        volatile uint32_t lle_crc_clk_gate                 : 1 ;
        volatile uint32_t lle_whitening_clk_gate           : 1 ;
        volatile uint32_t lle_cte_clk_gate                 : 1 ;
        volatile uint32_t lle_hclk_gate_dr                 : 1 ;
        volatile uint32_t lle_cc1_clk_gate_dr              : 1 ;
        volatile uint32_t lle_cc2_clk_gate_dr              : 1 ;
        volatile uint32_t lle_aes_clk_gate_dr              : 1 ;
        volatile uint32_t lle_rfctrl_clk_gate_dr           : 1 ;
        volatile uint32_t lle_decoder_hclk_gate_dr         : 1 ;
        volatile uint32_t lle_encoder_hclk_gate_dr         : 1 ;
        volatile uint32_t lle_crc_clk_gate_dr              : 1 ;
        volatile uint32_t lle_whitening_clk_gate_dr        : 1 ;
        volatile uint32_t lle_cte_clk_gate_dr              : 1 ;
        volatile uint32_t div_32k_denom                    : 12;
    }f;
}sys_ctrl_REG_cgu_cfg7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t hrstn_rom_sw                     : 1 ;
        volatile uint32_t hrstn_sm0_sw                     : 1 ;
        volatile uint32_t hrstn_sm1_sw                     : 1 ;
        volatile uint32_t hrstn_sm2_sw                     : 1 ;
        volatile uint32_t hrstn_dma_sw                     : 1 ;
        volatile uint32_t hrstn_aes_sw                     : 1 ;
        volatile uint32_t hrstn_flash_sw                   : 1 ;
        volatile uint32_t hrstn_qspi_sw                    : 1 ;
        volatile uint32_t hrstn_spi2ahb_sw                 : 1 ;
        volatile uint32_t hrstn_lle_sw                     : 1 ;
        volatile uint32_t prstn_sys_ctrl_sw                : 1 ;
        volatile uint32_t prstn_wdt_sw                     : 1 ;
        volatile uint32_t prstn_timer0_sw                  : 1 ;
        volatile uint32_t prstn_timer1_sw                  : 1 ;
        volatile uint32_t prstn_timer2_sw                  : 1 ;
        volatile uint32_t prstn_pwm_sw                     : 1 ;
        volatile uint32_t prstn_iomux_sw                   : 1 ;
        volatile uint32_t prstn_trng_sw                    : 1 ;
        volatile uint32_t prstn_pdm_sw                     : 1 ;
        volatile uint32_t prstn_qdec_sw                    : 1 ;
        volatile uint32_t prstn_key_scanner_sw             : 1 ;
        volatile uint32_t prstn_ir_sw                      : 1 ;
        volatile uint32_t prstn_dma_sw                     : 1 ;
        volatile uint32_t prstn_qspi_sw                    : 1 ;
        volatile uint32_t prstn_spi_sw                     : 1 ;
        volatile uint32_t prstn_sar_adc_sw                 : 1 ;
        volatile uint32_t prstn_i2s_sw                     : 1 ;
        volatile uint32_t prstn_uart0_sw                   : 1 ;
        volatile uint32_t prstn_uart1_sw                   : 1 ;
        volatile uint32_t prstn_i2c0_sw                    : 1 ;
        volatile uint32_t prstn_i2c1_sw                    : 1 ;
        volatile uint32_t prstn_gpio0_sw                   : 1 ;
    }f;
}sys_ctrl_REG_rstu_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t prstn_gpio1_sw                   : 1 ;
        volatile uint32_t prstn_efuse_sw                   : 1 ;
        volatile uint32_t rstn_timer0_sw                   : 1 ;
        volatile uint32_t rstn_timer1_sw                   : 1 ;
        volatile uint32_t rstn_timer2_sw                   : 1 ;
        volatile uint32_t rstn_flash_sw                    : 1 ;
        volatile uint32_t rstn_uart0_sw                    : 1 ;
        volatile uint32_t rstn_uart1_sw                    : 1 ;
        volatile uint32_t rstn_qspi_sw                     : 1 ;
        volatile uint32_t rstn_spi_sw                      : 1 ;
        volatile uint32_t rstn_i2s_sw                      : 1 ;
        volatile uint32_t rstn_pdm_sw                      : 1 ;
        volatile uint32_t rstn_pwm_sw                      : 1 ;
        volatile uint32_t rstn_ir_sw                       : 1 ;
        volatile uint32_t rstn_sar_adc_sw                  : 1 ;
        volatile uint32_t rstn_efuse_sw                    : 1 ;
        volatile uint32_t rstn_32k_cpu_sw                  : 1 ;
        volatile uint32_t rstn_32k_ir_sw                   : 1 ;
        volatile uint32_t rstn_32k_wdt_sw                  : 1 ;
        volatile uint32_t rstn_lle_sram0_sw                : 1 ;
        volatile uint32_t unused6                          : 1 ;
        volatile uint32_t rstn_lle_hrstn_sw                : 1 ;
        volatile uint32_t rstn_lle_global_sw               : 1 ;
        volatile uint32_t rstn_lle_aes_sw                  : 1 ;
        volatile uint32_t rstn_lle_rfctrl_sw               : 1 ;
        volatile uint32_t reg_rstn_lle_domain_soft         : 1 ;
        volatile uint32_t rstn_keyscanner_sw               : 1 ;
        volatile uint32_t unused7                          : 5 ;
    }f;
}sys_ctrl_REG_rstu_cfg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sel_sys_dcachen                  : 1 ;
        volatile uint32_t sel_sys_icachen                  : 1 ;
        volatile uint32_t wdt_pause                        : 1 ;
        volatile uint32_t timer0_pit_pause                 : 1 ;
        volatile uint32_t timer1_pit_pause                 : 1 ;
        volatile uint32_t timer2_pit_pause                 : 1 ;
        volatile uint32_t spi_default_as_slave             : 1 ;
        volatile uint32_t spi_default_mode3                : 1 ;
        volatile uint32_t fpixc_reg                        : 1 ;
        volatile uint32_t fpofc_reg                        : 1 ;
        volatile uint32_t fpufc_reg                        : 1 ;
        volatile uint32_t fpioc_reg                        : 1 ;
        volatile uint32_t fpdzc_reg                        : 1 ;
        volatile uint32_t fpidc_reg                        : 1 ;
        volatile uint32_t fpudisable_reg                   : 1 ;
        volatile uint32_t spi1_default_as_slave            : 1 ;
        volatile uint32_t spi1_default_mode3               : 1 ;
        volatile uint32_t clk_force_on_ir                  : 1 ;
        volatile uint32_t lle_syswkup_req                  : 1 ;
        volatile uint32_t reg_bypass_wdt_por               : 1 ;
        volatile uint32_t reg_bypass_wdt_sys               : 1 ;
        volatile uint32_t reg_bypass_cpu_rst_req           : 1 ;
        volatile uint32_t reg_rstn_sys_global_soft         : 1 ;
        volatile uint32_t reg_rstn_sys_cgu_soft            : 1 ;
        volatile uint32_t reg_rstn_cpu_soft                : 1 ;
        volatile uint32_t reg_rstn_rf_soft                 : 1 ;
        volatile uint32_t unused8                          : 6 ;
    }f;
}sys_ctrl_REG_sys_ctrl_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t div_adc_denom                    : 6 ;
        volatile uint32_t div_adc_num                      : 6 ;
        volatile uint32_t div_adc_update                   : 1 ;
        volatile uint32_t sel_clk_32k_cpu                  : 1 ;
        volatile uint32_t sel_clk_pdm                      : 1 ;
        volatile uint32_t sel_clk_pwm                      : 1 ;
        volatile uint32_t sel_clk_ir                       : 1 ;
        volatile uint32_t sel_clk_sar_adc                  : 1 ;
        volatile uint32_t sel_clk_lle_main                 : 1 ;
        volatile uint32_t sel_clk_efuse                    : 1 ;
        volatile uint32_t div_timer_denom                  : 4 ;
        volatile uint32_t div_timer_update                 : 1 ;
        volatile uint32_t sel_clk_lle_sram2                : 1 ;
        volatile uint32_t sel_clk_lle_sram3                : 1 ;
        volatile uint32_t div_pwm_denom                    : 4 ;
        volatile uint32_t div_pwm_update                   : 1 ;
    }f;
}sys_ctrl_REG_cgu_cfg8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dma_ch0_sel                      : 4 ;
        volatile uint32_t dma_ch1_sel                      : 4 ;
        volatile uint32_t dma_ch2_sel                      : 4 ;
        volatile uint32_t dma_ch3_sel                      : 4 ;
        volatile uint32_t dma_ch4_sel                      : 4 ;
        volatile uint32_t dma_ch5_sel                      : 4 ;
        volatile uint32_t dma_ch6_sel                      : 4 ;
        volatile uint32_t dma_ch7_sel                      : 4 ;
    }f;
}sys_ctrl_REG_dma_ctrl_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dma_ch8_sel                      : 4 ;
        volatile uint32_t dma_ch9_sel                      : 4 ;
        volatile uint32_t dma_ch10_sel                     : 4 ;
        volatile uint32_t dma_ch11_sel                     : 4 ;
        volatile uint32_t dma_ch12_sel                     : 4 ;
        volatile uint32_t dma_ch13_sel                     : 4 ;
        volatile uint32_t dma_ch14_sel                     : 4 ;
        volatile uint32_t dma_ch15_sel                     : 4 ;
    }f;
}sys_ctrl_REG_dma_ctrl_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t pll_en_reg                       : 1 ;
        volatile uint32_t pll_div_pre_reg                  : 6 ;
        volatile uint32_t pll_div_loop_reg                 : 8 ;
        volatile uint32_t pll_div_out_reg                  : 6 ;
        volatile uint32_t pll_sw_vco_reg                   : 3 ;
        volatile uint32_t pll_sw_icp_reg                   : 2 ;
        volatile uint32_t pll_sw_lp_reg                    : 1 ;
        volatile uint32_t pll_sw_lock_reg                  : 1 ;
        volatile uint32_t pll_sw_vreg_reg                  : 2 ;
        volatile uint32_t pll_lock_reg                     : 1 ;
        volatile uint32_t unused9                          : 1 ;
    }f;
}sys_ctrl_REG_pll_ctrl_reg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_bypass_pmu_pdr_rstn          : 1 ;
        volatile uint32_t reg_bypass_pmu_pvd_rstn          : 1 ;
        volatile uint32_t reg_pmu_pvd_pol                  : 1 ;
        volatile uint32_t int_pmu_pdr_en                   : 1 ;
        volatile uint32_t int_pmu_pvd_en                   : 1 ;
        volatile uint32_t int_rcmfd_trim_done_en           : 1 ;
        volatile uint32_t int_lpc_neg_en                   : 1 ;
        volatile uint32_t int_lpc_pos_en                   : 1 ;
        volatile uint32_t int_xo_ready_en                  : 1 ;
        volatile uint32_t int_lle_domain_en                : 1 ;
        volatile uint32_t int_pmu_pdr                      : 1 ;
        volatile uint32_t int_pmu_pvd                      : 1 ;
        volatile uint32_t int_rcmfd_trim_done              : 1 ;
        volatile uint32_t int_lpc_neg                      : 1 ;
        volatile uint32_t int_lpc_pos                      : 1 ;
        volatile uint32_t int_xo_ready                     : 1 ;
        volatile uint32_t int_lle_domain                   : 1 ;
        volatile uint32_t unused10                         : 15;
    }f;
}sys_ctrl_REG_ana_ctrl_reg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t i2s_from_pdm                     : 1 ;
        volatile uint32_t unused11                         : 31;
    }f;
}sys_ctrl_REG_pdm_i2s_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t qdec_slclk_eq_sysclk             : 1 ;
        volatile uint32_t div_qdec_denom                   : 10;
        volatile uint32_t div_qdec_update                  : 1 ;
        volatile uint32_t fclk_qdec_gate                   : 1 ;
        volatile uint32_t fclk_qdec_gate_dr                : 1 ;
        volatile uint32_t rstn_qdec_sw                     : 1 ;
        volatile uint32_t sel_clk_qdec                     : 1 ;
        volatile uint32_t qdec_index_cfg                   : 3 ;
        volatile uint32_t qdec_index_sel                   : 1 ;
        volatile uint32_t qdec_index_reg_en                : 1 ;
        volatile uint32_t unused12                         : 11;
    }f;
}sys_ctrl_REG_qdec_cfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t div_pdm_denom                    : 6 ;
        volatile uint32_t div_pdm_num                      : 6 ;
        volatile uint32_t div_pdm_update                   : 1 ;
        volatile uint32_t unused13                         : 19;
    }f;
}sys_ctrl_REG_cgu_cfg9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_bond0                         : 1 ;
        volatile uint32_t io_bond1                         : 1 ;
        volatile uint32_t io_bond2                         : 1 ;
        volatile uint32_t test_mode                        : 1 ;
        volatile uint32_t rf_test_mode                     : 1 ;
        volatile uint32_t func_test_mode                   : 1 ;
        volatile uint32_t io_in_msb                        : 10;
        volatile uint32_t sys_io_wk_source_msb             : 9 ;
        volatile uint32_t unused14                         : 7 ;
    }f;
}sys_ctrl_REG_sys_io_status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sys_io_wk_source_lsb             : 32;
    }f;
}sys_ctrl_REG_sys_io_wk_source;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_in_lsb                        : 32;
    }f;
}sys_ctrl_REG_sys_io_in_status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sys_eco_0                        : 16;
        volatile uint32_t sys_eco_1                        : 16;
    }f;
}sys_ctrl_REG_sys_eco_reg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lock_sw                          : 1 ;
        volatile uint32_t lock_spi2ahb                     : 1 ;
        volatile uint32_t int_spi2ahb                      : 1 ;
        volatile uint32_t unused15                         : 29;
    }f;
}sys_ctrl_REG_sys_lock;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t div_usb_denom                    : 4 ;
        volatile uint32_t div_usb_update                   : 1 ;
        volatile uint32_t fclk_usb_gate                    : 1 ;
        volatile uint32_t fclk_usb_gate_dr                 : 1 ;
        volatile uint32_t ahb_clk_usb_gate                 : 1 ;
        volatile uint32_t ahb_clk_usb_gate_dr              : 1 ;
        volatile uint32_t rstn_usb_sw                      : 1 ;
        volatile uint32_t hrstn_usb_sw                     : 1 ;
        volatile uint32_t usb_id_sw                        : 1 ;
        volatile uint32_t usb_id_mode                      : 1 ;
        volatile uint32_t unused16                         : 19;
    }f;
}sys_ctrl_REG_usb_cfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t version_id                       : 4 ;
        volatile uint32_t metal_id                         : 5 ;
        volatile uint32_t bond_id                          : 3 ;
        volatile uint32_t prod_id                          : 20;
    }f;
}sys_ctrl_REG_chip_id;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cfg_sys_mem00                    : 5 ;
        volatile uint32_t cfg_sys_mem10                    : 5 ;
        volatile uint32_t cfg_lle_mem0                     : 5 ;
        volatile uint32_t cfg_lle_mem1                     : 5 ;
        volatile uint32_t cfg_rom                          : 5 ;
        volatile uint32_t unused17                         : 7 ;
    }f;
}sys_ctrl_REG_mem_cfg_reg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cfg_dcache_share_mem             : 5 ;
        volatile uint32_t cfg_dtag_mem                     : 5 ;
        volatile uint32_t cfg_icache_share_mem             : 5 ;
        volatile uint32_t cfg_itag_mem                     : 5 ;
        volatile uint32_t cfg_aes_mem                      : 5 ;
        volatile uint32_t cfg_usb_mem                      : 5 ;
        volatile uint32_t unused18                         : 2 ;
    }f;
}sys_ctrl_REG_mem_cfg_reg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dbg_en_clk                       : 1 ;
        volatile uint32_t dbg_sel_clk                      : 5 ;
        volatile uint32_t dbg_en_data                      : 1 ;
        volatile uint32_t dbg_sel_data                     : 5 ;
        volatile uint32_t dbg_swap                         : 5 ;
        volatile uint32_t unused19                         : 15;
    }f;
}sys_ctrl_REG_dbg_cfg_reg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_ppi_en                        : 1 ;
        volatile uint32_t c0_ppi_int                       : 1 ;
        volatile uint32_t c0_ppi_in_mask                   : 24;
        volatile uint32_t c0_ppi_int_mask                  : 1 ;
        volatile uint32_t unused20                         : 5 ;
    }f;
}sys_ctrl_REG_ppi_c0_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_ppi_out_mask                  : 16;
        volatile uint32_t unused21                         : 16;
    }f;
}sys_ctrl_REG_ppi_c0_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_ppi_en                        : 1 ;
        volatile uint32_t c1_ppi_int                       : 1 ;
        volatile uint32_t c1_ppi_in_mask                   : 24;
        volatile uint32_t c1_ppi_int_mask                  : 1 ;
        volatile uint32_t unused22                         : 5 ;
    }f;
}sys_ctrl_REG_ppi_c1_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_ppi_out_mask                  : 16;
        volatile uint32_t unused23                         : 16;
    }f;
}sys_ctrl_REG_ppi_c1_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_ppi_en                        : 1 ;
        volatile uint32_t c2_ppi_int                       : 1 ;
        volatile uint32_t c2_ppi_in_mask                   : 24;
        volatile uint32_t c2_ppi_int_mask                  : 1 ;
        volatile uint32_t unused24                         : 5 ;
    }f;
}sys_ctrl_REG_ppi_c2_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_ppi_out_mask                  : 16;
        volatile uint32_t unused25                         : 16;
    }f;
}sys_ctrl_REG_ppi_c2_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c3_ppi_en                        : 1 ;
        volatile uint32_t c3_ppi_int                       : 1 ;
        volatile uint32_t c3_ppi_in_mask                   : 24;
        volatile uint32_t c3_ppi_int_mask                  : 1 ;
        volatile uint32_t unused26                         : 5 ;
    }f;
}sys_ctrl_REG_ppi_c3_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c3_ppi_out_mask                  : 16;
        volatile uint32_t unused27                         : 16;
    }f;
}sys_ctrl_REG_ppi_c3_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t encry_key                        : 8 ;
        volatile uint32_t enable_encryption                : 1 ;
        volatile uint32_t unused28                         : 23;
    }f;
}sys_ctrl_REG_flash_encry;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sys_io_vddok                     : 1 ;
        volatile uint32_t unused29                         : 31;
    }f;
}sys_ctrl_REG_io_vdd_ok;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t vld_clk0_lle_main                : 1 ;
        volatile uint32_t vld_clk1_lle_main                : 1 ;
        volatile uint32_t vld_clk0_clk_rf_rc               : 1 ;
        volatile uint32_t vld_clk1_clk_rf_rc               : 1 ;
        volatile uint32_t vld_clk0_clk_32k_sw              : 1 ;
        volatile uint32_t vld_clk1_clk_32k_sw              : 1 ;
        volatile uint32_t vld_clk0_hclk                    : 1 ;
        volatile uint32_t vld_clk1_hclk                    : 1 ;
        volatile uint32_t vld_clk0_clk_32k                 : 1 ;
        volatile uint32_t vld_clk1_clk_32k                 : 1 ;
        volatile uint32_t vld_clk0_clk_timer0              : 1 ;
        volatile uint32_t vld_clk1_clk_timer0              : 1 ;
        volatile uint32_t vld_clk0_clk_timer1              : 1 ;
        volatile uint32_t vld_clk1_clk_timer1              : 1 ;
        volatile uint32_t vld_clk0_clk_timer2              : 1 ;
        volatile uint32_t vld_clk1_clk_timer2              : 1 ;
        volatile uint32_t vld_clk0_clk_ks                  : 1 ;
        volatile uint32_t vld_clk1_clk_ks                  : 1 ;
        volatile uint32_t vld_clk0_clk_pwm                 : 1 ;
        volatile uint32_t vld_clk1_clk_pwm                 : 1 ;
        volatile uint32_t vld_clk0_clk_uart0               : 1 ;
        volatile uint32_t vld_clk1_clk_uart0               : 1 ;
        volatile uint32_t vld_clk0_clk_uart1               : 1 ;
        volatile uint32_t vld_clk1_clk_uart1               : 1 ;
        volatile uint32_t vld_clk0_clk_spi                 : 1 ;
        volatile uint32_t vld_clk1_clk_spi                 : 1 ;
        volatile uint32_t vld_clk0_clk_flash               : 1 ;
        volatile uint32_t vld_clk1_clk_flash               : 1 ;
        volatile uint32_t vld_clk0_clk_qspi                : 1 ;
        volatile uint32_t vld_clk1_clk_qspi                : 1 ;
        volatile uint32_t vld_clk0_clk_i2s                 : 1 ;
        volatile uint32_t vld_clk1_clk_i2s                 : 1 ;
    }f;
}sys_ctrl_REG_vld_clk_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t vld_clk0_clk_qdec                : 1 ;
        volatile uint32_t vld_clk1_clk_qdec                : 1 ;
        volatile uint32_t vld_clk0_glc_fast                : 1 ;
        volatile uint32_t vld_clk1_glc_fast                : 1 ;
        volatile uint32_t unused30                         : 28;
    }f;
}sys_ctrl_REG_vld_clk_rd1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rd_io_o_lsb                  : 32;
    }f;
}sys_ctrl_REG_reg_rd_io0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rd_io_oe_lsb                 : 32;
    }f;
}sys_ctrl_REG_reg_rd_io1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rd_io_ie_lsb                 : 32;
    }f;
}sys_ctrl_REG_reg_rd_io2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rd_io_o_msb                  : 10;
        volatile uint32_t reg_rd_io_oe_msb                 : 10;
        volatile uint32_t reg_rd_io_ie_msb                 : 10;
        volatile uint32_t unused31                         : 2 ;
    }f;
}sys_ctrl_REG_reg_rd_io3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t div_clkout_func_denom            : 10;
        volatile uint32_t div_clkout_func_update           : 1 ;
        volatile uint32_t en_clkout_func                   : 1 ;
        volatile uint32_t unused32                         : 20;
    }f;
}sys_ctrl_REG_clk_out_func_cfg;

typedef struct
{
volatile sys_ctrl_REG_cgu_cfg0                                       cgu_cfg0                      ; //0x0
volatile sys_ctrl_REG_cgu_cfg1                                       cgu_cfg1                      ; //0x4
volatile sys_ctrl_REG_cgu_cfg2                                       cgu_cfg2                      ; //0x8
volatile sys_ctrl_REG_cgu_cfg3                                       cgu_cfg3                      ; //0xC
volatile sys_ctrl_REG_cgu_cfg4                                       cgu_cfg4                      ; //0x10
volatile sys_ctrl_REG_cgu_cfg5                                       cgu_cfg5                      ; //0x14
volatile sys_ctrl_REG_cgu_cfg6                                       cgu_cfg6                      ; //0x18
volatile sys_ctrl_REG_cgu_cfg7                                       cgu_cfg7                      ; //0x1C
volatile sys_ctrl_REG_rstu_cfg0                                      rstu_cfg0                     ; //0x20
volatile sys_ctrl_REG_rstu_cfg1                                      rstu_cfg1                     ; //0x24
volatile sys_ctrl_REG_sys_ctrl_0                                     sys_ctrl_0                    ; //0x28
volatile sys_ctrl_REG_cgu_cfg8                                       cgu_cfg8                      ; //0x2C
volatile sys_ctrl_REG_dma_ctrl_0                                     dma_ctrl_0                    ; //0x30
volatile sys_ctrl_REG_dma_ctrl_1                                     dma_ctrl_1                    ; //0x34
volatile uint32_t                                                    null_reg0                     ; //0x38
volatile uint32_t                                                    null_reg1                     ; //0x3C
volatile sys_ctrl_REG_pll_ctrl_reg                                   pll_ctrl_reg                  ; //0x40
volatile sys_ctrl_REG_ana_ctrl_reg                                   ana_ctrl_reg                  ; //0x44
volatile uint32_t                                                    null_reg2                     ; //0x48
volatile uint32_t                                                    null_reg3                     ; //0x4C
volatile sys_ctrl_REG_pdm_i2s_ctrl                                   pdm_i2s_ctrl                  ; //0x50
volatile sys_ctrl_REG_qdec_cfg                                       qdec_cfg                      ; //0x54
volatile sys_ctrl_REG_cgu_cfg9                                       cgu_cfg9                      ; //0x58
volatile uint32_t                                                    null_reg4                     ; //0x5C
volatile sys_ctrl_REG_sys_io_status                                  sys_io_status                 ; //0x60
volatile sys_ctrl_REG_sys_io_wk_source                               sys_io_wk_source              ; //0x64
volatile sys_ctrl_REG_sys_io_in_status                               sys_io_in_status              ; //0x68
volatile uint32_t                                                    null_reg5                     ; //0x6C
volatile uint32_t                                                    null_reg6                     ; //0x70
volatile sys_ctrl_REG_sys_eco_reg                                    sys_eco_reg                   ; //0x74
volatile uint32_t                                                    null_reg7                     ; //0x78
volatile sys_ctrl_REG_sys_lock                                       sys_lock                      ; //0x7C
volatile sys_ctrl_REG_usb_cfg                                        usb_cfg                       ; //0x80
volatile uint32_t                                                    null_reg8                     ; //0x84
volatile uint32_t                                                    null_reg9                     ; //0x88
volatile uint32_t                                                    null_reg10                    ; //0x8C
volatile uint32_t                                                    null_reg11                    ; //0x90
volatile uint32_t                                                    null_reg12                    ; //0x94
volatile uint32_t                                                    null_reg13                    ; //0x98
volatile uint32_t                                                    null_reg14                    ; //0x9C
volatile uint32_t                                                    null_reg15                    ; //0xA0
volatile uint32_t                                                    null_reg16                    ; //0xA4
volatile uint32_t                                                    null_reg17                    ; //0xA8
volatile uint32_t                                                    null_reg18                    ; //0xAC
volatile uint32_t                                                    null_reg19                    ; //0xB0
volatile uint32_t                                                    null_reg20                    ; //0xB4
volatile uint32_t                                                    null_reg21                    ; //0xB8
volatile uint32_t                                                    null_reg22                    ; //0xBC
volatile uint32_t                                                    null_reg23                    ; //0xC0
volatile uint32_t                                                    null_reg24                    ; //0xC4
volatile uint32_t                                                    null_reg25                    ; //0xC8
volatile uint32_t                                                    null_reg26                    ; //0xCC
volatile uint32_t                                                    null_reg27                    ; //0xD0
volatile uint32_t                                                    null_reg28                    ; //0xD4
volatile uint32_t                                                    null_reg29                    ; //0xD8
volatile uint32_t                                                    null_reg30                    ; //0xDC
volatile uint32_t                                                    null_reg31                    ; //0xE0
volatile uint32_t                                                    null_reg32                    ; //0xE4
volatile uint32_t                                                    null_reg33                    ; //0xE8
volatile uint32_t                                                    null_reg34                    ; //0xEC
volatile uint32_t                                                    null_reg35                    ; //0xF0
volatile uint32_t                                                    null_reg36                    ; //0xF4
volatile uint32_t                                                    null_reg37                    ; //0xF8
volatile uint32_t                                                    null_reg38                    ; //0xFC
volatile uint32_t                                                    null_reg39                    ; //0x100
volatile uint32_t                                                    null_reg40                    ; //0x104
volatile uint32_t                                                    null_reg41                    ; //0x108
volatile uint32_t                                                    null_reg42                    ; //0x10C
volatile uint32_t                                                    null_reg43                    ; //0x110
volatile uint32_t                                                    null_reg44                    ; //0x114
volatile uint32_t                                                    null_reg45                    ; //0x118
volatile uint32_t                                                    null_reg46                    ; //0x11C
volatile uint32_t                                                    null_reg47                    ; //0x120
volatile uint32_t                                                    null_reg48                    ; //0x124
volatile uint32_t                                                    null_reg49                    ; //0x128
volatile uint32_t                                                    null_reg50                    ; //0x12C
volatile uint32_t                                                    null_reg51                    ; //0x130
volatile uint32_t                                                    null_reg52                    ; //0x134
volatile uint32_t                                                    null_reg53                    ; //0x138
volatile uint32_t                                                    null_reg54                    ; //0x13C
volatile uint32_t                                                    null_reg55                    ; //0x140
volatile uint32_t                                                    null_reg56                    ; //0x144
volatile uint32_t                                                    null_reg57                    ; //0x148
volatile uint32_t                                                    null_reg58                    ; //0x14C
volatile uint32_t                                                    null_reg59                    ; //0x150
volatile uint32_t                                                    null_reg60                    ; //0x154
volatile uint32_t                                                    null_reg61                    ; //0x158
volatile uint32_t                                                    null_reg62                    ; //0x15C
volatile sys_ctrl_REG_chip_id                                        chip_id                       ; //0x160
volatile uint32_t                                                    null_reg63                    ; //0x164
volatile uint32_t                                                    null_reg64                    ; //0x168
volatile uint32_t                                                    null_reg65                    ; //0x16C
volatile sys_ctrl_REG_mem_cfg_reg0                                   mem_cfg_reg0                  ; //0x170
volatile sys_ctrl_REG_mem_cfg_reg1                                   mem_cfg_reg1                  ; //0x174
volatile uint32_t                                                    null_reg66                    ; //0x178
volatile uint32_t                                                    null_reg67                    ; //0x17C
volatile uint32_t                                                    null_reg68                    ; //0x180
volatile uint32_t                                                    null_reg69                    ; //0x184
volatile uint32_t                                                    null_reg70                    ; //0x188
volatile uint32_t                                                    null_reg71                    ; //0x18C
volatile sys_ctrl_REG_dbg_cfg_reg                                    dbg_cfg_reg                   ; //0x190
volatile uint32_t                                                    null_reg72                    ; //0x194
volatile uint32_t                                                    null_reg73                    ; //0x198
volatile uint32_t                                                    null_reg74                    ; //0x19C
volatile sys_ctrl_REG_ppi_c0_ctrl0                                   ppi_c0_ctrl0                  ; //0x1A0
volatile sys_ctrl_REG_ppi_c0_ctrl1                                   ppi_c0_ctrl1                  ; //0x1A4
volatile sys_ctrl_REG_ppi_c1_ctrl0                                   ppi_c1_ctrl0                  ; //0x1A8
volatile sys_ctrl_REG_ppi_c1_ctrl1                                   ppi_c1_ctrl1                  ; //0x1AC
volatile sys_ctrl_REG_ppi_c2_ctrl0                                   ppi_c2_ctrl0                  ; //0x1B0
volatile sys_ctrl_REG_ppi_c2_ctrl1                                   ppi_c2_ctrl1                  ; //0x1B4
volatile sys_ctrl_REG_ppi_c3_ctrl0                                   ppi_c3_ctrl0                  ; //0x1B8
volatile sys_ctrl_REG_ppi_c3_ctrl1                                   ppi_c3_ctrl1                  ; //0x1BC
volatile sys_ctrl_REG_flash_encry                                    flash_encry                   ; //0x1C0
volatile sys_ctrl_REG_io_vdd_ok                                      io_vdd_ok                     ; //0x1C4
volatile sys_ctrl_REG_vld_clk_rd                                     vld_clk_rd                    ; //0x1C8
volatile sys_ctrl_REG_vld_clk_rd1                                    vld_clk_rd1                   ; //0x1CC
volatile sys_ctrl_REG_reg_rd_io0                                     reg_rd_io0                    ; //0x1D0
volatile sys_ctrl_REG_reg_rd_io1                                     reg_rd_io1                    ; //0x1D4
volatile sys_ctrl_REG_reg_rd_io2                                     reg_rd_io2                    ; //0x1D8
volatile sys_ctrl_REG_reg_rd_io3                                     reg_rd_io3                    ; //0x1DC
volatile sys_ctrl_REG_clk_out_func_cfg                               clk_out_func_cfg              ; //0x1E0
}sys_ctrl_REG_H;

#endif //sys_ctrl_REG__H
