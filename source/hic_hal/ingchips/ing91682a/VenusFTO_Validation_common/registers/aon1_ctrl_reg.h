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
// File         : aon1_ctrl_reg.h
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
#ifndef AON1_CTRL_REG__H
#define AON1_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_reg_en_rc32k                 : 1 ;
        volatile uint32_t int_pmu_ldo3_core_reg            : 4 ;
        volatile uint32_t int_reg_sel_clk_32k_source       : 1 ;
        volatile uint32_t int_reg_en_clk_32k_cnt           : 1 ;
        volatile uint32_t int_reg_io_osc_en                : 1 ;
        volatile uint32_t int_reg_io_osc_byp               : 1 ;
        volatile uint32_t int_reg_io_osc_cfg               : 8 ;
        volatile uint32_t int_reg_rc32k_iopsel             : 1 ;
        volatile uint32_t int_reg_rc32k_freq_ctune         : 6 ;
        volatile uint32_t int_reg_rc32k_freq_ftune         : 4 ;
        volatile uint32_t int_pmu_ldo1_aon_reg             : 4 ;
    }f;
}aon1_ctrl_REG_aon1_reg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_reg_cnt_tgt                  : 32;
    }f;
}aon1_ctrl_REG_aon1_reg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_pmu_pvd_reg                  : 4 ;
        volatile uint32_t int_pmu_mbgp_reg                 : 7 ;
        volatile uint32_t int_pmu_ldo2_flash_reg           : 4 ;
        volatile uint32_t int_pmu_dcdc_reg1                : 8 ;
        volatile uint32_t int_pmu_dcdc_reg2                : 8 ;
        volatile uint32_t pmu_ldo2_flash_reg_en            : 1 ;
    }f;
}aon1_ctrl_REG_aon1_reg2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rcmfd_en_mode                : 1 ;
        volatile uint32_t reg_rcmfd_en                     : 1 ;
        volatile uint32_t reg_boot_pin_clr                 : 1 ;
        volatile uint32_t reg_dcdc_bypass                  : 1 ;
        volatile uint32_t reg_rstn_rtc_cnt                 : 1 ;
        volatile uint32_t reg_prstn_aon1_ctrl              : 1 ;
        volatile uint32_t int_pmu_ldo4_rf_reg              : 5 ;
        volatile uint32_t int_pmu_reg_vref12               : 5 ;
        volatile uint32_t aon1_reg3_reserved               : 16;
    }f;
}aon1_ctrl_REG_aon1_reg3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_aon1_rcmfd_ostb              : 4 ;
        volatile uint32_t reg_aon1_rcmfd_sw                : 1 ;
        volatile uint32_t reg_aon1_slp2fact_en             : 1 ;
        volatile uint32_t reg_ana_fast_boot                : 1 ;
        volatile uint32_t reg_aon1_io_latch                : 1 ;
        volatile uint32_t reg_sel_clkrf_rcn_sys            : 1 ;
        volatile uint32_t int_reg_ext_int_pol              : 1 ;
        volatile uint32_t int_reg_ext_int_en               : 1 ;
        volatile uint32_t int_reg_cnt_slp_en               : 1 ;
        volatile uint32_t reg_aon1_deepsleep_en            : 1 ;
        volatile uint32_t reg_dpslp_ramret                 : 1 ;
        volatile uint32_t lle_cte_ant_pattern_aon_en       : 1 ;
        volatile uint32_t reg_en_clk_32k_rstu              : 1 ;
        volatile uint32_t reg_pmu_pdr_pd                   : 1 ;
        volatile uint32_t reg_pmu_pvd_en                   : 1 ;
        volatile uint32_t dly_ldo4_en_slp2act_aon1         : 8 ;
        volatile uint32_t aon1_reg4_reserved3              : 6 ;
    }f;
}aon1_ctrl_REG_aon1_reg4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_aon1_boot                    : 32;
    }f;
}aon1_ctrl_REG_aon1_boot;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_mem_ps_sel_pd_slp2act        : 16;
        volatile uint32_t dly_ldo2_en_slp2act              : 4 ;
        volatile uint32_t dly_ldo3_en_slp2act              : 4 ;
        volatile uint32_t dly_pd_sram_slp2act              : 6 ;
        volatile uint32_t dly_ldo4_en_slp2act_aon1_en      : 1 ;
        volatile uint32_t reg_aon_bond_pe                  : 1 ;
    }f;
}aon1_ctrl_REG_aon1_reg5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t en_actcnt                        : 1 ;
        volatile uint32_t clr_actcnt                       : 1 ;
        volatile uint32_t int_actcnt_mask                  : 1 ;
        volatile uint32_t int_actcnt_32k                   : 1 ;
        volatile uint32_t reg_bypass_pmu_pdr_rstn          : 1 ;
        volatile uint32_t reg_bypass_pmu_pvd_rstn          : 1 ;
        volatile uint32_t reg_dpslp_disable_32k            : 1 ;
        volatile uint32_t io_bond0_pe_aon                  : 1 ;
        volatile uint32_t io_bond1_pe_aon                  : 1 ;
        volatile uint32_t io_bond2_pe_aon                  : 1 ;
        volatile uint32_t unused0                          : 22;
    }f;
}aon1_ctrl_REG_aon1_cnt_int;

typedef struct
{
volatile aon1_ctrl_REG_aon1_reg0                                     aon1_reg0                     ; //0x0
volatile aon1_ctrl_REG_aon1_reg1                                     aon1_reg1                     ; //0x4
volatile aon1_ctrl_REG_aon1_reg2                                     aon1_reg2                     ; //0x8
volatile aon1_ctrl_REG_aon1_reg3                                     aon1_reg3                     ; //0xC
volatile aon1_ctrl_REG_aon1_reg4                                     aon1_reg4                     ; //0x10
volatile aon1_ctrl_REG_aon1_boot                                     aon1_boot                     ; //0x14
volatile aon1_ctrl_REG_aon1_reg5                                     aon1_reg5                     ; //0x18
volatile aon1_ctrl_REG_aon1_cnt_int                                  aon1_cnt_int                  ; //0x1C
}aon1_ctrl_REG_H;

#endif //aon1_ctrl_REG__H
