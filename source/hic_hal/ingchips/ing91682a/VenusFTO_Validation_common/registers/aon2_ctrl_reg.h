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
// File         : aon2_ctrl_reg.h
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
#ifndef AON2_CTRL_REG__H
#define AON2_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t pmu_buck_en_act                  : 1 ;
        volatile uint32_t pmu_mbgp_en_act                  : 1 ;
        volatile uint32_t unused0                          : 1 ;
        volatile uint32_t pmu_ldo3_en_act                  : 1 ;
        volatile uint32_t pmu_ldo2_en_act                  : 1 ;
        volatile uint32_t pmu_ldo4_en_act                  : 1 ;
        volatile uint32_t pmu_mem_ps_sel_act               : 1 ;
        volatile uint32_t unused1                          : 2 ;
        volatile uint32_t sw_pd_sysdomain_act              : 1 ;
        volatile uint32_t iso_sys2rtc_act                  : 1 ;
        volatile uint32_t seq_xo_en_dig_act                : 1 ;
        volatile uint32_t isob_dig2rf_act                  : 1 ;
        volatile uint32_t rstn_sys_act                     : 1 ;
        volatile uint32_t cg_sys_act                       : 1 ;
        volatile uint32_t seq_ldo_ant_hiz_act              : 1 ;
        volatile uint32_t sw_pd_lledomain_act              : 1 ;
        volatile uint32_t iso_lle2rtc_act                  : 1 ;
        volatile uint32_t rstn_lle_act                     : 1 ;
        volatile uint32_t cg_lle_act                       : 1 ;
        volatile uint32_t unused2                          : 12;
    }f;
}aon2_ctrl_REG_power_act_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sram0_slp_act                    : 1 ;
        volatile uint32_t sram1_slp_act                    : 1 ;
        volatile uint32_t sram2_slp_act                    : 1 ;
        volatile uint32_t sram3_slp_act                    : 1 ;
        volatile uint32_t sram4_slp_act                    : 1 ;
        volatile uint32_t unused3                          : 3 ;
        volatile uint32_t sram0_ds_act                     : 1 ;
        volatile uint32_t sram1_ds_act                     : 1 ;
        volatile uint32_t sram2_ds_act                     : 1 ;
        volatile uint32_t sram3_ds_act                     : 1 ;
        volatile uint32_t sram4_ds_act                     : 1 ;
        volatile uint32_t unused4                          : 3 ;
        volatile uint32_t sram0_sd_act                     : 1 ;
        volatile uint32_t sram1_sd_act                     : 1 ;
        volatile uint32_t sram2_sd_act                     : 1 ;
        volatile uint32_t sram3_sd_act                     : 1 ;
        volatile uint32_t sram4_sd_act                     : 1 ;
        volatile uint32_t sram5_sd_act                     : 1 ;
        volatile uint32_t unused5                          : 10;
    }f;
}aon2_ctrl_REG_power_act_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t pmu_buck_en_slp                  : 1 ;
        volatile uint32_t pmu_mbgp_en_slp                  : 1 ;
        volatile uint32_t unused6                          : 1 ;
        volatile uint32_t pmu_ldo3_en_slp                  : 1 ;
        volatile uint32_t pmu_ldo2_en_slp                  : 1 ;
        volatile uint32_t pmu_ldo4_en_slp                  : 1 ;
        volatile uint32_t pmu_mem_ps_sel_slp               : 1 ;
        volatile uint32_t unused7                          : 2 ;
        volatile uint32_t sw_pd_sysdomain_slp              : 1 ;
        volatile uint32_t iso_sys2rtc_slp                  : 1 ;
        volatile uint32_t seq_xo_en_dig_slp                : 1 ;
        volatile uint32_t isob_dig2rf_slp                  : 1 ;
        volatile uint32_t rstn_sys_slp                     : 1 ;
        volatile uint32_t cg_sys_slp                       : 1 ;
        volatile uint32_t seq_ldo_ant_hiz_slp              : 1 ;
        volatile uint32_t sw_pd_lledomain_slp              : 1 ;
        volatile uint32_t iso_lle2rtc_slp                  : 1 ;
        volatile uint32_t rstn_lle_slp                     : 1 ;
        volatile uint32_t cg_lle_slp                       : 1 ;
        volatile uint32_t unused8                          : 12;
    }f;
}aon2_ctrl_REG_power_slp_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sram0_slp_slp                    : 1 ;
        volatile uint32_t sram1_slp_slp                    : 1 ;
        volatile uint32_t sram2_slp_slp                    : 1 ;
        volatile uint32_t sram3_slp_slp                    : 1 ;
        volatile uint32_t sram4_slp_slp                    : 1 ;
        volatile uint32_t unused9                          : 3 ;
        volatile uint32_t sram0_ds_slp                     : 1 ;
        volatile uint32_t sram1_ds_slp                     : 1 ;
        volatile uint32_t sram2_ds_slp                     : 1 ;
        volatile uint32_t sram3_ds_slp                     : 1 ;
        volatile uint32_t sram4_ds_slp                     : 1 ;
        volatile uint32_t unused10                         : 3 ;
        volatile uint32_t sram0_sd_slp                     : 1 ;
        volatile uint32_t sram1_sd_slp                     : 1 ;
        volatile uint32_t sram2_sd_slp                     : 1 ;
        volatile uint32_t sram3_sd_slp                     : 1 ;
        volatile uint32_t sram4_sd_slp                     : 1 ;
        volatile uint32_t sram5_sd_slp                     : 1 ;
        volatile uint32_t unused11                         : 10;
    }f;
}aon2_ctrl_REG_power_slp_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_mbgp_en_slp2act              : 3 ;
        volatile uint32_t dly_dcdc_en_slp2act              : 5 ;
        volatile uint32_t dly_ldo4_en_slp2act              : 8 ;
        volatile uint32_t unused12                         : 16;
    }f;
}aon2_ctrl_REG_dly_slp2act_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_sys_cg_slp2act               : 4 ;
        volatile uint32_t dly_sys_rstn_slp2act             : 4 ;
        volatile uint32_t dly_sys_pd_slp2act               : 5 ;
        volatile uint32_t dly_isob_dig2rf_slp2act          : 3 ;
        volatile uint32_t dly_sys_iso_slp2act              : 3 ;
        volatile uint32_t dly_sram_sd_slp2act              : 3 ;
        volatile uint32_t unused13                         : 10;
    }f;
}aon2_ctrl_REG_dly_slp2act_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_mbgp_en_act2slp              : 3 ;
        volatile uint32_t dly_dcdc_en_act2slp              : 3 ;
        volatile uint32_t dly_ldo4_en_act2slp              : 3 ;
        volatile uint32_t dly_ldo3_en_act2slp              : 3 ;
        volatile uint32_t dly_ldo2_en_act2slp              : 3 ;
        volatile uint32_t dly_mem_ps_sel_pd_act2slp        : 3 ;
        volatile uint32_t unused14                         : 14;
    }f;
}aon2_ctrl_REG_dly_act2slp_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_sys_rstn_act2slp             : 3 ;
        volatile uint32_t dly_sys_cg_act2slp               : 3 ;
        volatile uint32_t dly_isob_dig2rf_act2slp          : 3 ;
        volatile uint32_t dly_sys_iso_act2slp              : 3 ;
        volatile uint32_t dly_sys_pd_act2slp               : 3 ;
        volatile uint32_t dly_pd_sram_act2slp              : 3 ;
        volatile uint32_t dly_sram_sd_act2slp              : 3 ;
        volatile uint32_t dly_seq_xo_en_act2slp            : 3 ;
        volatile uint32_t unused15                         : 8 ;
    }f;
}aon2_ctrl_REG_dly_act2slp_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused16                         : 5 ;
        volatile uint32_t rf_seq_isobrtc                   : 1 ;
        volatile uint32_t reg_rstn_global_soft             : 1 ;
        volatile uint32_t unused17                         : 1 ;
        volatile uint32_t reg_rstn_tbcu_soft               : 1 ;
        volatile uint32_t reg_rstn_lpc_soft                : 1 ;
        volatile uint32_t reg_rstn_aon_ctrl_soft           : 1 ;
        volatile uint32_t reg_rstn_rtc_soft                : 1 ;
        volatile uint32_t reg_rstn_glc_soft                : 1 ;
        volatile uint32_t reg_bypass_wdt_aon2              : 1 ;
        volatile uint32_t reg_bypass_wdt_aon1              : 1 ;
        volatile uint32_t reg_bypass_wdt_sys               : 1 ;
        volatile uint32_t reg_bypass_wdt_por_cpu           : 1 ;
        volatile uint32_t unused18                         : 2 ;
        volatile uint32_t reg_en_clk_rtc                   : 1 ;
        volatile uint32_t reg_en_clk_tbcu                  : 1 ;
        volatile uint32_t reg_en_clk_glc                   : 1 ;
        volatile uint32_t unused19                         : 10;
    }f;
}aon2_ctrl_REG_pwr_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused20                         : 1 ;
        volatile uint32_t io_ie_aon_4to1                   : 4 ;
        volatile uint32_t unused21                         : 2 ;
        volatile uint32_t io_ie_aon_17to7                  : 11;
        volatile uint32_t unused22                         : 6 ;
        volatile uint32_t io_ie_aon_25to24                 : 2 ;
        volatile uint32_t unused23                         : 3 ;
        volatile uint32_t io_ie_aon_31to29                 : 3 ;
    }f;
}aon2_ctrl_REG_aon1_reg4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_ie_aon_35to32                 : 4 ;
        volatile uint32_t unused24                         : 28;
    }f;
}aon2_ctrl_REG_aon1_reg6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused25                         : 1 ;
        volatile uint32_t io_pe_aon_4to1                   : 4 ;
        volatile uint32_t unused26                         : 2 ;
        volatile uint32_t io_pe_aon_17to7                  : 11;
        volatile uint32_t unused27                         : 6 ;
        volatile uint32_t io_pe_aon_25to24                 : 2 ;
        volatile uint32_t unused28                         : 3 ;
        volatile uint32_t io_pe_aon_31to29                 : 3 ;
    }f;
}aon2_ctrl_REG_aon1_reg7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_pe_aon_35to32                 : 4 ;
        volatile uint32_t unused29                         : 28;
    }f;
}aon2_ctrl_REG_aon1_reg8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused30                         : 1 ;
        volatile uint32_t io_ps_aon_4to1                   : 4 ;
        volatile uint32_t unused31                         : 2 ;
        volatile uint32_t io_ps_aon_17to7                  : 11;
        volatile uint32_t unused32                         : 6 ;
        volatile uint32_t io_ps_aon_25to24                 : 2 ;
        volatile uint32_t unused33                         : 3 ;
        volatile uint32_t io_ps_aon_31to29                 : 3 ;
    }f;
}aon2_ctrl_REG_aon1_reg9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_ps_aon_35to32                 : 4 ;
        volatile uint32_t unused34                         : 28;
    }f;
}aon2_ctrl_REG_aon1_reg10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon1_rtc_cnt_lsb                 : 32;
    }f;
}aon2_ctrl_REG_aon1_reg_rd11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon1_rtc_cnt_msb                 : 11;
        volatile uint32_t unused35                         : 21;
    }f;
}aon2_ctrl_REG_aon1_reg_rd12;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cnt_slp                          : 32;
    }f;
}aon2_ctrl_REG_aon1_reg_rd13;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t boot_force                       : 1 ;
        volatile uint32_t boot_power_up                    : 1 ;
        volatile uint32_t pmu_buck_ok                      : 1 ;
        volatile uint32_t pmu_v12core_ok                   : 1 ;
        volatile uint32_t pmu_v18flash_ok                  : 1 ;
        volatile uint32_t pmu_v15_rf_ok                    : 1 ;
        volatile uint32_t unused36                         : 1 ;
        volatile uint32_t pmu_vmem_ok                      : 1 ;
        volatile uint32_t pmu_pdr                          : 1 ;
        volatile uint32_t pmu_pvd                          : 1 ;
        volatile uint32_t pmu_ldo1_aon_ok                  : 1 ;
        volatile uint32_t pmu_buck_oc                      : 1 ;
        volatile uint32_t power_cs                         : 4 ;
        volatile uint32_t unused37                         : 3 ;
        volatile uint32_t aon1_reg_latch_en                : 1 ;
        volatile uint32_t io_aon_bond                      : 1 ;
        volatile uint32_t unused38                         : 11;
    }f;
}aon2_ctrl_REG_pwr_ctrl_status0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t eco_0                            : 32;
    }f;
}aon2_ctrl_REG_aon_eco_reg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t eco_1                            : 32;
    }f;
}aon2_ctrl_REG_aon_eco_reg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_pol_17to0             : 18;
        volatile uint32_t unused39                         : 3 ;
        volatile uint32_t reg_wakeup_pol_25to21            : 5 ;
        volatile uint32_t unused40                         : 3 ;
        volatile uint32_t reg_wakeup_pol_31to29            : 3 ;
    }f;
}aon2_ctrl_REG_sys_wakeup_pol0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_pol_msb               : 9 ;
        volatile uint32_t unused41                         : 23;
    }f;
}aon2_ctrl_REG_sys_wakeup_pol1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_en_17to0              : 18;
        volatile uint32_t unused42                         : 3 ;
        volatile uint32_t reg_wakeup_en_25to21             : 5 ;
        volatile uint32_t unused43                         : 3 ;
        volatile uint32_t reg_wakeup_en_31to29             : 3 ;
    }f;
}aon2_ctrl_REG_sys_wakeup_en0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_en_msb                : 9 ;
        volatile uint32_t unused44                         : 23;
    }f;
}aon2_ctrl_REG_sys_wakeup_en1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_mode_0                : 32;
    }f;
}aon2_ctrl_REG_sys_wakeup_mode0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_mode_1_0              : 4 ;
        volatile uint32_t unused45                         : 6 ;
        volatile uint32_t reg_wakeup_mode_1_1              : 10;
        volatile uint32_t unused46                         : 6 ;
        volatile uint32_t reg_wakeup_mode_1_2              : 6 ;
    }f;
}aon2_ctrl_REG_sys_wakeup_mode1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_wakeup_mode_2                : 18;
        volatile uint32_t unused47                         : 14;
    }f;
}aon2_ctrl_REG_sys_wakeup_mode2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon2_wk_source                   : 2 ;
        volatile uint32_t aon2_io_wk_source                : 8 ;
        volatile uint32_t unused48                         : 22;
    }f;
}aon2_ctrl_REG_aon2_wk_source;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon2_io_aon_i_low                : 32;
    }f;
}aon2_ctrl_REG_aon2_io_aon_i_low;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon2_io_aon_i_high               : 6 ;
        volatile uint32_t unused49                         : 26;
    }f;
}aon2_ctrl_REG_aon2_io_aon_i_high;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused50                         : 2 ;
        volatile uint32_t seq_xo_en_clk_aux_rtc            : 1 ;
        volatile uint32_t reg_xo_mode_rtc                  : 1 ;
        volatile uint32_t reg_xo_cap_off_rtc               : 1 ;
        volatile uint32_t reg_xo_cnt_off_rtc               : 1 ;
        volatile uint32_t reg_xo_dyn_cap_rtc               : 1 ;
        volatile uint32_t reg_xo_dyn_isel_rtc              : 1 ;
        volatile uint32_t reg_xo_force_amp_rtc             : 1 ;
        volatile uint32_t reg_xo_ldo_boost_rtc             : 1 ;
        volatile uint32_t reg_xo_ldo_bypass_rtc            : 1 ;
        volatile uint32_t reg_xo_ldo_force_rtc             : 1 ;
        volatile uint32_t reg_xo_sense_off_rtc             : 1 ;
        volatile uint32_t seq_rcsoc_pup_rtc                : 1 ;
        volatile uint32_t reg_rcosc_ready_force_rtc        : 1 ;
        volatile uint32_t unused51                         : 2 ;
        volatile uint32_t reg_xo_cdac_rtc                  : 6 ;
        volatile uint32_t reg_xo_ldo_trim_rtc              : 2 ;
        volatile uint32_t reg_xo_isel_rtc                  : 4 ;
        volatile uint32_t reg_xo_cnt_rtc                   : 2 ;
        volatile uint32_t sel_rtc_dig_n                    : 1 ;
    }f;
}aon2_ctrl_REG_rf_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rcosc_trim_rtc               : 6 ;
        volatile uint32_t reg_rcosc_trim_fine_rtc          : 2 ;
        volatile uint32_t reg_sparertc_rtc                 : 3 ;
        volatile uint32_t unused52                         : 1 ;
        volatile uint32_t reg_lle_clk_config               : 6 ;
        volatile uint32_t cg_rc_div_out                    : 1 ;
        volatile uint32_t div_rc_denom                     : 3 ;
        volatile uint32_t div_rc_update                    : 1 ;
        volatile uint32_t unused53                         : 9 ;
    }f;
}aon2_ctrl_REG_rf_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t usb_phy_ctrl                     : 9 ;
        volatile uint32_t unused54                         : 23;
    }f;
}aon2_ctrl_REG_usb11_phy_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_lpc_ctrl                     : 16;
        volatile uint32_t lpc_out                          : 1 ;
        volatile uint32_t unused55                         : 15;
    }f;
}aon2_ctrl_REG_comparator_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rcmfd_ctrl_0                 : 17;
        volatile uint32_t unused56                         : 2 ;
        volatile uint32_t reg_rcmfd_out                    : 13;
    }f;
}aon2_ctrl_REG_rcmfd_config_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_rcmfd_ctrl_1                 : 24;
        volatile uint32_t unused57                         : 8 ;
    }f;
}aon2_ctrl_REG_rcmfd_config_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_sys_pd_fact2lact             : 4 ;
        volatile uint32_t unused58                         : 4 ;
        volatile uint32_t dly_sys_iso_fact2lact            : 4 ;
        volatile uint32_t unused59                         : 4 ;
        volatile uint32_t dly_sys_cg_fact2lact             : 4 ;
        volatile uint32_t unused60                         : 4 ;
        volatile uint32_t dly_sys_rstn_fact2lact           : 4 ;
        volatile uint32_t unused61                         : 4 ;
    }f;
}aon2_ctrl_REG_dly_fact2lact;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_sys_pd_lact2fact             : 4 ;
        volatile uint32_t unused62                         : 4 ;
        volatile uint32_t dly_sys_iso_lact2fact            : 4 ;
        volatile uint32_t unused63                         : 4 ;
        volatile uint32_t dly_sys_cg_lact2fact             : 4 ;
        volatile uint32_t unused64                         : 4 ;
        volatile uint32_t dly_sys_rstn_lact2fact           : 4 ;
        volatile uint32_t unused65                         : 4 ;
    }f;
}aon2_ctrl_REG_dly_lact2fact;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dly_lle_pd_cact2fact             : 4 ;
        volatile uint32_t unused66                         : 4 ;
        volatile uint32_t dly_lle_iso_cact2fact            : 4 ;
        volatile uint32_t unused67                         : 4 ;
        volatile uint32_t dly_lle_rstn_cact2fact           : 4 ;
        volatile uint32_t unused68                         : 4 ;
        volatile uint32_t dly_isob_dig2rf_cact2fact        : 4 ;
        volatile uint32_t unused69                         : 4 ;
    }f;
}aon2_ctrl_REG_dly_cact2fact_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sw_cact2slp_p                    : 1 ;
        volatile uint32_t sw_cact2fact_p                   : 1 ;
        volatile uint32_t sw_fact2lact_p                   : 1 ;
        volatile uint32_t aon2_sleep_cnt_start_p           : 1 ;
        volatile uint32_t aon2_sleep_cnt_clr_p             : 1 ;
        volatile uint32_t sel_clkrf_rcn_lle_hw_clr         : 1 ;
        volatile uint32_t sel_clkrf_rcn_lle_sw             : 1 ;
        volatile uint32_t sel_clkrf_rcn_lle_mode           : 1 ;
        volatile uint32_t aon2_sleep_time_tgt_valid        : 1 ;
        volatile uint32_t fact2slp_fast                    : 1 ;
        volatile uint32_t sel_sys_llepd                    : 1 ;
        volatile uint32_t sel_clk_lle_sram1                : 1 ;
        volatile uint32_t sel_clk_lle_sram2                : 1 ;
        volatile uint32_t sel_clk_lle_sram3                : 1 ;
        volatile uint32_t io_latch                         : 1 ;
        volatile uint32_t io_ds                            : 1 ;
        volatile uint32_t aon2_wk_source_en_p              : 1 ;
        volatile uint32_t reg_rcmfd_cg_mode                : 1 ;
        volatile uint32_t reg_rcmfd_cg                     : 1 ;
        volatile uint32_t reg_slp_ctrl                     : 1 ;
        volatile uint32_t pll_en_reg                       : 1 ;
        volatile uint32_t unused70                         : 11;
    }f;
}aon2_ctrl_REG_aon2_sleep_ctrl_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aon2_sleep_cnt                   : 32;
    }f;
}aon2_ctrl_REG_aon2_sleep_cnt_rd;

typedef struct
{
volatile aon2_ctrl_REG_power_act_ctrl0                               power_act_ctrl0               ; //0x0
volatile aon2_ctrl_REG_power_act_ctrl1                               power_act_ctrl1               ; //0x4
volatile uint32_t                                                    null_reg0                     ; //0x8
volatile uint32_t                                                    null_reg1                     ; //0xC
volatile aon2_ctrl_REG_power_slp_ctrl0                               power_slp_ctrl0               ; //0x10
volatile aon2_ctrl_REG_power_slp_ctrl1                               power_slp_ctrl1               ; //0x14
volatile uint32_t                                                    null_reg2                     ; //0x18
volatile uint32_t                                                    null_reg3                     ; //0x1C
volatile aon2_ctrl_REG_dly_slp2act_ctrl0                             dly_slp2act_ctrl0             ; //0x20
volatile aon2_ctrl_REG_dly_slp2act_ctrl1                             dly_slp2act_ctrl1             ; //0x24
volatile uint32_t                                                    null_reg4                     ; //0x28
volatile uint32_t                                                    null_reg5                     ; //0x2C
volatile aon2_ctrl_REG_dly_act2slp_ctrl0                             dly_act2slp_ctrl0             ; //0x30
volatile aon2_ctrl_REG_dly_act2slp_ctrl1                             dly_act2slp_ctrl1             ; //0x34
volatile uint32_t                                                    null_reg6                     ; //0x38
volatile uint32_t                                                    null_reg7                     ; //0x3C
volatile aon2_ctrl_REG_pwr_ctrl0                                     pwr_ctrl0                     ; //0x40
volatile uint32_t                                                    null_reg8                     ; //0x44
volatile uint32_t                                                    null_reg9                     ; //0x48
volatile uint32_t                                                    null_reg10                    ; //0x4C
volatile uint32_t                                                    null_reg11                    ; //0x50
volatile uint32_t                                                    null_reg12                    ; //0x54
volatile uint32_t                                                    null_reg13                    ; //0x58
volatile uint32_t                                                    null_reg14                    ; //0x5C
volatile aon2_ctrl_REG_aon1_reg4                                     aon1_reg4                     ; //0x60
volatile aon2_ctrl_REG_aon1_reg6                                     aon1_reg6                     ; //0x64
volatile aon2_ctrl_REG_aon1_reg7                                     aon1_reg7                     ; //0x68
volatile aon2_ctrl_REG_aon1_reg8                                     aon1_reg8                     ; //0x6C
volatile aon2_ctrl_REG_aon1_reg9                                     aon1_reg9                     ; //0x70
volatile aon2_ctrl_REG_aon1_reg10                                    aon1_reg10                    ; //0x74
volatile uint32_t                                                    null_reg15                    ; //0x78
volatile uint32_t                                                    null_reg16                    ; //0x7C
volatile uint32_t                                                    null_reg17                    ; //0x80
volatile uint32_t                                                    null_reg18                    ; //0x84
volatile uint32_t                                                    null_reg19                    ; //0x88
volatile uint32_t                                                    null_reg20                    ; //0x8C
volatile uint32_t                                                    null_reg21                    ; //0x90
volatile uint32_t                                                    null_reg22                    ; //0x94
volatile uint32_t                                                    null_reg23                    ; //0x98
volatile uint32_t                                                    null_reg24                    ; //0x9C
volatile uint32_t                                                    null_reg25                    ; //0xA0
volatile uint32_t                                                    null_reg26                    ; //0xA4
volatile uint32_t                                                    null_reg27                    ; //0xA8
volatile uint32_t                                                    null_reg28                    ; //0xAC
volatile uint32_t                                                    null_reg29                    ; //0xB0
volatile uint32_t                                                    null_reg30                    ; //0xB4
volatile aon2_ctrl_REG_aon1_reg_rd11                                 aon1_reg_rd11                 ; //0xB8
volatile aon2_ctrl_REG_aon1_reg_rd12                                 aon1_reg_rd12                 ; //0xBC
volatile aon2_ctrl_REG_aon1_reg_rd13                                 aon1_reg_rd13                 ; //0xC0
volatile uint32_t                                                    null_reg31                    ; //0xC4
volatile uint32_t                                                    null_reg32                    ; //0xC8
volatile uint32_t                                                    null_reg33                    ; //0xCC
volatile uint32_t                                                    null_reg34                    ; //0xD0
volatile uint32_t                                                    null_reg35                    ; //0xD4
volatile uint32_t                                                    null_reg36                    ; //0xD8
volatile uint32_t                                                    null_reg37                    ; //0xDC
volatile uint32_t                                                    null_reg38                    ; //0xE0
volatile uint32_t                                                    null_reg39                    ; //0xE4
volatile uint32_t                                                    null_reg40                    ; //0xE8
volatile uint32_t                                                    null_reg41                    ; //0xEC
volatile uint32_t                                                    null_reg42                    ; //0xF0
volatile uint32_t                                                    null_reg43                    ; //0xF4
volatile uint32_t                                                    null_reg44                    ; //0xF8
volatile uint32_t                                                    null_reg45                    ; //0xFC
volatile aon2_ctrl_REG_pwr_ctrl_status0                              pwr_ctrl_status0              ; //0x100
volatile uint32_t                                                    null_reg46                    ; //0x104
volatile uint32_t                                                    null_reg47                    ; //0x108
volatile uint32_t                                                    null_reg48                    ; //0x10C
volatile aon2_ctrl_REG_aon_eco_reg0                                  aon_eco_reg0                  ; //0x110
volatile aon2_ctrl_REG_aon_eco_reg1                                  aon_eco_reg1                  ; //0x114
volatile uint32_t                                                    null_reg49                    ; //0x118
volatile uint32_t                                                    null_reg50                    ; //0x11C
volatile aon2_ctrl_REG_sys_wakeup_pol0                               sys_wakeup_pol0               ; //0x120
volatile aon2_ctrl_REG_sys_wakeup_pol1                               sys_wakeup_pol1               ; //0x124
volatile aon2_ctrl_REG_sys_wakeup_en0                                sys_wakeup_en0                ; //0x128
volatile aon2_ctrl_REG_sys_wakeup_en1                                sys_wakeup_en1                ; //0x12C
volatile aon2_ctrl_REG_sys_wakeup_mode0                              sys_wakeup_mode0              ; //0x130
volatile aon2_ctrl_REG_sys_wakeup_mode1                              sys_wakeup_mode1              ; //0x134
volatile aon2_ctrl_REG_sys_wakeup_mode2                              sys_wakeup_mode2              ; //0x138
volatile uint32_t                                                    null_reg51                    ; //0x13C
volatile aon2_ctrl_REG_aon2_wk_source                                aon2_wk_source                ; //0x140
volatile uint32_t                                                    null_reg52                    ; //0x144
volatile aon2_ctrl_REG_aon2_io_aon_i_low                             aon2_io_aon_i_low             ; //0x148
volatile aon2_ctrl_REG_aon2_io_aon_i_high                            aon2_io_aon_i_high            ; //0x14C
volatile uint32_t                                                    null_reg53                    ; //0x150
volatile uint32_t                                                    null_reg54                    ; //0x154
volatile uint32_t                                                    null_reg55                    ; //0x158
volatile uint32_t                                                    null_reg56                    ; //0x15C
volatile aon2_ctrl_REG_rf_ctrl0                                      rf_ctrl0                      ; //0x160
volatile aon2_ctrl_REG_rf_ctrl1                                      rf_ctrl1                      ; //0x164
volatile uint32_t                                                    null_reg57                    ; //0x168
volatile uint32_t                                                    null_reg58                    ; //0x16C
volatile uint32_t                                                    null_reg59                    ; //0x170
volatile aon2_ctrl_REG_usb11_phy_config                              usb11_phy_config              ; //0x174
volatile aon2_ctrl_REG_comparator_config                             comparator_config             ; //0x178
volatile aon2_ctrl_REG_rcmfd_config_0                                rcmfd_config_0                ; //0x17C
volatile aon2_ctrl_REG_rcmfd_config_1                                rcmfd_config_1                ; //0x180
volatile uint32_t                                                    null_reg60                    ; //0x184
volatile uint32_t                                                    null_reg61                    ; //0x188
volatile uint32_t                                                    null_reg62                    ; //0x18C
volatile aon2_ctrl_REG_dly_fact2lact                                 dly_fact2lact                 ; //0x190
volatile aon2_ctrl_REG_dly_lact2fact                                 dly_lact2fact                 ; //0x194
volatile aon2_ctrl_REG_dly_cact2fact_0                               dly_cact2fact_0               ; //0x198
volatile uint32_t                                                    null_reg63                    ; //0x19C
volatile uint32_t                                                    null_reg64                    ; //0x1A0
volatile uint32_t                                                    null_reg65                    ; //0x1A4
volatile aon2_ctrl_REG_aon2_sleep_ctrl_1                             aon2_sleep_ctrl_1             ; //0x1A8
volatile aon2_ctrl_REG_aon2_sleep_cnt_rd                             aon2_sleep_cnt_rd             ; //0x1AC
}aon2_ctrl_REG_H;

#endif //aon2_ctrl_REG__H
