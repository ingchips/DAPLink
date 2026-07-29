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
// File         : lle_top_reg.h
//                                                                             
// Author       : INGCHIPS                                                     
//                                                                             
// Organisation : INGCHIPS                                                     
//                                                                             
// Date         : 2022/1/22 
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
// 2022/1/22  INGCHIPS        v0.1      Create
//                                                                             
// ----------------------------------------------------------------------------
#ifndef LLE_TOP_REG__H
#define LLE_TOP_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ble_ena                          : 1 ;
        volatile uint32_t encrypt_ena                      : 1 ;
        volatile uint32_t rx_delay_match_ena               : 1 ;
        volatile uint32_t crc_ena                          : 1 ;
        volatile uint32_t white_ena                        : 1 ;
        volatile uint32_t fqhop_remap_ena                  : 1 ;
        volatile uint32_t wlcoext_ena                      : 1 ;
        volatile uint32_t wlcoext_blesi_ena                : 1 ;
        volatile uint32_t dbg_ena                          : 1 ;
        volatile uint32_t adv_err_rpt_ena                  : 1 ;
        volatile uint32_t sync_mode                        : 1 ;
        volatile uint32_t sync_thrd                        : 5 ;
        volatile uint32_t rx_fine_syncwind                 : 10;
        volatile uint32_t tx_winoff_sel                    : 1 ;
        volatile uint32_t rxpkt_cont_sel                   : 1 ;
        volatile uint32_t md_ena                           : 1 ;
        volatile uint32_t sn_ena                           : 1 ;
        volatile uint32_t nesn_ena                         : 1 ;
        volatile uint32_t crc_result_sel                   : 1 ;
    }f;
}LLE_TOP_REG_LLE_GLB_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lle_glb_soft_rst                 : 4 ;
        volatile uint32_t lle_main_soft_rst                : 1 ;
        volatile uint32_t lle_encrypt_soft_rst             : 1 ;
        volatile uint32_t lle_tbcu_soft_rst                : 1 ;
        volatile uint32_t lle_reg_soft_rst                 : 1 ;
        volatile uint32_t rsvd0                            : 8 ;
        volatile uint32_t scan_tmit                        : 1 ;
        volatile uint32_t advert_tmit                      : 1 ;
        volatile uint32_t conn_tmit                        : 1 ;
        volatile uint32_t rftest_tmit                      : 1 ;
        volatile uint32_t unused0                          : 12;
    }f;
}LLE_TOP_REG_LLE_SOFT_RST;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lle_pup_dly                      : 6 ;
        volatile uint32_t decoder_clk_sel                  : 2 ;
        volatile uint32_t rf_switch_dly_tx                 : 8 ;
        volatile uint32_t rf_switch_dly_rx                 : 8 ;
        volatile uint32_t rx_fine_syncwind_coded           : 4 ;
        volatile uint32_t tx_power_msb_pos                 : 4 ;
    }f;
}LLE_TOP_REG_LLE_POWER_UP_DLY;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cur_ecpt0_process                : 1 ;
        volatile uint32_t cur_ecpt1_process                : 1 ;
        volatile uint32_t unused1                          : 30;
    }f;
}LLE_TOP_REG_LLE_CUR_ECPT_PROCESS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cur_rxfstcont_loc                : 18;
        volatile uint32_t unused2                          : 13;
        volatile uint32_t cur_rxfstcont_loc_rdy            : 1 ;
    }f;
}LLE_TOP_REG_LLE_CUR_RXFSTCONT_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t last_used_rxcont_loc             : 18;
        volatile uint32_t unused3                          : 14;
    }f;
}LLE_TOP_REG_LLE_LAST_USED_RXCONT_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t addr_width_cfg                   : 6 ;
        volatile uint32_t sync_cfg                         : 1 ;
        volatile uint32_t encrypt_exist                    : 1 ;
        volatile uint32_t dbg_exist                        : 1 ;
        volatile uint32_t wlcoext_exist                    : 1 ;
        volatile uint32_t ahb2sm_exist                     : 1 ;
        volatile uint32_t samp_rate_cfg                    : 5 ;
        volatile uint32_t rf_spt_cfg                       : 3 ;
        volatile uint32_t clk_config                       : 6 ;
        volatile uint32_t unused4                          : 7 ;
    }f;
}LLE_TOP_REG_LLE_HWCFGA;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sm_base_addr_cfg                 : 32;
    }f;
}LLE_TOP_REG_LLE_HWCFGB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t btspec_ver                       : 8 ;
        volatile uint32_t release_ver                      : 8 ;
        volatile uint32_t cmp_name                         : 16;
    }f;
}LLE_TOP_REG_LLE_PROPERTY;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t adv_chid37_valid                 : 1 ;
        volatile uint32_t adv_chid38_valid                 : 1 ;
        volatile uint32_t adv_chid39_valid                 : 1 ;
        volatile uint32_t rsvd1                            : 1 ;
        volatile uint32_t rand_adv_ch_mode                 : 3 ;
        volatile uint32_t rand_adv_ch_enab                 : 1 ;
        volatile uint32_t unused5                          : 24;
    }f;
}LLE_TOP_REG_LLE_ADV_CHM;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t adv_pktintv                      : 16;
        volatile uint32_t unused6                          : 16;
    }f;
}LLE_TOP_REG_LLE_ADV_PKTINTV;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t backoff_inimax                   : 9 ;
        volatile uint32_t unused7                          : 7 ;
        volatile uint32_t backoff_cnt                      : 9 ;
        volatile uint32_t unused8                          : 7 ;
    }f;
}LLE_TOP_REG_LLE_ACT_SCAN_STAT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t whitelist_publoc                 : 18;
        volatile uint32_t unused9                          : 14;
    }f;
}LLE_TOP_REG_LLE_WITELIST_PUBLOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t whitelist_privloc                : 18;
        volatile uint32_t unused10                         : 14;
    }f;
}LLE_TOP_REG_LLE_WHITELIST_PRIVLOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t whitelist_pubdev_num             : 8 ;
        volatile uint32_t whitelist_privdev_num            : 8 ;
        volatile uint32_t unused11                         : 16;
    }f;
}LLE_TOP_REG_LLE_WHITELIST_DEVNUM;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t devaddrl                         : 32;
    }f;
}LLE_TOP_REG_LLE_PKTPROC_DEVADDRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t devaddrh                         : 16;
        volatile uint32_t priv_ind                         : 1 ;
        volatile uint32_t devaddr_loc                      : 1 ;
        volatile uint32_t unused12                         : 14;
    }f;
}LLE_TOP_REG_LLE_PKTPROC_DEVADDRH;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sm_addr_max                      : 18;
        volatile uint32_t unused13                         : 14;
    }f;
}LLE_TOP_REG_LLE_SMCTRL_SMADDR_MAX;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sm_addr_min                      : 18;
        volatile uint32_t unused14                         : 14;
    }f;
}LLE_TOP_REG_LLE_SMCTRL_SMADDR_MIN;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_trig_mode                    : 1 ;
        volatile uint32_t txpkt_int_msk                    : 1 ;
        volatile uint32_t rxpkt_int_msk                    : 1 ;
        volatile uint32_t encrypt_int_msk                  : 1 ;
        volatile uint32_t cte_smp_int_msk                  : 1 ;
        volatile uint32_t wkup_int_msk                     : 1 ;
        volatile uint32_t err_int_msk                      : 1 ;
        volatile uint32_t bts_int_msk                      : 1 ;
        volatile uint32_t event_int_msk                    : 1 ;
        volatile uint32_t long_time_int_msk                : 1 ;
        volatile uint32_t short_time_int_msk               : 1 ;
        volatile uint32_t sleep_fail_int_msk               : 1 ;
        volatile uint32_t rf_ini_int_msk                   : 1 ;
        volatile uint32_t us_time_int_msk                  : 1 ;
        volatile uint32_t extwkup_req_int_msk              : 1 ;
        volatile uint32_t unused15                         : 17;
    }f;
}LLE_TOP_REG_LLE_INT_MASK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rsvd2                            : 1 ;
        volatile uint32_t txpkt_int_stat                   : 1 ;
        volatile uint32_t rxpkt_int_stat                   : 1 ;
        volatile uint32_t encrypt_int_stat                 : 1 ;
        volatile uint32_t cte_smp_int_stat                 : 1 ;
        volatile uint32_t wkup_int_stat                    : 1 ;
        volatile uint32_t err_int_stat                     : 1 ;
        volatile uint32_t bts_int_stat                     : 1 ;
        volatile uint32_t event_int_stat                   : 1 ;
        volatile uint32_t long_time_int_stat               : 1 ;
        volatile uint32_t short_time_int_stat              : 1 ;
        volatile uint32_t sleep_fail_int_stat              : 1 ;
        volatile uint32_t rf_ini_int_stat                  : 1 ;
        volatile uint32_t us_time_int_stat                 : 1 ;
        volatile uint32_t extwkup_req_int_stat             : 1 ;
        volatile uint32_t unused16                         : 17;
    }f;
}LLE_TOP_REG_LLE_MASKINT_STAT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rsvd3                            : 1 ;
        volatile uint32_t txpkt_orig_int_stat              : 1 ;
        volatile uint32_t rxpkt_orig_int_stat              : 1 ;
        volatile uint32_t encrypt_orig_int_stat            : 1 ;
        volatile uint32_t cte_smp_orig_int_stat            : 1 ;
        volatile uint32_t wkup_orig_int_stat               : 1 ;
        volatile uint32_t err_orig_int_stat                : 1 ;
        volatile uint32_t bts_orig_int_stat                : 1 ;
        volatile uint32_t event_orig_int_stat              : 1 ;
        volatile uint32_t long_time_orig_int_stat          : 1 ;
        volatile uint32_t short_time_orig_int_stat         : 1 ;
        volatile uint32_t sleep_fail_orig_int_stat         : 1 ;
        volatile uint32_t rf_ini_orig_int_stat             : 1 ;
        volatile uint32_t us_time_orig_int_stat            : 1 ;
        volatile uint32_t extwkup_req_orig_int_stat        : 1 ;
        volatile uint32_t unused17                         : 17;
    }f;
}LLE_TOP_REG_LLE_ORIGINT_STAT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rsvd4                            : 1 ;
        volatile uint32_t txpkt_int_clr                    : 1 ;
        volatile uint32_t rxpkt_int_clr                    : 1 ;
        volatile uint32_t encrypt_int_clr                  : 1 ;
        volatile uint32_t cte_smp_int_clr                  : 1 ;
        volatile uint32_t wkup_int_clr                     : 1 ;
        volatile uint32_t err_int_clr                      : 1 ;
        volatile uint32_t bts_int_clr                      : 1 ;
        volatile uint32_t event_int_clr                    : 1 ;
        volatile uint32_t long_time_int_clr                : 1 ;
        volatile uint32_t short_time_int_clr               : 1 ;
        volatile uint32_t sleep_fail_int_clr               : 1 ;
        volatile uint32_t rf_ini_int_clr                   : 1 ;
        volatile uint32_t us_time_int_clr                  : 1 ;
        volatile uint32_t extwkup_req_int_clr              : 1 ;
        volatile uint32_t unused18                         : 17;
    }f;
}LLE_TOP_REG_LLE_INT_CLR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ecpt_evttype_err                 : 1 ;
        volatile uint32_t ecpt_txloc_err                   : 1 ;
        volatile uint32_t ecpt_rxloc_err                   : 1 ;
        volatile uint32_t ecpt_rxcont_err                  : 1 ;
        volatile uint32_t tbcu_sleep_time_err              : 1 ;
        volatile uint32_t txencrypt_err                    : 1 ;
        volatile uint32_t rxdecrypt_err                    : 1 ;
        volatile uint32_t pktproc_smacc_err                : 1 ;
        volatile uint32_t txcont_finishst_err              : 1 ;
        volatile uint32_t evt_overlap_err                  : 1 ;
        volatile uint32_t ifs_err                          : 1 ;
        volatile uint32_t ecpt_llchm_err                   : 1 ;
        volatile uint32_t whitelist_err                    : 1 ;
        volatile uint32_t rf_smacc_err                     : 1 ;
        volatile uint32_t sm_bound_err                     : 1 ;
        volatile uint32_t ral_underrun_err                 : 1 ;
        volatile uint32_t ral_err                          : 1 ;
        volatile uint32_t event_tmit_err                   : 1 ;
        volatile uint32_t cte_smp_busy                     : 1 ;
        volatile uint32_t adv_chnum_err                    : 1 ;
        volatile uint32_t evt_maxtime_zero_err             : 1 ;
        volatile uint32_t freqhop_fsm_err                  : 1 ;
        volatile uint32_t txcont_check_err                 : 1 ;
        volatile uint32_t rxcont_check_err                 : 1 ;
        volatile uint32_t rsvd5                            : 1 ;
        volatile uint32_t evt_start_time_err               : 1 ;
        volatile uint32_t rsvd6                            : 1 ;
        volatile uint32_t evt_conflict_err                 : 1 ;
        volatile uint32_t meshccm_dec_mic_err              : 1 ;
        volatile uint32_t unused19                         : 3 ;
    }f;
}LLE_TOP_REG_LLE_ERR_STAT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dbg_sel                          : 8 ;
        volatile uint32_t dbg_swap                         : 2 ;
        volatile uint32_t dbg_en                           : 1 ;
        volatile uint32_t dbg_en_clk                       : 1 ;
        volatile uint32_t dbg_sel_clk                      : 4 ;
        volatile uint32_t pa_rx_delay                      : 8 ;
        volatile uint32_t pa_tx_delay                      : 8 ;
    }f;
}LLE_TOP_REG_LLE_DBG_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rsvd7                            : 16;
        volatile uint32_t rsvd8                            : 16;
    }f;
}LLE_TOP_REG_LLE_DBG_STAT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sw_dbg                           : 32;
    }f;
}LLE_TOP_REG_LLE_SW_DBG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cc1clk_auto_clkgate_ena          : 1 ;
        volatile uint32_t cc1clk_gate_sw_ctrl              : 1 ;
        volatile uint32_t cc2clk_auto_clkgate_ena          : 1 ;
        volatile uint32_t cc2clk_gate_sw_ctrl              : 1 ;
        volatile uint32_t aesclk_auto_clkgate_ena          : 1 ;
        volatile uint32_t aesclk_gate_sw_ctrl              : 1 ;
        volatile uint32_t rfctrlclk_auto_clkgate_ena       : 1 ;
        volatile uint32_t rfctrlclk_gate_sw_ctrl           : 1 ;
        volatile uint32_t syncclk_auto_clkgate_ena         : 1 ;
        volatile uint32_t syncclk_gate_sw_ctrl             : 1 ;
        volatile uint32_t crcclk_auto_clkgate_ena          : 1 ;
        volatile uint32_t crcclk_gate_sw_ctrl              : 1 ;
        volatile uint32_t whitenclk_auto_clkgate_ena       : 1 ;
        volatile uint32_t whitenclk_gate_sw_ctrl           : 1 ;
        volatile uint32_t wlcoextclk_auto_clkgate_ena      : 1 ;
        volatile uint32_t wlcoextclk_gate_sw_ctrl          : 1 ;
        volatile uint32_t decoderclk_auto_clkgate_ena      : 1 ;
        volatile uint32_t decoderclk_gate_sw_ctrl          : 1 ;
        volatile uint32_t encoderclk_auto_clkgate_ena      : 1 ;
        volatile uint32_t encoderclk_gate_sw_ctrl          : 1 ;
        volatile uint32_t unused20                         : 12;
    }f;
}LLE_TOP_REG_LLE_CLK_GATE;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sm_rfctrl_maxaddr                : 18;
        volatile uint32_t unused21                         : 14;
    }f;
}LLE_TOP_REG_LLE_SM_RFCTRL_MAXADDR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_delay_match_tgt_1m            : 16;
        volatile uint32_t rx_delay_match_tgt_2m            : 16;
    }f;
}LLE_TOP_REG_LLE_RX_DELAY_MATCH_UNCODED;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_delay_match_tgt_s8_even       : 8 ;
        volatile uint32_t rx_delay_match_tgt_s8_odd        : 8 ;
        volatile uint32_t rx_delay_match_tgt_s2_even       : 8 ;
        volatile uint32_t rx_delay_match_tgt_s2_odd        : 8 ;
    }f;
}LLE_TOP_REG_LLE_RX_DELAY_MATCH_CODED;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rssi_gain0_1m                    : 8 ;
        volatile uint32_t rssi_gain1_1m                    : 8 ;
        volatile uint32_t rssi_gain2_1m                    : 8 ;
        volatile uint32_t rssi_gain3_1m                    : 8 ;
    }f;
}LLE_TOP_REG_LLE_RFRSSI_GAINA;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rssi_gain4_1m                    : 8 ;
        volatile uint32_t rssi_gain5_1m                    : 8 ;
        volatile uint32_t rssi_gain0_2m                    : 8 ;
        volatile uint32_t rssi_gain1_2m                    : 8 ;
    }f;
}LLE_TOP_REG_LLE_RFRSSI_GAINB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rssi_gain2_2m                    : 8 ;
        volatile uint32_t rssi_gain3_2m                    : 8 ;
        volatile uint32_t rssi_gain4_2m                    : 8 ;
        volatile uint32_t rssi_gain5_2m                    : 8 ;
    }f;
}LLE_TOP_REG_LLE_RFRSSI_GAI;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ral_current_loc                  : 18;
        volatile uint32_t unused22                         : 14;
    }f;
}LLE_TOP_REG_LLE_RAL_CURRENT_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ral_lrng_value                   : 22;
        volatile uint32_t unused23                         : 10;
    }f;
}LLE_TOP_REG_LLE_RAL_LOCAL_RNG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ral_prng_value                   : 22;
        volatile uint32_t unused24                         : 10;
    }f;
}LLE_TOP_REG_LLE_RAL_PEER_RNG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_start_0                      : 1 ;
        volatile uint32_t unused25                         : 31;
    }f;
}LLE_TOP_REG_LLE_ECPT_START_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_carry_base_time_l_0          : 32;
    }f;
}LLE_TOP_REG_LLE_ECPT_CARRY_TIME_L_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_carry_base_time_h_0          : 8 ;
        volatile uint32_t unused26                         : 24;
    }f;
}LLE_TOP_REG_LLE_ECPT_CARRY_TIME_H_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_ecpt_loc_0                   : 18;
        volatile uint32_t unused27                         : 14;
    }f;
}LLE_TOP_REG_LLE_ECPT_LOC_0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_start_1                      : 1 ;
        volatile uint32_t unused28                         : 31;
    }f;
}LLE_TOP_REG_LLE_ECPT_START_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_carry_base_time_l_1          : 32;
    }f;
}LLE_TOP_REG_LLE_ECPT_CARRY_TIME_L_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_carry_base_time_h_1          : 8 ;
        volatile uint32_t unused29                         : 24;
    }f;
}LLE_TOP_REG_LLE_ECPT_CARRY_TIME_H_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t evt_ecpt_loc_1                   : 18;
        volatile uint32_t unused30                         : 14;
    }f;
}LLE_TOP_REG_LLE_ECPT_LOC_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_sw_pattern_default           : 8 ;
        volatile uint32_t cte_en                           : 1 ;
        volatile uint32_t cte_iq_length                    : 4 ;
        volatile uint32_t cte_iq_swap                      : 1 ;
        volatile uint32_t cte_iq_format                    : 1 ;
        volatile uint32_t cte_iq_sel                       : 5 ;
        volatile uint32_t cte_gp_ss_length                 : 4 ;
        volatile uint32_t cte_fifo_wr_clr                  : 1 ;
        volatile uint32_t cte_fifo_rd_clr                  : 1 ;
        volatile uint32_t cte_dbg_ctrl                     : 2 ;
        volatile uint32_t cte_fifo_wr_full                 : 1 ;
        volatile uint32_t cte_fifo_wr_empty                : 1 ;
        volatile uint32_t cte_fifo_rd_full                 : 1 ;
        volatile uint32_t cte_fifo_rd_empty                : 1 ;
    }f;
}LLE_TOP_REG_CTE_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_aoa_1ss_rf_adj_us_1m         : 6 ;
        volatile uint32_t cte_aoa_1ss_rf_adj_cycle_1m      : 6 ;
        volatile uint32_t cte_aoa_1ss_rf_adj_us_2m         : 6 ;
        volatile uint32_t cte_aoa_1ss_rf_adj_cycle_2m      : 6 ;
        volatile uint32_t unused31                         : 8 ;
    }f;
}LLE_TOP_REG_CTE_AOA_1SS_RF_ADJ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_aoa_2ss_rf_adj_us_1m         : 6 ;
        volatile uint32_t cte_aoa_2ss_rf_adj_cycle_1m      : 6 ;
        volatile uint32_t cte_aoa_2ss_rf_adj_us_2m         : 6 ;
        volatile uint32_t cte_aoa_2ss_rf_adj_cycle_2m      : 6 ;
        volatile uint32_t unused32                         : 8 ;
    }f;
}LLE_TOP_REG_CTE_AOA_2SS_RF_ADJ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_aod_1ss_rf_adj_us_1m         : 6 ;
        volatile uint32_t cte_aod_1ss_rf_adj_cycle_1m      : 6 ;
        volatile uint32_t cte_aod_1ss_rf_adj_us_2m         : 6 ;
        volatile uint32_t cte_aod_1ss_rf_adj_cycle_2m      : 6 ;
        volatile uint32_t unused33                         : 8 ;
    }f;
}LLE_TOP_REG_CTE_AOD_1SS_RF_ADJ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_aod_2ss_rf_adj_us_1m         : 6 ;
        volatile uint32_t cte_aod_2ss_rf_adj_cycle_1m      : 6 ;
        volatile uint32_t cte_aod_2ss_rf_adj_us_2m         : 6 ;
        volatile uint32_t cte_aod_2ss_rf_adj_cycle_2m      : 6 ;
        volatile uint32_t unused34                         : 8 ;
    }f;
}LLE_TOP_REG_CTE_AOD_2SS_RF_ADJ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_smp_adj_us_1m                : 6 ;
        volatile uint32_t cte_smp_adj_cycle_1m             : 6 ;
        volatile uint32_t cte_smp_adj_us_2m                : 6 ;
        volatile uint32_t cte_smp_adj_cycle_2m             : 6 ;
        volatile uint32_t unused35                         : 8 ;
    }f;
}LLE_TOP_REG_CTE_SMP_ADJ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lp_mode                          : 3 ;
        volatile uint32_t rsvd9                            : 5 ;
        volatile uint32_t unused36                         : 24;
    }f;
}LLE_TOP_REG_LLE_LPCU_CFGA;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lp_start                         : 1 ;
        volatile uint32_t wkup_mode                        : 2 ;
        volatile uint32_t lp_stat                          : 4 ;
        volatile uint32_t sleep_n_stat                     : 1 ;
        volatile uint32_t wkup_source                      : 3 ;
        volatile uint32_t sleep_cnt_clr                    : 1 ;
        volatile uint32_t extwkup_en                       : 1 ;
        volatile uint32_t unused37                         : 19;
    }f;
}LLE_TOP_REG_LLE_LPCU_CFGB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sleep_time                       : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_SLEEP_TIME;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cur_sleep_cnt                    : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_CUR_SLEEP_CNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sleep_fra_inc                    : 16;
        volatile uint32_t sleep_int_inc                    : 6 ;
        volatile uint32_t unused38                         : 10;
    }f;
}LLE_TOP_REG_LLE_TBCU_SLEEP_INC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t calib_ena                        : 1 ;
        volatile uint32_t calib_stat                       : 1 ;
        volatile uint32_t calib_result                     : 22;
        volatile uint32_t calib_mode                       : 4 ;
        volatile uint32_t unused39                         : 4 ;
    }f;
}LLE_TOP_REG_LLE_TBCU_CALIB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rsvd10                           : 8 ;
        volatile uint32_t rsvd11                           : 8 ;
        volatile uint32_t pow_stbtime                      : 12;
        volatile uint32_t unused40                         : 4 ;
    }f;
}LLE_TOP_REG_LLE_TBCU_RF_STBTIME;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t priv_cte_base_time               : 10;
        volatile uint32_t rsvd12                           : 6 ;
        volatile uint32_t rsvd13                           : 16;
    }f;
}LLE_TOP_REG_LLE_TBCU_OSC_STBTIME;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t base_ts_cnt_lsb                  : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_BASETSCNT_LSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t base_ts_cnt_hsb                  : 8 ;
        volatile uint32_t unused41                         : 24;
    }f;
}LLE_TOP_REG_LLE_TBCU_BASETSCNT_HSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sub_ts_cnt                       : 10;
        volatile uint32_t unused42                         : 22;
    }f;
}LLE_TOP_REG_LLE_TBCU_SUBTSCNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t bts_lock_cnt_lsb                 : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_LOCK_BASETSCNT_LSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t bts_lock_cnt_hsb                 : 8 ;
        volatile uint32_t unused43                         : 24;
    }f;
}LLE_TOP_REG_LLE_TBCU_LOCK_BASETSCNT_HSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sts_lock_cnt                     : 10;
        volatile uint32_t unused44                         : 22;
    }f;
}LLE_TOP_REG_LLE_TBCU_LOCK_SUBTSCNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tbu_cnt_lock                     : 1 ;
        volatile uint32_t tbcu_slp_cnt_lock                : 1 ;
        volatile uint32_t tbcu_slp_cnt_stable              : 1 ;
        volatile uint32_t unused45                         : 29;
    }f;
}LLE_TOP_REG_LLE_TBCU_CNT_LOCK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t long_time                        : 16;
        volatile uint32_t unused46                         : 16;
    }f;
}LLE_TOP_REG_LLE_TBCU_LONG_TIME;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t short_time                       : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_SHORT_TIME;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t us_base_time_lsb                 : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_US_TIME_L;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t us_base_time_hsb                 : 8 ;
        volatile uint32_t us_sub_time                      : 10;
        volatile uint32_t unused47                         : 14;
    }f;
}LLE_TOP_REG_LLE_TBCU_US_TIME_H;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t lp_ext_wakeup_dly_cnt            : 16;
        volatile uint32_t lp_ext_wakeup_dly_ena            : 1 ;
        volatile uint32_t unused48                         : 15;
    }f;
}LLE_TOP_REG_LLE_EXT_WAKEUP_DLY_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank0_en                    : 18;
        volatile uint32_t reserved                         : 2 ;
        volatile uint32_t gpio_bank0_ctrl                  : 2 ;
        volatile uint32_t unused49                         : 10;
    }f;
}LLE_TOP_REG_LLE_GPIO0_EN;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank0_bts_lock_cnt_lsb      : 32;
    }f;
}LLE_TOP_REG_LLE_GPIO0_LOCK_BASETSCNT_LSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank0_bts_lock_cnt_hsb      : 8 ;
        volatile uint32_t unused50                         : 24;
    }f;
}LLE_TOP_REG_LLE_GOIO0_LOCK_BASETSCNT_HSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank0_sts_lock_cnt          : 10;
        volatile uint32_t unused51                         : 22;
    }f;
}LLE_TOP_REG_LLE_GPIO0_LOCK_SUBTSCNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank1_en                    : 18;
        volatile uint32_t reserved                         : 2 ;
        volatile uint32_t gpio_bank1_ctrl                  : 2 ;
        volatile uint32_t unused52                         : 10;
    }f;
}LLE_TOP_REG_LLE_GPIO1_EN;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank1_bts_lock_cnt_lsb      : 32;
    }f;
}LLE_TOP_REG_LLE_GPIO1_LOCK_BASETSCNT_LSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank1_bts_lock_cnt_hsb      : 8 ;
        volatile uint32_t unused53                         : 24;
    }f;
}LLE_TOP_REG_LLE_GOIO1_LOCK_BASETSCNT_HSB;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_bank1_sts_lock_cnt          : 10;
        volatile uint32_t unused54                         : 22;
    }f;
}LLE_TOP_REG_LLE_GPIO1_LOCK_SUBTSCNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cur_sleep_lock_cnt               : 32;
    }f;
}LLE_TOP_REG_LLE_TBCU_CUR_SLEEP_LOCK_CNT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_lp_mode                       : 1 ;
        volatile uint32_t sw_init_rf_start                 : 1 ;
        volatile uint32_t rxtxon_delay                     : 3 ;
        volatile uint32_t sw_init_rf_finished              : 1 ;
        volatile uint32_t exprb_insert_ena                 : 1 ;
        volatile uint32_t exprb_insert_mode                : 2 ;
        volatile uint32_t unused55                         : 7 ;
        volatile uint32_t rf_rssi_addr                     : 8 ;
        volatile uint32_t rf_tx_fine_lat                   : 8 ;
    }f;
}LLE_TOP_REG_LLE_RF_CFG0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_rate                         : 3 ;
        volatile uint32_t spi_cpol                         : 1 ;
        volatile uint32_t spi_cpha                         : 1 ;
        volatile uint32_t unused56                         : 27;
    }f;
}LLE_TOP_REG_LLE_RF_CFG1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_reg_addr                      : 8 ;
        volatile uint32_t rf_reg_rd_en                     : 1 ;
        volatile uint32_t rf_reg_wr_en                     : 1 ;
        volatile uint32_t unused57                         : 22;
    }f;
}LLE_TOP_REG_LLE_RF_CFG2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_reg_rdata                     : 16;
        volatile uint32_t rf_reg_wdata                     : 16;
    }f;
}LLE_TOP_REG_LLE_RF_CFG3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_cfg4                          : 32;
    }f;
}LLE_TOP_REG_LLE_RF_CFG4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_byte_len                     : 7 ;
        volatile uint32_t rf_cfg5                          : 25;
    }f;
}LLE_TOP_REG_LLE_RF_CFG5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_spi_buf                       : 32;
    }f;
}LLE_TOP_REG_LLE_RF_SPI_BUF;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rftx_pup_dly                     : 8 ;
        volatile uint32_t rftx_pdn_dly                     : 4 ;
        volatile uint32_t rfrx_pup_dly                     : 8 ;
        volatile uint32_t rfrx_pdn_dly                     : 4 ;
        volatile uint32_t rsvd14                           : 8 ;
    }f;
}LLE_TOP_REG_LLE_RF_POWDLY;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tx_pld_sel                       : 1 ;
        volatile uint32_t tx_len_sel                       : 1 ;
        volatile uint32_t prbs_type                        : 1 ;
        volatile uint32_t rsvd15                           : 1 ;
        volatile uint32_t tx_endless                       : 1 ;
        volatile uint32_t rx_endless                       : 1 ;
        volatile uint32_t rsvd16                           : 2 ;
        volatile uint32_t tx_len                           : 8 ;
        volatile uint32_t unused58                         : 16;
    }f;
}LLE_TOP_REG_LLE_RF_TEST_CFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ifs_time_check                   : 8 ;
        volatile uint32_t unused59                         : 24;
    }f;
}LLE_TOP_REG_LLE_IFS_TIME_CHECK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_spi_pos                       : 4 ;
        volatile uint32_t tx_spi_pos                       : 4 ;
        volatile uint32_t tx_power_pos                     : 4 ;
        volatile uint32_t unused60                         : 20;
    }f;
}LLE_TOP_REG_LLE_RF_CFG6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aes_start                        : 1 ;
        volatile uint32_t meshccm_encrypt_start            : 1 ;
        volatile uint32_t meshccm_decrypt_start            : 1 ;
        volatile uint32_t aesccm_busy                      : 1 ;
        volatile uint32_t aesccm_int_source                : 2 ;
        volatile uint32_t unused61                         : 26;
    }f;
}LLE_TOP_REG_LLE_AES_CFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aeskey0                          : 32;
    }f;
}LLE_TOP_REG_LLE_AES_KEY0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aeskey1                          : 32;
    }f;
}LLE_TOP_REG_LLE_AES_KEY1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aeskey2                          : 32;
    }f;
}LLE_TOP_REG_LLE_AES_KEY2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aeskey3                          : 32;
    }f;
}LLE_TOP_REG_LLE_AES_KEY3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aes_loc                          : 18;
        volatile uint32_t unused62                         : 14;
    }f;
}LLE_TOP_REG_LLE_AES_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tx_mic                           : 32;
    }f;
}LLE_TOP_REG_LLE_CCM_TXMIC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_mic                           : 32;
    }f;
}LLE_TOP_REG_LLE_CCM_RXMIC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_mode                 : 1 ;
        volatile uint32_t meshccm_mic_mode                 : 1 ;
        volatile uint32_t meshccm_tx_lth                   : 9 ;
        volatile uint32_t meshccm_rx_lth                   : 9 ;
        volatile uint32_t unused63                         : 12;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_CFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_ptr                      : 18;
        volatile uint32_t unused64                         : 14;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_key0                     : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_KEY0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_key1                     : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_KEY1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_key2                     : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_KEY2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_key3                     : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_KEY3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data0                : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_AUT_DATA0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data1                : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_AUT_DATA1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data2                : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_AUT_DATA2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data3                : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_AUT_DATA3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce0                   : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_NONCE0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce1                   : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_NONCE1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce2                   : 32;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_NONCE2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce3                   : 8 ;
        volatile uint32_t unused65                         : 24;
    }f;
}LLE_TOP_REG_LLE_MESHCCM_NONCE3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_fe_ctrim0                     : 32;
    }f;
}LLE_TOP_REG_LLE_RF_FE_CTRIM0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_fe_ctrim1                     : 32;
    }f;
}LLE_TOP_REG_LLE_RF_FE_CTRIM1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_fe_ctrim2                     : 32;
    }f;
}LLE_TOP_REG_LLE_RF_FE_CTRIM2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_fe_ctrim3                     : 32;
    }f;
}LLE_TOP_REG_LLE_RF_FE_CTRIM3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rf_fe_ctrim4                     : 32;
    }f;
}LLE_TOP_REG_LLE_RF_FE_CTRIM4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cte_pattern                      : 1 ;
        volatile uint32_t bypass_rx_error                  : 1 ;
        volatile uint32_t private_len_en                   : 1 ;
        volatile uint32_t rf_cw_test                       : 3 ;
        volatile uint32_t whiten_init_sel                  : 1 ;
        volatile uint32_t whiten_init_value                : 7 ;
        volatile uint32_t cte_smp_sm_ctrl                  : 1 ;
        volatile uint32_t cte_smp_keep_dump                : 1 ;
        volatile uint32_t cte_smp_deci_rate                : 5 ;
        volatile uint32_t cte_smp_clk_mode                 : 1 ;
        volatile uint32_t cte_smp_clk_en                   : 1 ;
        volatile uint32_t private_adv_type_en              : 1 ;
        volatile uint32_t private_adv_type                 : 4 ;
        volatile uint32_t unused66                         : 4 ;
    }f;
}LLE_TOP_REG_LLE_PRIVATE_CTRL;

typedef struct
{
volatile LLE_TOP_REG_LLE_GLB_CTRL                                    LLE_GLB_CTRL                  ; //0x0
volatile LLE_TOP_REG_LLE_SOFT_RST                                    LLE_SOFT_RST                  ; //0x4
volatile LLE_TOP_REG_LLE_POWER_UP_DLY                                LLE_POWER_UP_DLY              ; //0x8
volatile uint32_t                                                    null_reg0                     ; //0xC
volatile LLE_TOP_REG_LLE_CUR_ECPT_PROCESS                            LLE_CUR_ECPT_PROCESS          ; //0x10
volatile LLE_TOP_REG_LLE_CUR_RXFSTCONT_LOC                           LLE_CUR_RXFSTCONT_LOC         ; //0x14
volatile LLE_TOP_REG_LLE_LAST_USED_RXCONT_LOC                        LLE_LAST_USED_RXCONT_LOC      ; //0x18
volatile LLE_TOP_REG_LLE_HWCFGA                                      LLE_HWCFGA                    ; //0x1C
volatile LLE_TOP_REG_LLE_HWCFGB                                      LLE_HWCFGB                    ; //0x20
volatile LLE_TOP_REG_LLE_PROPERTY                                    LLE_PROPERTY                  ; //0x24
volatile LLE_TOP_REG_LLE_ADV_CHM                                     LLE_ADV_CHM                   ; //0x28
volatile LLE_TOP_REG_LLE_ADV_PKTINTV                                 LLE_ADV_PKTINTV               ; //0x2C
volatile LLE_TOP_REG_LLE_ACT_SCAN_STAT                               LLE_ACT_SCAN_STAT             ; //0x30
volatile LLE_TOP_REG_LLE_WITELIST_PUBLOC                             LLE_WITELIST_PUBLOC           ; //0x34
volatile LLE_TOP_REG_LLE_WHITELIST_PRIVLOC                           LLE_WHITELIST_PRIVLOC         ; //0x38
volatile LLE_TOP_REG_LLE_WHITELIST_DEVNUM                            LLE_WHITELIST_DEVNUM          ; //0x3C
volatile LLE_TOP_REG_LLE_PKTPROC_DEVADDRL                            LLE_PKTPROC_DEVADDRL          ; //0x40
volatile LLE_TOP_REG_LLE_PKTPROC_DEVADDRH                            LLE_PKTPROC_DEVADDRH          ; //0x44
volatile LLE_TOP_REG_LLE_SMCTRL_SMADDR_MAX                           LLE_SMCTRL_SMADDR_MAX         ; //0x48
volatile LLE_TOP_REG_LLE_SMCTRL_SMADDR_MIN                           LLE_SMCTRL_SMADDR_MIN         ; //0x4C
volatile LLE_TOP_REG_LLE_INT_MASK                                    LLE_INT_MASK                  ; //0x50
volatile LLE_TOP_REG_LLE_MASKINT_STAT                                LLE_MASKINT_STAT              ; //0x54
volatile LLE_TOP_REG_LLE_ORIGINT_STAT                                LLE_ORIGINT_STAT              ; //0x58
volatile LLE_TOP_REG_LLE_INT_CLR                                     LLE_INT_CLR                   ; //0x5C
volatile LLE_TOP_REG_LLE_ERR_STAT                                    LLE_ERR_STAT                  ; //0x60
volatile LLE_TOP_REG_LLE_DBG_CTRL                                    LLE_DBG_CTRL                  ; //0x64
volatile LLE_TOP_REG_LLE_DBG_STAT                                    LLE_DBG_STAT                  ; //0x68
volatile LLE_TOP_REG_LLE_SW_DBG                                      LLE_SW_DBG                    ; //0x6C
volatile LLE_TOP_REG_LLE_CLK_GATE                                    LLE_CLK_GATE                  ; //0x70
volatile LLE_TOP_REG_LLE_SM_RFCTRL_MAXADDR                           LLE_SM_RFCTRL_MAXADDR         ; //0x74
volatile LLE_TOP_REG_LLE_RX_DELAY_MATCH_UNCODED                      LLE_RX_DELAY_MATCH_UNCODED    ; //0x78
volatile LLE_TOP_REG_LLE_RX_DELAY_MATCH_CODED                        LLE_RX_DELAY_MATCH_CODED      ; //0x7C
volatile LLE_TOP_REG_LLE_RFRSSI_GAINA                                LLE_RFRSSI_GAINA              ; //0x80
volatile LLE_TOP_REG_LLE_RFRSSI_GAINB                                LLE_RFRSSI_GAINB              ; //0x84
volatile uint32_t                                                    null_reg1                     ; //0x88
volatile LLE_TOP_REG_LLE_RFRSSI_GAI                                  LLE_RFRSSI_GAI                ; //0x8C
volatile uint32_t                                                    null_reg2                     ; //0x90
volatile uint32_t                                                    null_reg3                     ; //0x94
volatile LLE_TOP_REG_LLE_RAL_CURRENT_LOC                             LLE_RAL_CURRENT_LOC           ; //0x98
volatile LLE_TOP_REG_LLE_RAL_LOCAL_RNG                               LLE_RAL_LOCAL_RNG             ; //0x9C
volatile LLE_TOP_REG_LLE_RAL_PEER_RNG                                LLE_RAL_PEER_RNG              ; //0xA0
volatile uint32_t                                                    null_reg4                     ; //0xA4
volatile uint32_t                                                    null_reg5                     ; //0xA8
volatile uint32_t                                                    null_reg6                     ; //0xAC
volatile uint32_t                                                    null_reg7                     ; //0xB0
volatile uint32_t                                                    null_reg8                     ; //0xB4
volatile uint32_t                                                    null_reg9                     ; //0xB8
volatile uint32_t                                                    null_reg10                    ; //0xBC
volatile uint32_t                                                    null_reg11                    ; //0xC0
volatile uint32_t                                                    null_reg12                    ; //0xC4
volatile uint32_t                                                    null_reg13                    ; //0xC8
volatile uint32_t                                                    null_reg14                    ; //0xCC
volatile uint32_t                                                    null_reg15                    ; //0xD0
volatile uint32_t                                                    null_reg16                    ; //0xD4
volatile uint32_t                                                    null_reg17                    ; //0xD8
volatile uint32_t                                                    null_reg18                    ; //0xDC
volatile uint32_t                                                    null_reg19                    ; //0xE0
volatile uint32_t                                                    null_reg20                    ; //0xE4
volatile uint32_t                                                    null_reg21                    ; //0xE8
volatile uint32_t                                                    null_reg22                    ; //0xEC
volatile uint32_t                                                    null_reg23                    ; //0xF0
volatile uint32_t                                                    null_reg24                    ; //0xF4
volatile uint32_t                                                    null_reg25                    ; //0xF8
volatile uint32_t                                                    null_reg26                    ; //0xFC
volatile LLE_TOP_REG_LLE_ECPT_START_0                                LLE_ECPT_START_0              ; //0x100
volatile LLE_TOP_REG_LLE_ECPT_CARRY_TIME_L_0                         LLE_ECPT_CARRY_TIME_L_0       ; //0x104
volatile LLE_TOP_REG_LLE_ECPT_CARRY_TIME_H_0                         LLE_ECPT_CARRY_TIME_H_0       ; //0x108
volatile LLE_TOP_REG_LLE_ECPT_LOC_0                                  LLE_ECPT_LOC_0                ; //0x10C
volatile LLE_TOP_REG_LLE_ECPT_START_1                                LLE_ECPT_START_1              ; //0x110
volatile LLE_TOP_REG_LLE_ECPT_CARRY_TIME_L_1                         LLE_ECPT_CARRY_TIME_L_1       ; //0x114
volatile LLE_TOP_REG_LLE_ECPT_CARRY_TIME_H_1                         LLE_ECPT_CARRY_TIME_H_1       ; //0x118
volatile LLE_TOP_REG_LLE_ECPT_LOC_1                                  LLE_ECPT_LOC_1                ; //0x11C
volatile uint32_t                                                    null_reg27                    ; //0x120
volatile uint32_t                                                    null_reg28                    ; //0x124
volatile uint32_t                                                    null_reg29                    ; //0x128
volatile uint32_t                                                    null_reg30                    ; //0x12C
volatile uint32_t                                                    null_reg31                    ; //0x130
volatile uint32_t                                                    null_reg32                    ; //0x134
volatile uint32_t                                                    null_reg33                    ; //0x138
volatile uint32_t                                                    null_reg34                    ; //0x13C
volatile uint32_t                                                    null_reg35                    ; //0x140
volatile uint32_t                                                    null_reg36                    ; //0x144
volatile uint32_t                                                    null_reg37                    ; //0x148
volatile uint32_t                                                    null_reg38                    ; //0x14C
volatile uint32_t                                                    null_reg39                    ; //0x150
volatile uint32_t                                                    null_reg40                    ; //0x154
volatile uint32_t                                                    null_reg41                    ; //0x158
volatile uint32_t                                                    null_reg42                    ; //0x15C
volatile uint32_t                                                    null_reg43                    ; //0x160
volatile uint32_t                                                    null_reg44                    ; //0x164
volatile uint32_t                                                    null_reg45                    ; //0x168
volatile uint32_t                                                    null_reg46                    ; //0x16C
volatile uint32_t                                                    null_reg47                    ; //0x170
volatile uint32_t                                                    null_reg48                    ; //0x174
volatile uint32_t                                                    null_reg49                    ; //0x178
volatile uint32_t                                                    null_reg50                    ; //0x17C
volatile uint32_t                                                    null_reg51                    ; //0x180
volatile uint32_t                                                    null_reg52                    ; //0x184
volatile uint32_t                                                    null_reg53                    ; //0x188
volatile uint32_t                                                    null_reg54                    ; //0x18C
volatile uint32_t                                                    null_reg55                    ; //0x190
volatile uint32_t                                                    null_reg56                    ; //0x194
volatile uint32_t                                                    null_reg57                    ; //0x198
volatile uint32_t                                                    null_reg58                    ; //0x19C
volatile uint32_t                                                    null_reg59                    ; //0x1A0
volatile uint32_t                                                    null_reg60                    ; //0x1A4
volatile uint32_t                                                    null_reg61                    ; //0x1A8
volatile uint32_t                                                    null_reg62                    ; //0x1AC
volatile uint32_t                                                    null_reg63                    ; //0x1B0
volatile uint32_t                                                    null_reg64                    ; //0x1B4
volatile uint32_t                                                    null_reg65                    ; //0x1B8
volatile uint32_t                                                    null_reg66                    ; //0x1BC
volatile uint32_t                                                    null_reg67                    ; //0x1C0
volatile uint32_t                                                    null_reg68                    ; //0x1C4
volatile uint32_t                                                    null_reg69                    ; //0x1C8
volatile uint32_t                                                    null_reg70                    ; //0x1CC
volatile uint32_t                                                    null_reg71                    ; //0x1D0
volatile uint32_t                                                    null_reg72                    ; //0x1D4
volatile uint32_t                                                    null_reg73                    ; //0x1D8
volatile uint32_t                                                    null_reg74                    ; //0x1DC
volatile uint32_t                                                    null_reg75                    ; //0x1E0
volatile uint32_t                                                    null_reg76                    ; //0x1E4
volatile uint32_t                                                    null_reg77                    ; //0x1E8
volatile uint32_t                                                    null_reg78                    ; //0x1EC
volatile uint32_t                                                    null_reg79                    ; //0x1F0
volatile uint32_t                                                    null_reg80                    ; //0x1F4
volatile uint32_t                                                    null_reg81                    ; //0x1F8
volatile uint32_t                                                    null_reg82                    ; //0x1FC
volatile LLE_TOP_REG_CTE_CTRL                                        CTE_CTRL                      ; //0x200
volatile LLE_TOP_REG_CTE_AOA_1SS_RF_ADJ                              CTE_AOA_1SS_RF_ADJ            ; //0x204
volatile LLE_TOP_REG_CTE_AOA_2SS_RF_ADJ                              CTE_AOA_2SS_RF_ADJ            ; //0x208
volatile LLE_TOP_REG_CTE_AOD_1SS_RF_ADJ                              CTE_AOD_1SS_RF_ADJ            ; //0x20C
volatile LLE_TOP_REG_CTE_AOD_2SS_RF_ADJ                              CTE_AOD_2SS_RF_ADJ            ; //0x210
volatile LLE_TOP_REG_CTE_SMP_ADJ                                     CTE_SMP_ADJ                   ; //0x214
volatile uint32_t                                                    null_reg83                    ; //0x218
volatile uint32_t                                                    null_reg84                    ; //0x21C
volatile uint32_t                                                    null_reg85                    ; //0x220
volatile uint32_t                                                    null_reg86                    ; //0x224
volatile uint32_t                                                    null_reg87                    ; //0x228
volatile uint32_t                                                    null_reg88                    ; //0x22C
volatile uint32_t                                                    null_reg89                    ; //0x230
volatile uint32_t                                                    null_reg90                    ; //0x234
volatile uint32_t                                                    null_reg91                    ; //0x238
volatile uint32_t                                                    null_reg92                    ; //0x23C
volatile uint32_t                                                    null_reg93                    ; //0x240
volatile uint32_t                                                    null_reg94                    ; //0x244
volatile uint32_t                                                    null_reg95                    ; //0x248
volatile uint32_t                                                    null_reg96                    ; //0x24C
volatile uint32_t                                                    null_reg97                    ; //0x250
volatile uint32_t                                                    null_reg98                    ; //0x254
volatile uint32_t                                                    null_reg99                    ; //0x258
volatile uint32_t                                                    null_reg100                   ; //0x25C
volatile uint32_t                                                    null_reg101                   ; //0x260
volatile uint32_t                                                    null_reg102                   ; //0x264
volatile uint32_t                                                    null_reg103                   ; //0x268
volatile uint32_t                                                    null_reg104                   ; //0x26C
volatile uint32_t                                                    null_reg105                   ; //0x270
volatile uint32_t                                                    null_reg106                   ; //0x274
volatile uint32_t                                                    null_reg107                   ; //0x278
volatile uint32_t                                                    null_reg108                   ; //0x27C
volatile uint32_t                                                    null_reg109                   ; //0x280
volatile uint32_t                                                    null_reg110                   ; //0x284
volatile uint32_t                                                    null_reg111                   ; //0x288
volatile uint32_t                                                    null_reg112                   ; //0x28C
volatile uint32_t                                                    null_reg113                   ; //0x290
volatile uint32_t                                                    null_reg114                   ; //0x294
volatile uint32_t                                                    null_reg115                   ; //0x298
volatile uint32_t                                                    null_reg116                   ; //0x29C
volatile uint32_t                                                    null_reg117                   ; //0x2A0
volatile uint32_t                                                    null_reg118                   ; //0x2A4
volatile uint32_t                                                    null_reg119                   ; //0x2A8
volatile uint32_t                                                    null_reg120                   ; //0x2AC
volatile uint32_t                                                    null_reg121                   ; //0x2B0
volatile uint32_t                                                    null_reg122                   ; //0x2B4
volatile uint32_t                                                    null_reg123                   ; //0x2B8
volatile uint32_t                                                    null_reg124                   ; //0x2BC
volatile uint32_t                                                    null_reg125                   ; //0x2C0
volatile uint32_t                                                    null_reg126                   ; //0x2C4
volatile uint32_t                                                    null_reg127                   ; //0x2C8
volatile uint32_t                                                    null_reg128                   ; //0x2CC
volatile uint32_t                                                    null_reg129                   ; //0x2D0
volatile uint32_t                                                    null_reg130                   ; //0x2D4
volatile uint32_t                                                    null_reg131                   ; //0x2D8
volatile uint32_t                                                    null_reg132                   ; //0x2DC
volatile uint32_t                                                    null_reg133                   ; //0x2E0
volatile uint32_t                                                    null_reg134                   ; //0x2E4
volatile uint32_t                                                    null_reg135                   ; //0x2E8
volatile uint32_t                                                    null_reg136                   ; //0x2EC
volatile uint32_t                                                    null_reg137                   ; //0x2F0
volatile uint32_t                                                    null_reg138                   ; //0x2F4
volatile uint32_t                                                    null_reg139                   ; //0x2F8
volatile uint32_t                                                    null_reg140                   ; //0x2FC
volatile LLE_TOP_REG_LLE_LPCU_CFGA                                   LLE_LPCU_CFGA                 ; //0x300
volatile LLE_TOP_REG_LLE_LPCU_CFGB                                   LLE_LPCU_CFGB                 ; //0x304
volatile LLE_TOP_REG_LLE_TBCU_SLEEP_TIME                             LLE_TBCU_SLEEP_TIME           ; //0x308
volatile LLE_TOP_REG_LLE_TBCU_CUR_SLEEP_CNT                          LLE_TBCU_CUR_SLEEP_CNT        ; //0x30C
volatile LLE_TOP_REG_LLE_TBCU_SLEEP_INC                              LLE_TBCU_SLEEP_INC            ; //0x310
volatile LLE_TOP_REG_LLE_TBCU_CALIB                                  LLE_TBCU_CALIB                ; //0x314
volatile LLE_TOP_REG_LLE_TBCU_RF_STBTIME                             LLE_TBCU_RF_STBTIME           ; //0x318
volatile LLE_TOP_REG_LLE_TBCU_OSC_STBTIME                            LLE_TBCU_OSC_STBTIME          ; //0x31C
volatile LLE_TOP_REG_LLE_TBCU_BASETSCNT_LSB                          LLE_TBCU_BASETSCNT_LSB        ; //0x320
volatile LLE_TOP_REG_LLE_TBCU_BASETSCNT_HSB                          LLE_TBCU_BASETSCNT_HSB        ; //0x324
volatile LLE_TOP_REG_LLE_TBCU_SUBTSCNT                               LLE_TBCU_SUBTSCNT             ; //0x328
volatile LLE_TOP_REG_LLE_TBCU_LOCK_BASETSCNT_LSB                     LLE_TBCU_LOCK_BASETSCNT_LSB   ; //0x32C
volatile LLE_TOP_REG_LLE_TBCU_LOCK_BASETSCNT_HSB                     LLE_TBCU_LOCK_BASETSCNT_HSB   ; //0x330
volatile LLE_TOP_REG_LLE_TBCU_LOCK_SUBTSCNT                          LLE_TBCU_LOCK_SUBTSCNT        ; //0x334
volatile LLE_TOP_REG_LLE_TBCU_CNT_LOCK                               LLE_TBCU_CNT_LOCK             ; //0x338
volatile LLE_TOP_REG_LLE_TBCU_LONG_TIME                              LLE_TBCU_LONG_TIME            ; //0x33C
volatile LLE_TOP_REG_LLE_TBCU_SHORT_TIME                             LLE_TBCU_SHORT_TIME           ; //0x340
volatile LLE_TOP_REG_LLE_TBCU_US_TIME_L                              LLE_TBCU_US_TIME_L            ; //0x344
volatile LLE_TOP_REG_LLE_TBCU_US_TIME_H                              LLE_TBCU_US_TIME_H            ; //0x348
volatile LLE_TOP_REG_LLE_EXT_WAKEUP_DLY_CTRL                         LLE_EXT_WAKEUP_DLY_CTRL       ; //0x34C
volatile LLE_TOP_REG_LLE_GPIO0_EN                                    LLE_GPIO0_EN                  ; //0x350
volatile LLE_TOP_REG_LLE_GPIO0_LOCK_BASETSCNT_LSB                    LLE_GPIO0_LOCK_BASETSCNT_LSB  ; //0x354
volatile LLE_TOP_REG_LLE_GOIO0_LOCK_BASETSCNT_HSB                    LLE_GOIO0_LOCK_BASETSCNT_HSB  ; //0x358
volatile LLE_TOP_REG_LLE_GPIO0_LOCK_SUBTSCNT                         LLE_GPIO0_LOCK_SUBTSCNT       ; //0x35C
volatile LLE_TOP_REG_LLE_GPIO1_EN                                    LLE_GPIO1_EN                  ; //0x360
volatile LLE_TOP_REG_LLE_GPIO1_LOCK_BASETSCNT_LSB                    LLE_GPIO1_LOCK_BASETSCNT_LSB  ; //0x364
volatile LLE_TOP_REG_LLE_GOIO1_LOCK_BASETSCNT_HSB                    LLE_GOIO1_LOCK_BASETSCNT_HSB  ; //0x368
volatile LLE_TOP_REG_LLE_GPIO1_LOCK_SUBTSCNT                         LLE_GPIO1_LOCK_SUBTSCNT       ; //0x36C
volatile LLE_TOP_REG_LLE_TBCU_CUR_SLEEP_LOCK_CNT                     LLE_TBCU_CUR_SLEEP_LOCK_CNT   ; //0x370
volatile uint32_t                                                    null_reg141                   ; //0x374
volatile uint32_t                                                    null_reg142                   ; //0x378
volatile uint32_t                                                    null_reg143                   ; //0x37C
volatile uint32_t                                                    null_reg144                   ; //0x380
volatile uint32_t                                                    null_reg145                   ; //0x384
volatile uint32_t                                                    null_reg146                   ; //0x388
volatile uint32_t                                                    null_reg147                   ; //0x38C
volatile uint32_t                                                    null_reg148                   ; //0x390
volatile uint32_t                                                    null_reg149                   ; //0x394
volatile uint32_t                                                    null_reg150                   ; //0x398
volatile uint32_t                                                    null_reg151                   ; //0x39C
volatile uint32_t                                                    null_reg152                   ; //0x3A0
volatile uint32_t                                                    null_reg153                   ; //0x3A4
volatile uint32_t                                                    null_reg154                   ; //0x3A8
volatile uint32_t                                                    null_reg155                   ; //0x3AC
volatile uint32_t                                                    null_reg156                   ; //0x3B0
volatile uint32_t                                                    null_reg157                   ; //0x3B4
volatile uint32_t                                                    null_reg158                   ; //0x3B8
volatile uint32_t                                                    null_reg159                   ; //0x3BC
volatile uint32_t                                                    null_reg160                   ; //0x3C0
volatile uint32_t                                                    null_reg161                   ; //0x3C4
volatile uint32_t                                                    null_reg162                   ; //0x3C8
volatile uint32_t                                                    null_reg163                   ; //0x3CC
volatile uint32_t                                                    null_reg164                   ; //0x3D0
volatile uint32_t                                                    null_reg165                   ; //0x3D4
volatile uint32_t                                                    null_reg166                   ; //0x3D8
volatile uint32_t                                                    null_reg167                   ; //0x3DC
volatile uint32_t                                                    null_reg168                   ; //0x3E0
volatile uint32_t                                                    null_reg169                   ; //0x3E4
volatile uint32_t                                                    null_reg170                   ; //0x3E8
volatile uint32_t                                                    null_reg171                   ; //0x3EC
volatile uint32_t                                                    null_reg172                   ; //0x3F0
volatile uint32_t                                                    null_reg173                   ; //0x3F4
volatile uint32_t                                                    null_reg174                   ; //0x3F8
volatile uint32_t                                                    null_reg175                   ; //0x3FC
volatile LLE_TOP_REG_LLE_RF_CFG0                                     LLE_RF_CFG0                   ; //0x400
volatile LLE_TOP_REG_LLE_RF_CFG1                                     LLE_RF_CFG1                   ; //0x404
volatile LLE_TOP_REG_LLE_RF_CFG2                                     LLE_RF_CFG2                   ; //0x408
volatile LLE_TOP_REG_LLE_RF_CFG3                                     LLE_RF_CFG3                   ; //0x40C
volatile LLE_TOP_REG_LLE_RF_CFG4                                     LLE_RF_CFG4                   ; //0x410
volatile LLE_TOP_REG_LLE_RF_CFG5                                     LLE_RF_CFG5                   ; //0x414
volatile LLE_TOP_REG_LLE_RF_SPI_BUF                                  LLE_RF_SPI_BUF                ; //0x418
volatile LLE_TOP_REG_LLE_RF_POWDLY                                   LLE_RF_POWDLY                 ; //0x41C
volatile LLE_TOP_REG_LLE_RF_TEST_CFG                                 LLE_RF_TEST_CFG               ; //0x420
volatile LLE_TOP_REG_LLE_IFS_TIME_CHECK                              LLE_IFS_TIME_CHECK            ; //0x424
volatile LLE_TOP_REG_LLE_RF_CFG6                                     LLE_RF_CFG6                   ; //0x428
volatile uint32_t                                                    null_reg176                   ; //0x42C
volatile uint32_t                                                    null_reg177                   ; //0x430
volatile uint32_t                                                    null_reg178                   ; //0x434
volatile uint32_t                                                    null_reg179                   ; //0x438
volatile uint32_t                                                    null_reg180                   ; //0x43C
volatile uint32_t                                                    null_reg181                   ; //0x440
volatile uint32_t                                                    null_reg182                   ; //0x444
volatile uint32_t                                                    null_reg183                   ; //0x448
volatile uint32_t                                                    null_reg184                   ; //0x44C
volatile uint32_t                                                    null_reg185                   ; //0x450
volatile uint32_t                                                    null_reg186                   ; //0x454
volatile uint32_t                                                    null_reg187                   ; //0x458
volatile uint32_t                                                    null_reg188                   ; //0x45C
volatile uint32_t                                                    null_reg189                   ; //0x460
volatile uint32_t                                                    null_reg190                   ; //0x464
volatile uint32_t                                                    null_reg191                   ; //0x468
volatile uint32_t                                                    null_reg192                   ; //0x46C
volatile uint32_t                                                    null_reg193                   ; //0x470
volatile uint32_t                                                    null_reg194                   ; //0x474
volatile uint32_t                                                    null_reg195                   ; //0x478
volatile uint32_t                                                    null_reg196                   ; //0x47C
volatile uint32_t                                                    null_reg197                   ; //0x480
volatile uint32_t                                                    null_reg198                   ; //0x484
volatile uint32_t                                                    null_reg199                   ; //0x488
volatile uint32_t                                                    null_reg200                   ; //0x48C
volatile uint32_t                                                    null_reg201                   ; //0x490
volatile uint32_t                                                    null_reg202                   ; //0x494
volatile uint32_t                                                    null_reg203                   ; //0x498
volatile uint32_t                                                    null_reg204                   ; //0x49C
volatile uint32_t                                                    null_reg205                   ; //0x4A0
volatile uint32_t                                                    null_reg206                   ; //0x4A4
volatile uint32_t                                                    null_reg207                   ; //0x4A8
volatile uint32_t                                                    null_reg208                   ; //0x4AC
volatile uint32_t                                                    null_reg209                   ; //0x4B0
volatile uint32_t                                                    null_reg210                   ; //0x4B4
volatile uint32_t                                                    null_reg211                   ; //0x4B8
volatile uint32_t                                                    null_reg212                   ; //0x4BC
volatile uint32_t                                                    null_reg213                   ; //0x4C0
volatile uint32_t                                                    null_reg214                   ; //0x4C4
volatile uint32_t                                                    null_reg215                   ; //0x4C8
volatile uint32_t                                                    null_reg216                   ; //0x4CC
volatile uint32_t                                                    null_reg217                   ; //0x4D0
volatile uint32_t                                                    null_reg218                   ; //0x4D4
volatile uint32_t                                                    null_reg219                   ; //0x4D8
volatile uint32_t                                                    null_reg220                   ; //0x4DC
volatile uint32_t                                                    null_reg221                   ; //0x4E0
volatile uint32_t                                                    null_reg222                   ; //0x4E4
volatile uint32_t                                                    null_reg223                   ; //0x4E8
volatile uint32_t                                                    null_reg224                   ; //0x4EC
volatile uint32_t                                                    null_reg225                   ; //0x4F0
volatile uint32_t                                                    null_reg226                   ; //0x4F4
volatile uint32_t                                                    null_reg227                   ; //0x4F8
volatile uint32_t                                                    null_reg228                   ; //0x4FC
volatile LLE_TOP_REG_LLE_AES_CFG                                     LLE_AES_CFG                   ; //0x500
volatile LLE_TOP_REG_LLE_AES_KEY0                                    LLE_AES_KEY0                  ; //0x504
volatile LLE_TOP_REG_LLE_AES_KEY1                                    LLE_AES_KEY1                  ; //0x508
volatile LLE_TOP_REG_LLE_AES_KEY2                                    LLE_AES_KEY2                  ; //0x50C
volatile LLE_TOP_REG_LLE_AES_KEY3                                    LLE_AES_KEY3                  ; //0x510
volatile uint32_t                                                    null_reg229                   ; //0x514
volatile uint32_t                                                    null_reg230                   ; //0x518
volatile LLE_TOP_REG_LLE_AES_LOC                                     LLE_AES_LOC                   ; //0x51C
volatile LLE_TOP_REG_LLE_CCM_TXMIC                                   LLE_CCM_TXMIC                 ; //0x520
volatile LLE_TOP_REG_LLE_CCM_RXMIC                                   LLE_CCM_RXMIC                 ; //0x524
volatile uint32_t                                                    null_reg231                   ; //0x528
volatile uint32_t                                                    null_reg232                   ; //0x52C
volatile uint32_t                                                    null_reg233                   ; //0x530
volatile uint32_t                                                    null_reg234                   ; //0x534
volatile LLE_TOP_REG_LLE_MESHCCM_CFG                                 LLE_MESHCCM_CFG               ; //0x538
volatile LLE_TOP_REG_LLE_MESHCCM_LOC                                 LLE_MESHCCM_LOC               ; //0x53C
volatile LLE_TOP_REG_LLE_MESHCCM_KEY0                                LLE_MESHCCM_KEY0              ; //0x540
volatile LLE_TOP_REG_LLE_MESHCCM_KEY1                                LLE_MESHCCM_KEY1              ; //0x544
volatile LLE_TOP_REG_LLE_MESHCCM_KEY2                                LLE_MESHCCM_KEY2              ; //0x548
volatile LLE_TOP_REG_LLE_MESHCCM_KEY3                                LLE_MESHCCM_KEY3              ; //0x54C
volatile LLE_TOP_REG_LLE_MESHCCM_AUT_DATA0                           LLE_MESHCCM_AUT_DATA0         ; //0x550
volatile LLE_TOP_REG_LLE_MESHCCM_AUT_DATA1                           LLE_MESHCCM_AUT_DATA1         ; //0x554
volatile LLE_TOP_REG_LLE_MESHCCM_AUT_DATA2                           LLE_MESHCCM_AUT_DATA2         ; //0x558
volatile LLE_TOP_REG_LLE_MESHCCM_AUT_DATA3                           LLE_MESHCCM_AUT_DATA3         ; //0x55C
volatile LLE_TOP_REG_LLE_MESHCCM_NONCE0                              LLE_MESHCCM_NONCE0            ; //0x560
volatile LLE_TOP_REG_LLE_MESHCCM_NONCE1                              LLE_MESHCCM_NONCE1            ; //0x564
volatile LLE_TOP_REG_LLE_MESHCCM_NONCE2                              LLE_MESHCCM_NONCE2            ; //0x568
volatile LLE_TOP_REG_LLE_MESHCCM_NONCE3                              LLE_MESHCCM_NONCE3            ; //0x56C
volatile uint32_t                                                    null_reg235                   ; //0x570
volatile uint32_t                                                    null_reg236                   ; //0x574
volatile uint32_t                                                    null_reg237                   ; //0x578
volatile uint32_t                                                    null_reg238                   ; //0x57C
volatile LLE_TOP_REG_LLE_RF_FE_CTRIM0                                LLE_RF_FE_CTRIM0              ; //0x580
volatile LLE_TOP_REG_LLE_RF_FE_CTRIM1                                LLE_RF_FE_CTRIM1              ; //0x584
volatile LLE_TOP_REG_LLE_RF_FE_CTRIM2                                LLE_RF_FE_CTRIM2              ; //0x588
volatile LLE_TOP_REG_LLE_RF_FE_CTRIM3                                LLE_RF_FE_CTRIM3              ; //0x58C
volatile LLE_TOP_REG_LLE_RF_FE_CTRIM4                                LLE_RF_FE_CTRIM4              ; //0x590
volatile uint32_t                                                    null_reg239                   ; //0x594
volatile uint32_t                                                    null_reg240                   ; //0x598
volatile uint32_t                                                    null_reg241                   ; //0x59C
volatile uint32_t                                                    null_reg242                   ; //0x5A0
volatile uint32_t                                                    null_reg243                   ; //0x5A4
volatile uint32_t                                                    null_reg244                   ; //0x5A8
volatile uint32_t                                                    null_reg245                   ; //0x5AC
volatile uint32_t                                                    null_reg246                   ; //0x5B0
volatile uint32_t                                                    null_reg247                   ; //0x5B4
volatile uint32_t                                                    null_reg248                   ; //0x5B8
volatile uint32_t                                                    null_reg249                   ; //0x5BC
volatile uint32_t                                                    null_reg250                   ; //0x5C0
volatile uint32_t                                                    null_reg251                   ; //0x5C4
volatile uint32_t                                                    null_reg252                   ; //0x5C8
volatile uint32_t                                                    null_reg253                   ; //0x5CC
volatile uint32_t                                                    null_reg254                   ; //0x5D0
volatile uint32_t                                                    null_reg255                   ; //0x5D4
volatile uint32_t                                                    null_reg256                   ; //0x5D8
volatile uint32_t                                                    null_reg257                   ; //0x5DC
volatile uint32_t                                                    null_reg258                   ; //0x5E0
volatile uint32_t                                                    null_reg259                   ; //0x5E4
volatile uint32_t                                                    null_reg260                   ; //0x5E8
volatile uint32_t                                                    null_reg261                   ; //0x5EC
volatile uint32_t                                                    null_reg262                   ; //0x5F0
volatile uint32_t                                                    null_reg263                   ; //0x5F4
volatile uint32_t                                                    null_reg264                   ; //0x5F8
volatile uint32_t                                                    null_reg265                   ; //0x5FC
volatile uint32_t                                                    null_reg266                   ; //0x600
volatile uint32_t                                                    null_reg267                   ; //0x604
volatile uint32_t                                                    null_reg268                   ; //0x608
volatile uint32_t                                                    null_reg269                   ; //0x60C
volatile uint32_t                                                    null_reg270                   ; //0x610
volatile uint32_t                                                    null_reg271                   ; //0x614
volatile uint32_t                                                    null_reg272                   ; //0x618
volatile uint32_t                                                    null_reg273                   ; //0x61C
volatile uint32_t                                                    null_reg274                   ; //0x620
volatile uint32_t                                                    null_reg275                   ; //0x624
volatile uint32_t                                                    null_reg276                   ; //0x628
volatile uint32_t                                                    null_reg277                   ; //0x62C
volatile uint32_t                                                    null_reg278                   ; //0x630
volatile uint32_t                                                    null_reg279                   ; //0x634
volatile uint32_t                                                    null_reg280                   ; //0x638
volatile uint32_t                                                    null_reg281                   ; //0x63C
volatile uint32_t                                                    null_reg282                   ; //0x640
volatile uint32_t                                                    null_reg283                   ; //0x644
volatile uint32_t                                                    null_reg284                   ; //0x648
volatile uint32_t                                                    null_reg285                   ; //0x64C
volatile uint32_t                                                    null_reg286                   ; //0x650
volatile uint32_t                                                    null_reg287                   ; //0x654
volatile uint32_t                                                    null_reg288                   ; //0x658
volatile uint32_t                                                    null_reg289                   ; //0x65C
volatile uint32_t                                                    null_reg290                   ; //0x660
volatile uint32_t                                                    null_reg291                   ; //0x664
volatile uint32_t                                                    null_reg292                   ; //0x668
volatile uint32_t                                                    null_reg293                   ; //0x66C
volatile uint32_t                                                    null_reg294                   ; //0x670
volatile uint32_t                                                    null_reg295                   ; //0x674
volatile uint32_t                                                    null_reg296                   ; //0x678
volatile uint32_t                                                    null_reg297                   ; //0x67C
volatile uint32_t                                                    null_reg298                   ; //0x680
volatile uint32_t                                                    null_reg299                   ; //0x684
volatile uint32_t                                                    null_reg300                   ; //0x688
volatile uint32_t                                                    null_reg301                   ; //0x68C
volatile uint32_t                                                    null_reg302                   ; //0x690
volatile uint32_t                                                    null_reg303                   ; //0x694
volatile uint32_t                                                    null_reg304                   ; //0x698
volatile uint32_t                                                    null_reg305                   ; //0x69C
volatile uint32_t                                                    null_reg306                   ; //0x6A0
volatile uint32_t                                                    null_reg307                   ; //0x6A4
volatile uint32_t                                                    null_reg308                   ; //0x6A8
volatile uint32_t                                                    null_reg309                   ; //0x6AC
volatile uint32_t                                                    null_reg310                   ; //0x6B0
volatile uint32_t                                                    null_reg311                   ; //0x6B4
volatile uint32_t                                                    null_reg312                   ; //0x6B8
volatile uint32_t                                                    null_reg313                   ; //0x6BC
volatile uint32_t                                                    null_reg314                   ; //0x6C0
volatile uint32_t                                                    null_reg315                   ; //0x6C4
volatile uint32_t                                                    null_reg316                   ; //0x6C8
volatile uint32_t                                                    null_reg317                   ; //0x6CC
volatile uint32_t                                                    null_reg318                   ; //0x6D0
volatile uint32_t                                                    null_reg319                   ; //0x6D4
volatile uint32_t                                                    null_reg320                   ; //0x6D8
volatile uint32_t                                                    null_reg321                   ; //0x6DC
volatile uint32_t                                                    null_reg322                   ; //0x6E0
volatile uint32_t                                                    null_reg323                   ; //0x6E4
volatile uint32_t                                                    null_reg324                   ; //0x6E8
volatile uint32_t                                                    null_reg325                   ; //0x6EC
volatile uint32_t                                                    null_reg326                   ; //0x6F0
volatile uint32_t                                                    null_reg327                   ; //0x6F4
volatile uint32_t                                                    null_reg328                   ; //0x6F8
volatile uint32_t                                                    null_reg329                   ; //0x6FC
volatile LLE_TOP_REG_LLE_PRIVATE_CTRL                                LLE_PRIVATE_CTRL              ; //0x700
}LLE_TOP_REG_H;

#endif //LLE_TOP_REG__H
