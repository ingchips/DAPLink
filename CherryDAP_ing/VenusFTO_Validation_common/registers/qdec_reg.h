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
// File         : qdec_reg.h
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
#ifndef QDEC_REG__H
#define QDEC_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t clken_cmb                        : 1 ;
        volatile uint32_t clkdis_cmb                       : 1 ;
        volatile uint32_t swtrig_mem                       : 1 ;
        volatile uint32_t unused0                          : 29;
    }f;
}qdec_REG_channel0_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tcclks                           : 3 ;
        volatile uint32_t clki                             : 1 ;
        volatile uint32_t burst                            : 2 ;
        volatile uint32_t cpcstop                          : 1 ;
        volatile uint32_t cpcdis                           : 1 ;
        volatile uint32_t etrgedg                          : 2 ;
        volatile uint32_t etrg                             : 2 ;
        volatile uint32_t entrg                            : 1 ;
        volatile uint32_t mode_up_down                     : 1 ;
        volatile uint32_t cpctrg                           : 1 ;
        volatile uint32_t wave                             : 1 ;
        volatile uint32_t acpa                             : 2 ;
        volatile uint32_t acpc                             : 2 ;
        volatile uint32_t aetrg                            : 2 ;
        volatile uint32_t astrg                            : 2 ;
        volatile uint32_t bcpb                             : 2 ;
        volatile uint32_t bcpc                             : 2 ;
        volatile uint32_t betrg                            : 2 ;
        volatile uint32_t bswtrg                           : 2 ;
    }f;
}qdec_REG_channel0_mode;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t step_en                          : 1 ;
        volatile uint32_t step_down                        : 1 ;
        volatile uint32_t dma_req_a_mask                   : 1 ;
        volatile uint32_t dma_req_b_mask                   : 1 ;
        volatile uint32_t unused1                          : 28;
    }f;
}qdec_REG_channel0_step;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rab                           : 16;
        volatile uint32_t unused2                          : 16;
    }f;
}qdec_REG_channel0_read_rab;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_cv                            : 16;
        volatile uint32_t unused3                          : 16;
    }f;
}qdec_REG_channel0_read_tc;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_ra                            : 16;
        volatile uint32_t unused4                          : 16;
    }f;
}qdec_REG_channel0write_a;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rb                            : 16;
        volatile uint32_t unused5                          : 16;
    }f;
}qdec_REG_channel0write_b;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rc                            : 16;
        volatile uint32_t unused6                          : 16;
    }f;
}qdec_REG_channel0write_c;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_sr                            : 10;
        volatile uint32_t unused7                          : 6 ;
        volatile uint32_t clk_sta                          : 1 ;
        volatile uint32_t tioa                             : 1 ;
        volatile uint32_t tiob                             : 1 ;
        volatile uint32_t unused8                          : 13;
    }f;
}qdec_REG_channel0tiob0_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_enable                    : 10;
        volatile uint32_t unused9                          : 22;
    }f;
}qdec_REG_channel0int_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_disable                   : 10;
        volatile uint32_t unused10                         : 22;
    }f;
}qdec_REG_channel0int_dis;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr                           : 10;
        volatile uint32_t unused11                         : 22;
    }f;
}qdec_REG_channel0int_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t clken_cmb                        : 1 ;
        volatile uint32_t clkdis_cmb                       : 1 ;
        volatile uint32_t swtrig_mem                       : 1 ;
        volatile uint32_t unused12                         : 29;
    }f;
}qdec_REG_channel1_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tcclks                           : 3 ;
        volatile uint32_t clki                             : 1 ;
        volatile uint32_t burst                            : 2 ;
        volatile uint32_t cpcstop                          : 1 ;
        volatile uint32_t cpcdis                           : 1 ;
        volatile uint32_t etrgedg                          : 2 ;
        volatile uint32_t etrg                             : 2 ;
        volatile uint32_t entrg                            : 1 ;
        volatile uint32_t mode_up_down                     : 1 ;
        volatile uint32_t cpctrg                           : 1 ;
        volatile uint32_t wave                             : 1 ;
        volatile uint32_t acpa                             : 2 ;
        volatile uint32_t acpc                             : 2 ;
        volatile uint32_t aetrg                            : 2 ;
        volatile uint32_t astrg                            : 2 ;
        volatile uint32_t bcpb                             : 2 ;
        volatile uint32_t bcpc                             : 2 ;
        volatile uint32_t betrg                            : 2 ;
        volatile uint32_t bswtrg                           : 2 ;
    }f;
}qdec_REG_channel1_mode;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t step_en                          : 1 ;
        volatile uint32_t step_down                        : 1 ;
        volatile uint32_t dma_req_a_mask                   : 1 ;
        volatile uint32_t dma_req_b_mask                   : 1 ;
        volatile uint32_t unused13                         : 28;
    }f;
}qdec_REG_channel1_step;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rab                           : 16;
        volatile uint32_t unused14                         : 16;
    }f;
}qdec_REG_channel1_read_rab;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_cv                            : 16;
        volatile uint32_t unused15                         : 16;
    }f;
}qdec_REG_channel1_read_tc;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_ra                            : 16;
        volatile uint32_t unused16                         : 16;
    }f;
}qdec_REG_channel1write_a;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rb                            : 16;
        volatile uint32_t unused17                         : 16;
    }f;
}qdec_REG_channel1write_b;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rc                            : 16;
        volatile uint32_t unused18                         : 16;
    }f;
}qdec_REG_channel1write_c;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_sr                            : 10;
        volatile uint32_t unused19                         : 6 ;
        volatile uint32_t clk_sta                          : 1 ;
        volatile uint32_t tioa                             : 1 ;
        volatile uint32_t tiob                             : 1 ;
        volatile uint32_t unused20                         : 13;
    }f;
}qdec_REG_channel1tiob0_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_enable                    : 10;
        volatile uint32_t unused21                         : 22;
    }f;
}qdec_REG_channel1int_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_disable                   : 10;
        volatile uint32_t unused22                         : 22;
    }f;
}qdec_REG_channel1int_dis;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr                           : 10;
        volatile uint32_t unused23                         : 22;
    }f;
}qdec_REG_channel1int_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t clken_cmb                        : 1 ;
        volatile uint32_t clkdis_cmb                       : 1 ;
        volatile uint32_t swtrig_mem                       : 1 ;
        volatile uint32_t unused24                         : 29;
    }f;
}qdec_REG_channel2_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tcclks                           : 3 ;
        volatile uint32_t clki                             : 1 ;
        volatile uint32_t burst                            : 2 ;
        volatile uint32_t cpcstop                          : 1 ;
        volatile uint32_t cpcdis                           : 1 ;
        volatile uint32_t etrgedg                          : 2 ;
        volatile uint32_t etrg                             : 2 ;
        volatile uint32_t entrg                            : 1 ;
        volatile uint32_t mode_up_down                     : 1 ;
        volatile uint32_t cpctrg                           : 1 ;
        volatile uint32_t wave                             : 1 ;
        volatile uint32_t acpa                             : 2 ;
        volatile uint32_t acpc                             : 2 ;
        volatile uint32_t aetrg                            : 2 ;
        volatile uint32_t astrg                            : 2 ;
        volatile uint32_t bcpb                             : 2 ;
        volatile uint32_t bcpc                             : 2 ;
        volatile uint32_t betrg                            : 2 ;
        volatile uint32_t bswtrg                           : 2 ;
    }f;
}qdec_REG_channel2_mode;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t step_en                          : 1 ;
        volatile uint32_t step_down                        : 1 ;
        volatile uint32_t dma_req_a_mask                   : 1 ;
        volatile uint32_t dma_req_b_mask                   : 1 ;
        volatile uint32_t unused25                         : 28;
    }f;
}qdec_REG_channel2_step;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rab                           : 16;
        volatile uint32_t unused26                         : 16;
    }f;
}qdec_REG_channel2_read_rab;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_cv                            : 16;
        volatile uint32_t unused27                         : 16;
    }f;
}qdec_REG_channel2_read_tc;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_ra                            : 16;
        volatile uint32_t unused28                         : 16;
    }f;
}qdec_REG_channel2write_a;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rb                            : 16;
        volatile uint32_t unused29                         : 16;
    }f;
}qdec_REG_channel2write_b;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_rc                            : 16;
        volatile uint32_t unused30                         : 16;
    }f;
}qdec_REG_channel3write_c;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_sr                            : 10;
        volatile uint32_t unused31                         : 6 ;
        volatile uint32_t clk_sta                          : 1 ;
        volatile uint32_t tioa                             : 1 ;
        volatile uint32_t tiob                             : 1 ;
        volatile uint32_t unused32                         : 13;
    }f;
}qdec_REG_channel2tiob0_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_enable                    : 10;
        volatile uint32_t unused33                         : 22;
    }f;
}qdec_REG_channel2int_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr_disable                   : 10;
        volatile uint32_t unused34                         : 22;
    }f;
}qdec_REG_channel2int_dis;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc_imr                           : 10;
        volatile uint32_t unused35                         : 22;
    }f;
}qdec_REG_channel2int_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t bcr                              : 1 ;
        volatile uint32_t unused36                         : 7 ;
        volatile uint32_t softreset                        : 1 ;
        volatile uint32_t unused37                         : 23;
    }f;
}qdec_REG_bcr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tc0xc0s                          : 2 ;
        volatile uint32_t tc1xc1s                          : 2 ;
        volatile uint32_t tc2xc2s                          : 2 ;
        volatile uint32_t unused38                         : 2 ;
        volatile uint32_t i_qdec_en                        : 1 ;
        volatile uint32_t i_position_en                    : 1 ;
        volatile uint32_t i_speed_en                       : 1 ;
        volatile uint32_t i_qd_transparent                 : 1 ;
        volatile uint32_t i_count_mode                     : 1 ;
        volatile uint32_t i_invert_pha_a                   : 1 ;
        volatile uint32_t i_invert_pha_b                   : 1 ;
        volatile uint32_t i_invert_idx                     : 1 ;
        volatile uint32_t i_swap_phase                     : 1 ;
        volatile uint32_t i_phb_is_idx                     : 1 ;
        volatile uint32_t i_autocrt_en                     : 1 ;
        volatile uint32_t i_filter                         : 1 ;
        volatile uint32_t i_max_filter                     : 6 ;
        volatile uint32_t i_max_miss                       : 4 ;
        volatile uint32_t unused39                         : 2 ;
    }f;
}qdec_REG_bmr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t qdec_int_en                      : 4 ;
        volatile uint32_t unused40                         : 28;
    }f;
}qdec_REG_qdec_inten;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t qdec_int_dis                     : 4 ;
        volatile uint32_t unused41                         : 28;
    }f;
}qdec_REG_qdec_intdis;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t qdec_int_mask                    : 4 ;
        volatile uint32_t unused42                         : 28;
    }f;
}qdec_REG_qdec_inten_rd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t flag_it_index_cmb                : 1 ;
        volatile uint32_t flag_dir_change_cmb              : 1 ;
        volatile uint32_t flag_code_error_cmb              : 1 ;
        volatile uint32_t miss_edge_overflow_cmb           : 1 ;
        volatile uint32_t unused43                         : 4 ;
        volatile uint32_t direction                        : 1 ;
        volatile uint32_t qdec_status_sel                  : 23;
    }f;
}qdec_REG_qdec_status_sel;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t i_en0_fault                      : 1 ;
        volatile uint32_t i_en1_fault                      : 1 ;
        volatile uint32_t unused44                         : 30;
    }f;
}qdec_REG_pwm_fault;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dummy                            : 32;
    }f;
}qdec_REG_dummy;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t wr_protected                     : 1 ;
        volatile uint32_t unused45                         : 7 ;
        volatile uint32_t wr_p_data                        : 24;
    }f;
}qdec_REG_wpmode;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t addrsize                         : 32;
    }f;
}qdec_REG_addrsize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NAME1                            : 32;
    }f;
}qdec_REG_name1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NAME2                            : 32;
    }f;
}qdec_REG_name2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FEATURES                         : 32;
    }f;
}qdec_REG_FEATURES;

typedef struct
{
volatile qdec_REG_channel0_ctrl                                      channel0_ctrl                 ; //0x0
volatile qdec_REG_channel0_mode                                      channel0_mode                 ; //0x4
volatile qdec_REG_channel0_step                                      channel0_step                 ; //0x8
volatile qdec_REG_channel0_read_rab                                  channel0_read_rab             ; //0xC
volatile qdec_REG_channel0_read_tc                                   channel0_read_tc              ; //0x10
volatile qdec_REG_channel0write_a                                    channel0write_a               ; //0x14
volatile qdec_REG_channel0write_b                                    channel0write_b               ; //0x18
volatile qdec_REG_channel0write_c                                    channel0write_c               ; //0x1C
volatile qdec_REG_channel0tiob0_rd                                   channel0tiob0_rd              ; //0x20
volatile qdec_REG_channel0int_en                                     channel0int_en                ; //0x24
volatile qdec_REG_channel0int_dis                                    channel0int_dis               ; //0x28
volatile qdec_REG_channel0int_rd                                     channel0int_rd                ; //0x2C
volatile uint32_t                                                    null_reg0                     ; //0x30
volatile uint32_t                                                    null_reg1                     ; //0x34
volatile uint32_t                                                    null_reg2                     ; //0x38
volatile uint32_t                                                    null_reg3                     ; //0x3C
volatile qdec_REG_channel1_ctrl                                      channel1_ctrl                 ; //0x40
volatile qdec_REG_channel1_mode                                      channel1_mode                 ; //0x44
volatile qdec_REG_channel1_step                                      channel1_step                 ; //0x48
volatile qdec_REG_channel1_read_rab                                  channel1_read_rab             ; //0x4C
volatile qdec_REG_channel1_read_tc                                   channel1_read_tc              ; //0x50
volatile qdec_REG_channel1write_a                                    channel1write_a               ; //0x54
volatile qdec_REG_channel1write_b                                    channel1write_b               ; //0x58
volatile qdec_REG_channel1write_c                                    channel1write_c               ; //0x5C
volatile qdec_REG_channel1tiob0_rd                                   channel1tiob0_rd              ; //0x60
volatile qdec_REG_channel1int_en                                     channel1int_en                ; //0x64
volatile qdec_REG_channel1int_dis                                    channel1int_dis               ; //0x68
volatile qdec_REG_channel1int_rd                                     channel1int_rd                ; //0x6C
volatile uint32_t                                                    null_reg4                     ; //0x70
volatile uint32_t                                                    null_reg5                     ; //0x74
volatile uint32_t                                                    null_reg6                     ; //0x78
volatile uint32_t                                                    null_reg7                     ; //0x7C
volatile qdec_REG_channel2_ctrl                                      channel2_ctrl                 ; //0x80
volatile qdec_REG_channel2_mode                                      channel2_mode                 ; //0x84
volatile qdec_REG_channel2_step                                      channel2_step                 ; //0x88
volatile qdec_REG_channel2_read_rab                                  channel2_read_rab             ; //0x8C
volatile qdec_REG_channel2_read_tc                                   channel2_read_tc              ; //0x90
volatile qdec_REG_channel2write_a                                    channel2write_a               ; //0x94
volatile qdec_REG_channel2write_b                                    channel2write_b               ; //0x98
volatile qdec_REG_channel3write_c                                    channel3write_c               ; //0x9C
volatile qdec_REG_channel2tiob0_rd                                   channel2tiob0_rd              ; //0xA0
volatile qdec_REG_channel2int_en                                     channel2int_en                ; //0xA4
volatile qdec_REG_channel2int_dis                                    channel2int_dis               ; //0xA8
volatile qdec_REG_channel2int_rd                                     channel2int_rd                ; //0xAC
volatile uint32_t                                                    null_reg8                     ; //0xB0
volatile uint32_t                                                    null_reg9                     ; //0xB4
volatile uint32_t                                                    null_reg10                    ; //0xB8
volatile uint32_t                                                    null_reg11                    ; //0xBC
volatile qdec_REG_bcr                                                bcr                           ; //0xC0
volatile qdec_REG_bmr                                                bmr                           ; //0xC4
volatile qdec_REG_qdec_inten                                         qdec_inten                    ; //0xC8
volatile qdec_REG_qdec_intdis                                        qdec_intdis                   ; //0xCC
volatile qdec_REG_qdec_inten_rd                                      qdec_inten_rd                 ; //0xD0
volatile qdec_REG_qdec_status_sel                                    qdec_status_sel               ; //0xD4
volatile qdec_REG_pwm_fault                                          pwm_fault                     ; //0xD8
volatile uint32_t                                                    null_reg12                    ; //0xDC
volatile qdec_REG_dummy                                              dummy                         ; //0xE0
volatile qdec_REG_wpmode                                             wpmode                        ; //0xE4
volatile uint32_t                                                    null_reg13                    ; //0xE8
volatile qdec_REG_addrsize                                           addrsize                      ; //0xEC
volatile qdec_REG_name1                                              name1                         ; //0xF0
volatile qdec_REG_name2                                              name2                         ; //0xF4
volatile qdec_REG_FEATURES                                           FEATURES                      ; //0xF8
}qdec_REG_H;

#endif //qdec_REG__H
