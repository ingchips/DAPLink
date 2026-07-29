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
// File         : sadc_reg.h
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
#ifndef SADC_REG__H
#define SADC_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t adc_mode                         : 1 ;
        volatile uint32_t en_pga                           : 1 ;
        volatile uint32_t g_pga                            : 3 ;
        volatile uint32_t adc_ana_rstn                     : 1 ;
        volatile uint32_t calrpt                           : 2 ;
        volatile uint32_t sdif                             : 1 ;
        volatile uint32_t adc_ana_en                       : 1 ;
        volatile uint32_t ctrl_dr                          : 1 ;
        volatile uint32_t soc_reg                          : 1 ;
        volatile uint32_t smux_reg                         : 4 ;
        volatile uint32_t adc_clk_sel                      : 1 ;
        volatile uint32_t en_soc_fall                      : 1 ;
        volatile uint32_t soc_fall_cfg                     : 4 ;
        volatile uint32_t en_wr_cnt                        : 1 ;
        volatile uint32_t wr_cnt_cfg                       : 4 ;
        volatile uint32_t wr_eoc_sel                       : 1 ;
        volatile uint32_t unused0                          : 4 ;
    }f;
}sadc_REG_sadc_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t loop_delay                       : 32;
    }f;
}sadc_REG_sadc_cfg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t adc_ctrl_mode                    : 1 ;
        volatile uint32_t dish                             : 1 ;
        volatile uint32_t adc_start                        : 1 ;
        volatile uint32_t en_ch                            : 12;
        volatile uint32_t dma_en                           : 1 ;
        volatile uint32_t dma_trig_cfg                     : 4 ;
        volatile uint32_t int_trig_cfg                     : 4 ;
        volatile uint32_t dly_cnt_cfg                      : 4 ;
        volatile uint32_t unused1                          : 4 ;
    }f;
}sadc_REG_sadc_cfg2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sadc_data                        : 18;
        volatile uint32_t unused2                          : 14;
    }f;
}sadc_REG_sadc_data;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused3                          : 12;
        volatile uint32_t wr_ptr                           : 5 ;
        volatile uint32_t rd_ptr                           : 5 ;
        volatile uint32_t fifo_clr_reg                     : 1 ;
        volatile uint32_t busy                             : 1 ;
        volatile uint32_t unused4                          : 8 ;
    }f;
}sadc_REG_sadc_status0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_fifo_empty_en                : 1 ;
        volatile uint32_t int_fifo_full_en                 : 1 ;
        volatile uint32_t int_fifo_halffull_en             : 1 ;
        volatile uint32_t int_sadc_trig_en                 : 1 ;
        volatile uint32_t unused5                          : 28;
    }f;
}sadc_REG_sadc_int_mask;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t fifo_empty                       : 1 ;
        volatile uint32_t fifo_full                        : 1 ;
        volatile uint32_t fifo_half                        : 1 ;
        volatile uint32_t int_sadc_trig                    : 1 ;
        volatile uint32_t int_sadc                         : 1 ;
        volatile uint32_t unused6                          : 27;
    }f;
}sadc_REG_sadc_int;

typedef struct
{
volatile sadc_REG_sadc_cfg0                                          sadc_cfg0                     ; //0x0
volatile sadc_REG_sadc_cfg1                                          sadc_cfg1                     ; //0x4
volatile sadc_REG_sadc_cfg2                                          sadc_cfg2                     ; //0x8
volatile sadc_REG_sadc_data                                          sadc_data                     ; //0xC
volatile sadc_REG_sadc_status0                                       sadc_status0                  ; //0x10
volatile uint32_t                                                    null_reg0                     ; //0x14
volatile uint32_t                                                    null_reg1                     ; //0x18
volatile uint32_t                                                    null_reg2                     ; //0x1C
volatile uint32_t                                                    null_reg3                     ; //0x20
volatile uint32_t                                                    null_reg4                     ; //0x24
volatile uint32_t                                                    null_reg5                     ; //0x28
volatile uint32_t                                                    null_reg6                     ; //0x2C
volatile sadc_REG_sadc_int_mask                                      sadc_int_mask                 ; //0x30
volatile sadc_REG_sadc_int                                           sadc_int                      ; //0x34
}sadc_REG_H;

#endif //sadc_REG__H
