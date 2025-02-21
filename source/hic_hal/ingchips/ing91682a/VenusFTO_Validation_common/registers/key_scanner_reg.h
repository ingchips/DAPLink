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
// File         : key_scanner_reg.h
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
#ifndef KEY_SCANNER_REG__H
#define KEY_SCANNER_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_scanner_en                   : 1 ;
        volatile uint32_t en_debounce                      : 20;
        volatile uint32_t debounce_cnt_cfg                 : 8 ;
        volatile uint32_t dma_en                           : 1 ;
        volatile uint32_t db_clk_sel                       : 1 ;
        volatile uint32_t unused0                          : 1 ;
    }f;
}key_scanner_REG_key_scanner_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t release_cfg                      : 11;
        volatile uint32_t scan_itv_cfg                     : 16;
        volatile uint32_t unused1                          : 5 ;
    }f;
}key_scanner_REG_key_scanner_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_mask_row                     : 20;
        volatile uint32_t unused2                          : 12;
    }f;
}key_scanner_REG_key_row_mask_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_mask_col                     : 20;
        volatile uint32_t unused3                          : 12;
    }f;
}key_scanner_REG_key_col_mask_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_trig_en                      : 1 ;
        volatile uint32_t int_fifo_full_en                 : 1 ;
        volatile uint32_t int_fifo_empty_en                : 1 ;
        volatile uint32_t int_fifo_halffull_en             : 1 ;
        volatile uint32_t fifo_clr_reg                     : 1 ;
        volatile uint32_t clr_int_loop                     : 1 ;
        volatile uint32_t int_loop_en                      : 1 ;
        volatile uint32_t unused4                          : 25;
    }f;
}key_scanner_REG_key_int_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_trig                         : 1 ;
        volatile uint32_t int_fifo_full                    : 1 ;
        volatile uint32_t int_fifo_empty                   : 1 ;
        volatile uint32_t int_fifo_halffull                : 1 ;
        volatile uint32_t int_trig_raw                     : 1 ;
        volatile uint32_t int_fifo_full_raw                : 1 ;
        volatile uint32_t int_fifo_empty_raw               : 1 ;
        volatile uint32_t int_fifo_halffull_raw            : 1 ;
        volatile uint32_t int_loop_cnt                     : 1 ;
        volatile uint32_t unused5                          : 23;
    }f;
}key_scanner_REG_key_int;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t fifo_cnt                         : 6 ;
        volatile uint32_t key_scanner_busy                 : 1 ;
        volatile uint32_t cnt_loop                         : 3 ;
        volatile uint32_t unused6                          : 22;
    }f;
}key_scanner_REG_fifo_status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_data                         : 11;
        volatile uint32_t unused7                          : 21;
    }f;
}key_scanner_REG_key_data;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_trig_cfg                     : 5 ;
        volatile uint32_t dma_trig_cfg                     : 5 ;
        volatile uint32_t int_loop_cfg                     : 3 ;
        volatile uint32_t unused8                          : 19;
    }f;
}key_scanner_REG_key_trig;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t fifo_th_cfg                      : 5 ;
        volatile uint32_t err_en                           : 1 ;
        volatile uint32_t err_flag                         : 1 ;
        volatile uint32_t unused9                          : 25;
    }f;
}key_scanner_REG_key_err_cfg;

typedef struct
{
volatile key_scanner_REG_key_scanner_ctrl0                           key_scanner_ctrl0             ; //0x0
volatile key_scanner_REG_key_scanner_ctrl1                           key_scanner_ctrl1             ; //0x4
volatile key_scanner_REG_key_row_mask_ctrl                           key_row_mask_ctrl             ; //0x8
volatile key_scanner_REG_key_col_mask_ctrl                           key_col_mask_ctrl             ; //0xC
volatile key_scanner_REG_key_int_en                                  key_int_en                    ; //0x10
volatile key_scanner_REG_key_int                                     key_int                       ; //0x14
volatile key_scanner_REG_fifo_status                                 fifo_status                   ; //0x18
volatile key_scanner_REG_key_data                                    key_data                      ; //0x1C
volatile key_scanner_REG_key_trig                                    key_trig                      ; //0x20
volatile key_scanner_REG_key_err_cfg                                 key_err_cfg                   ; //0x24
}key_scanner_REG_H;

#endif //key_scanner_REG__H
