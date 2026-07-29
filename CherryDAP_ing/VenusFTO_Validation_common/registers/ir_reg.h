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
// File         : ir_reg.h
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
#ifndef IR_REG__H
#define IR_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_en                            : 1 ;
        volatile uint32_t ir_mode                          : 2 ;
        volatile uint32_t ir_end_detect_en                 : 1 ;
        volatile uint32_t ir_int_en                        : 1 ;
        volatile uint32_t ir_int_verify_en                 : 1 ;
        volatile uint32_t ir_usercode_verify               : 1 ;
        volatile uint32_t ir_datacode_verify               : 1 ;
        volatile uint32_t bit_time_1                       : 7 ;
        volatile uint32_t tx_repeat_mode                   : 1 ;
        volatile uint32_t bit_time_2                       : 7 ;
        volatile uint32_t txrx_mode                        : 1 ;
        volatile uint32_t ir_bit_cycle                     : 7 ;
        volatile uint32_t unused0                          : 1 ;
    }f;
}IR_REG_ir_ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tx_start                         : 1 ;
        volatile uint32_t ir_pol                           : 1 ;
        volatile uint32_t ir_debug_sel                     : 1 ;
        volatile uint32_t carrier_cnt_clr                  : 1 ;
        volatile uint32_t rc5_toggle_bit                   : 1 ;
        volatile uint32_t unused1                          : 27;
    }f;
}IR_REG_ir_tx_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_carry_low                     : 9 ;
        volatile uint32_t ir_carry_high                    : 9 ;
        volatile uint32_t unused2                          : 14;
    }f;
}IR_REG_ir_carry_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_time_s1                       : 12;
        volatile uint32_t unused3                          : 20;
    }f;
}IR_REG_ir_time_1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_time_s2                       : 12;
        volatile uint32_t unused4                          : 20;
    }f;
}IR_REG_ir_time_2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_time_s3                       : 12;
        volatile uint32_t unused5                          : 20;
    }f;
}IR_REG_ir_time_3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_time_s4                       : 12;
        volatile uint32_t unused6                          : 20;
    }f;
}IR_REG_ir_time_4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_time_s5                       : 12;
        volatile uint32_t unused7                          : 20;
    }f;
}IR_REG_ir_time_5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_rx_usercode                   : 16;
        volatile uint32_t ir_rx_datacode                   : 16;
    }f;
}IR_REG_ir_rx_code;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_tx_usercode                   : 16;
        volatile uint32_t ir_tx_datacode                   : 16;
    }f;
}IR_REG_ir_tx_code;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ir_received_ok                   : 1 ;
        volatile uint32_t ir_usercode_verify               : 1 ;
        volatile uint32_t ir_datacode_verify               : 1 ;
        volatile uint32_t ir_repeat                        : 1 ;
        volatile uint32_t ir_transmit_ok                   : 1 ;
        volatile uint32_t ir_tx_repeat                     : 1 ;
        volatile uint32_t ir_rx_rc5_toggle_bit             : 1 ;
        volatile uint32_t unused8                          : 25;
    }f;
}IR_REG_ir_fsm;

typedef struct
{
volatile IR_REG_ir_ctrl                                              ir_ctrl                       ; //0x0
volatile IR_REG_ir_tx_config                                         ir_tx_config                  ; //0x4
volatile IR_REG_ir_carry_config                                      ir_carry_config               ; //0x8
volatile IR_REG_ir_time_1                                            ir_time_1                     ; //0xC
volatile IR_REG_ir_time_2                                            ir_time_2                     ; //0x10
volatile IR_REG_ir_time_3                                            ir_time_3                     ; //0x14
volatile IR_REG_ir_time_4                                            ir_time_4                     ; //0x18
volatile IR_REG_ir_time_5                                            ir_time_5                     ; //0x1C
volatile IR_REG_ir_rx_code                                           ir_rx_code                    ; //0x20
volatile IR_REG_ir_tx_code                                           ir_tx_code                    ; //0x24
volatile IR_REG_ir_fsm                                               ir_fsm                        ; //0x28
}IR_REG_H;

#endif //IR_REG__H
