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
// File         : pwm_reg.h
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
#ifndef PWM_REG__H
#define PWM_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_msk                           : 2 ;
        volatile uint32_t c0_haltctrl_en                   : 2 ;
        volatile uint32_t c0_haltctrl_cfg                  : 2 ;
        volatile uint32_t c0_enable_pwm                    : 1 ;
        volatile uint32_t c0_mode                          : 3 ;
        volatile uint32_t c0_inv_en                        : 2 ;
        volatile uint32_t c0_fifo_trig_cfg                 : 3 ;
        volatile uint32_t c0_fifo_clr                      : 1 ;
        volatile uint32_t c0_fifo_empty_en                 : 1 ;
        volatile uint32_t c0_fifo_full_en                  : 1 ;
        volatile uint32_t c0_fifo_halffull_en              : 1 ;
        volatile uint32_t c0_fifo_trig_en                  : 1 ;
        volatile uint32_t c0_dma_en                        : 1 ;
        volatile uint32_t c0_dma_trig_cfg                  : 3 ;
        volatile uint32_t unused0                          : 8 ;
    }f;
}pwm_REG_c0_pwm_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_fifo_empty                    : 1 ;
        volatile uint32_t c0_fifo_full                     : 1 ;
        volatile uint32_t c0_fifo_halffull                 : 1 ;
        volatile uint32_t c0_fifo_trig                     : 1 ;
        volatile uint32_t c0_fifo_cnt                      : 3 ;
        volatile uint32_t c0_fifo_wr_st                    : 2 ;
        volatile uint32_t unused1                          : 23;
    }f;
}pwm_REG_c0_pwm_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_pera_th                       : 20;
        volatile uint32_t unused2                          : 12;
    }f;
}pwm_REG_c0_pwm_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_high_th                       : 20;
        volatile uint32_t unused3                          : 12;
    }f;
}pwm_REG_c0_pwm_ctrl3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_dzone_th                      : 20;
        volatile uint32_t unused4                          : 12;
    }f;
}pwm_REG_c0_pwm_ctrl4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_dma_data                      : 20;
        volatile uint32_t unused5                          : 12;
    }f;
}pwm_REG_c0_pwm_ctrl5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_msk                           : 2 ;
        volatile uint32_t c1_haltctrl_en                   : 2 ;
        volatile uint32_t c1_haltctrl_cfg                  : 2 ;
        volatile uint32_t c1_enable_pwm                    : 1 ;
        volatile uint32_t c1_mode                          : 3 ;
        volatile uint32_t c1_inv_en                        : 2 ;
        volatile uint32_t c1_fifo_trig_cfg                 : 3 ;
        volatile uint32_t c1_fifo_clr                      : 1 ;
        volatile uint32_t c1_fifo_empty_en                 : 1 ;
        volatile uint32_t c1_fifo_full_en                  : 1 ;
        volatile uint32_t c1_fifo_halffull_en              : 1 ;
        volatile uint32_t c1_fifo_trig_en                  : 1 ;
        volatile uint32_t c1_dma_en                        : 1 ;
        volatile uint32_t c1_dma_trig_cfg                  : 3 ;
        volatile uint32_t unused6                          : 8 ;
    }f;
}pwm_REG_c1_pwm_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_fifo_empty                    : 1 ;
        volatile uint32_t c1_fifo_full                     : 1 ;
        volatile uint32_t c1_fifo_halffull                 : 1 ;
        volatile uint32_t c1_fifo_trig                     : 1 ;
        volatile uint32_t c1_fifo_cnt                      : 3 ;
        volatile uint32_t c1_fifo_wr_st                    : 2 ;
        volatile uint32_t unused7                          : 23;
    }f;
}pwm_REG_c1_pwm_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_pera_th                       : 20;
        volatile uint32_t unused8                          : 12;
    }f;
}pwm_REG_c1_pwm_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_high_th                       : 20;
        volatile uint32_t unused9                          : 12;
    }f;
}pwm_REG_c1_pwm_ctrl3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_dzone_th                      : 20;
        volatile uint32_t unused10                         : 12;
    }f;
}pwm_REG_c1_pwm_ctrl4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_dma_data                      : 20;
        volatile uint32_t unused11                         : 12;
    }f;
}pwm_REG_c1_pwm_ctrl5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_msk                           : 2 ;
        volatile uint32_t c2_haltctrl_en                   : 2 ;
        volatile uint32_t c2_haltctrl_cfg                  : 2 ;
        volatile uint32_t c2_enable_pwm                    : 1 ;
        volatile uint32_t c2_mode                          : 3 ;
        volatile uint32_t c2_inv_en                        : 2 ;
        volatile uint32_t c2_fifo_trig_cfg                 : 3 ;
        volatile uint32_t c2_fifo_clr                      : 1 ;
        volatile uint32_t c2_fifo_empty_en                 : 1 ;
        volatile uint32_t c2_fifo_full_en                  : 1 ;
        volatile uint32_t c2_fifo_halffull_en              : 1 ;
        volatile uint32_t c2_fifo_trig_en                  : 1 ;
        volatile uint32_t c2_dma_en                        : 1 ;
        volatile uint32_t c2_dma_trig_cfg                  : 3 ;
        volatile uint32_t unused12                         : 8 ;
    }f;
}pwm_REG_c2_pwm_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_fifo_empty                    : 1 ;
        volatile uint32_t c2_fifo_full                     : 1 ;
        volatile uint32_t c2_fifo_halffull                 : 1 ;
        volatile uint32_t c2_fifo_trig                     : 1 ;
        volatile uint32_t c2_fifo_cnt                      : 3 ;
        volatile uint32_t c2_fifo_wr_st                    : 2 ;
        volatile uint32_t unused13                         : 23;
    }f;
}pwm_REG_c2_pwm_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_pera_th                       : 20;
        volatile uint32_t unused14                         : 12;
    }f;
}pwm_REG_c2_pwm_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_high_th                       : 20;
        volatile uint32_t unused15                         : 12;
    }f;
}pwm_REG_c2_pwm_ctrl3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_dzone_th                      : 20;
        volatile uint32_t unused16                         : 12;
    }f;
}pwm_REG_c2_pwm_ctrl4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_dma_data                      : 20;
        volatile uint32_t unused17                         : 12;
    }f;
}pwm_REG_c2_pwm_ctrl5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_p_cap_0_p_en                  : 1 ;
        volatile uint32_t c0_p_cap_0_n_en                  : 1 ;
        volatile uint32_t c0_p_cap_1_p_en                  : 1 ;
        volatile uint32_t c0_p_cap_1_n_en                  : 1 ;
        volatile uint32_t unused18                         : 28;
    }f;
}pwm_REG_c0_cap_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c0_cap_data                      : 32;
    }f;
}pwm_REG_c0_cap_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_p_cap_0_p_en                  : 1 ;
        volatile uint32_t c1_p_cap_0_n_en                  : 1 ;
        volatile uint32_t c1_p_cap_1_p_en                  : 1 ;
        volatile uint32_t c1_p_cap_1_n_en                  : 1 ;
        volatile uint32_t unused19                         : 28;
    }f;
}pwm_REG_c1_cap_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c1_cap_data                      : 32;
    }f;
}pwm_REG_c1_cap_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_p_cap_0_p_en                  : 1 ;
        volatile uint32_t c2_p_cap_0_n_en                  : 1 ;
        volatile uint32_t c2_p_cap_1_p_en                  : 1 ;
        volatile uint32_t c2_p_cap_1_n_en                  : 1 ;
        volatile uint32_t unused20                         : 28;
    }f;
}pwm_REG_c2_cap_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t c2_cap_data                      : 32;
    }f;
}pwm_REG_c2_cap_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cap_cnt_en                       : 1 ;
        volatile uint32_t unused21                         : 31;
    }f;
}pwm_REG_cap_cnt_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cap_cnt                          : 32;
    }f;
}pwm_REG_cap_cnt;

typedef struct
{
volatile pwm_REG_c0_pwm_ctrl0                                        c0_pwm_ctrl0                  ; //0x0
volatile pwm_REG_c0_pwm_ctrl1                                        c0_pwm_ctrl1                  ; //0x4
volatile uint32_t                                                    null_reg0                     ; //0x8
volatile uint32_t                                                    null_reg1                     ; //0xC
volatile pwm_REG_c0_pwm_ctrl2                                        c0_pwm_ctrl2                  ; //0x10
volatile pwm_REG_c0_pwm_ctrl3                                        c0_pwm_ctrl3                  ; //0x14
volatile pwm_REG_c0_pwm_ctrl4                                        c0_pwm_ctrl4                  ; //0x18
volatile pwm_REG_c0_pwm_ctrl5                                        c0_pwm_ctrl5                  ; //0x1C
volatile pwm_REG_c1_pwm_ctrl0                                        c1_pwm_ctrl0                  ; //0x20
volatile pwm_REG_c1_pwm_ctrl1                                        c1_pwm_ctrl1                  ; //0x24
volatile uint32_t                                                    null_reg2                     ; //0x28
volatile uint32_t                                                    null_reg3                     ; //0x2C
volatile pwm_REG_c1_pwm_ctrl2                                        c1_pwm_ctrl2                  ; //0x30
volatile pwm_REG_c1_pwm_ctrl3                                        c1_pwm_ctrl3                  ; //0x34
volatile pwm_REG_c1_pwm_ctrl4                                        c1_pwm_ctrl4                  ; //0x38
volatile pwm_REG_c1_pwm_ctrl5                                        c1_pwm_ctrl5                  ; //0x3C
volatile pwm_REG_c2_pwm_ctrl0                                        c2_pwm_ctrl0                  ; //0x40
volatile pwm_REG_c2_pwm_ctrl1                                        c2_pwm_ctrl1                  ; //0x44
volatile uint32_t                                                    null_reg4                     ; //0x48
volatile uint32_t                                                    null_reg5                     ; //0x4C
volatile pwm_REG_c2_pwm_ctrl2                                        c2_pwm_ctrl2                  ; //0x50
volatile pwm_REG_c2_pwm_ctrl3                                        c2_pwm_ctrl3                  ; //0x54
volatile pwm_REG_c2_pwm_ctrl4                                        c2_pwm_ctrl4                  ; //0x58
volatile pwm_REG_c2_pwm_ctrl5                                        c2_pwm_ctrl5                  ; //0x5C
volatile pwm_REG_c0_cap_ctrl0                                        c0_cap_ctrl0                  ; //0x60
volatile pwm_REG_c0_cap_ctrl1                                        c0_cap_ctrl1                  ; //0x64
volatile uint32_t                                                    null_reg6                     ; //0x68
volatile uint32_t                                                    null_reg7                     ; //0x6C
volatile pwm_REG_c1_cap_ctrl0                                        c1_cap_ctrl0                  ; //0x70
volatile pwm_REG_c1_cap_ctrl1                                        c1_cap_ctrl1                  ; //0x74
volatile uint32_t                                                    null_reg8                     ; //0x78
volatile uint32_t                                                    null_reg9                     ; //0x7C
volatile pwm_REG_c2_cap_ctrl0                                        c2_cap_ctrl0                  ; //0x80
volatile pwm_REG_c2_cap_ctrl1                                        c2_cap_ctrl1                  ; //0x84
volatile uint32_t                                                    null_reg10                    ; //0x88
volatile uint32_t                                                    null_reg11                    ; //0x8C
volatile pwm_REG_cap_cnt_en                                          cap_cnt_en                    ; //0x90
volatile pwm_REG_cap_cnt                                             cap_cnt                       ; //0x94
}pwm_REG_H;

#endif //pwm_REG__H
