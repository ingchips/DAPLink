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
// File         : pdm_reg.h
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
#ifndef PDM_REG__H
#define PDM_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_dma_cfg                      : 3 ;
        volatile uint32_t fifo_clr_reg                     : 1 ;
        volatile uint32_t unused0                          : 4 ;
        volatile uint32_t dma_en                           : 1 ;
        volatile uint32_t int_trig_en                      : 1 ;
        volatile uint32_t int_fifo_empty_en                : 1 ;
        volatile uint32_t int_fifo_full_en                 : 1 ;
        volatile uint32_t int_fifo_halffull_en             : 1 ;
        volatile uint32_t int_trig_cfg                     : 3 ;
        volatile uint32_t D_VOL                            : 6 ;
        volatile uint32_t vol_r_en                         : 1 ;
        volatile uint32_t vol_l_en                         : 1 ;
        volatile uint32_t dmic0_i2s_slavemode              : 1 ;
        volatile uint32_t dmic0_i2s_bitwidth               : 1 ;
        volatile uint32_t unused1                          : 2 ;
        volatile uint32_t HPF_EN                           : 1 ;
        volatile uint32_t FAST_MODE                        : 1 ;
        volatile uint32_t DCBYPASS                         : 1 ;
        volatile uint32_t dmic0_sw_rstn                    : 1 ;
    }f;
}PDM_REG_reg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t dmic0_scale                      : 8 ;
        volatile uint32_t dmic0_dcoff1                     : 8 ;
        volatile uint32_t dmic0_dcoff2                     : 8 ;
        volatile uint32_t dmic0_dcoff3                     : 8 ;
    }f;
}PDM_REG_reg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg8                             : 32;
    }f;
}PDM_REG_reg8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t data                             : 32;
    }f;
}PDM_REG_reg9;

typedef struct
{
volatile PDM_REG_reg0                                                reg0                          ; //0x0
volatile PDM_REG_reg1                                                reg1                          ; //0x4
volatile uint32_t                                                    null_reg0                     ; //0x8
volatile uint32_t                                                    null_reg1                     ; //0xC
volatile uint32_t                                                    null_reg2                     ; //0x10
volatile uint32_t                                                    null_reg3                     ; //0x14
volatile uint32_t                                                    null_reg4                     ; //0x18
volatile uint32_t                                                    null_reg5                     ; //0x1C
volatile PDM_REG_reg8                                                reg8                          ; //0x20
volatile PDM_REG_reg9                                                reg9                          ; //0x24
}PDM_REG_H;

#endif //PDM_REG__H
