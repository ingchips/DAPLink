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
// File         : trng_reg.h
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
#ifndef TRNG_REG__H
#define TRNG_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ena_gen                          : 1 ;
        volatile uint32_t ena_post                         : 1 ;
        volatile uint32_t int_clr                          : 1 ;
        volatile uint32_t int_mask                         : 1 ;
        volatile uint32_t int_mode                         : 1 ;
        volatile uint32_t clk_force_on                     : 1 ;
        volatile uint32_t rsvd0                            : 2 ;
        volatile uint32_t rate_osc                         : 8 ;
        volatile uint32_t rsvd1                            : 16;
    }f;
}TRNG_REG_TRNG_CONFIG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t bit_sel                          : 19;
        volatile uint32_t rsvd2                            : 13;
    }f;
}TRNG_REG_TRNG_BIT_SEL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t data_mode                        : 2 ;
        volatile uint32_t data_val                         : 1 ;
        volatile uint32_t rsvd3                            : 29;
    }f;
}TRNG_REG_TRNG_DATA_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t int_mask                         : 1 ;
        volatile uint32_t int_saw                          : 1 ;
        volatile uint32_t rsvd4                            : 30;
    }f;
}TRNG_REG_TRNG_INT_STATUS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t data                             : 32;
    }f;
}TRNG_REG_TRNG_DATA;

typedef struct
{
volatile TRNG_REG_TRNG_CONFIG                                        TRNG_CONFIG                   ; //0x0
volatile TRNG_REG_TRNG_BIT_SEL                                       TRNG_BIT_SEL                  ; //0x4
volatile TRNG_REG_TRNG_DATA_CTRL                                     TRNG_DATA_CTRL                ; //0x8
volatile TRNG_REG_TRNG_INT_STATUS                                    TRNG_INT_STATUS               ; //0xC
volatile TRNG_REG_TRNG_DATA                                          TRNG_DATA                     ; //0x10
}TRNG_REG_H;

#endif //TRNG_REG__H
