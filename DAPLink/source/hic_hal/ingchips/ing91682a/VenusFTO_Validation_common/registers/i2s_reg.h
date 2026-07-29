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
// File         : i2s_reg.h
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
#ifndef I2S_REG__H
#define I2S_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TX_EN                            : 1 ;
        volatile uint32_t RX_EN                            : 1 ;
        volatile uint32_t MODE_SEL                         : 2 ;
        volatile uint32_t MONO                             : 1 ;
        volatile uint32_t unused0                          : 27;
    }f;
}I2S_REG_I2S_MODE_CONFIG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t LRDIV                            : 7 ;
        volatile uint32_t unused1                          : 1 ;
        volatile uint32_t BDIV                             : 7 ;
        volatile uint32_t unused2                          : 17;
    }f;
}I2S_REG_I2S_CLK_DIV;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MSS                              : 1 ;
        volatile uint32_t LRCLK_POL                        : 1 ;
        volatile uint32_t TXBCLK_POL                       : 1 ;
        volatile uint32_t RXBCLK_POL                       : 1 ;
        volatile uint32_t TX_IE                            : 1 ;
        volatile uint32_t RX_IE                            : 1 ;
        volatile uint32_t TX_RST_FIFO                      : 1 ;
        volatile uint32_t RX_RST_FIFO                      : 1 ;
        volatile uint32_t DATA_LEN                         : 5 ;
        volatile uint32_t I2S_TX_DMA_EN                    : 1 ;
        volatile uint32_t I2S_RX_DMA_EN                    : 1 ;
        volatile uint32_t unused3                          : 17;
    }f;
}I2S_REG_I2S_CONFIG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t I2S_TX                           : 32;
    }f;
}I2S_REG_I2S_TX;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t I2S_RX                           : 32;
    }f;
}I2S_REG_I2S_RX;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TX_TRIG                          : 4 ;
        volatile uint32_t unused4                          : 4 ;
        volatile uint32_t RX_TRIG                          : 5 ;
        volatile uint32_t unused5                          : 19;
    }f;
}I2S_REG_I2S_TRIG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TX_INT                           : 1 ;
        volatile uint32_t RX_INT                           : 1 ;
        volatile uint32_t unused6                          : 30;
    }f;
}I2S_REG_I2S_STATUS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TX_FIFO_CNT                      : 5 ;
        volatile uint32_t unused7                          : 3 ;
        volatile uint32_t RX_FIFO_CNT                      : 5 ;
        volatile uint32_t unused8                          : 19;
    }f;
}I2S_REG_I2S_FIFO_STATUS;

typedef struct
{
volatile I2S_REG_I2S_MODE_CONFIG                                     I2S_MODE_CONFIG               ; //0x0
volatile I2S_REG_I2S_CLK_DIV                                         I2S_CLK_DIV                   ; //0x4
volatile I2S_REG_I2S_CONFIG                                          I2S_CONFIG                    ; //0x8
volatile I2S_REG_I2S_TX                                              I2S_TX                        ; //0xC
volatile I2S_REG_I2S_RX                                              I2S_RX                        ; //0x10
volatile I2S_REG_I2S_TRIG                                            I2S_TRIG                      ; //0x14
volatile I2S_REG_I2S_STATUS                                          I2S_STATUS                    ; //0x18
volatile I2S_REG_I2S_FIFO_STATUS                                     I2S_FIFO_STATUS               ; //0x1C
}I2S_REG_H;

#endif //I2S_REG__H
