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
// File         : dma_reg.h
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
#ifndef DMA_REG__H
#define DMA_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t REV_MINOR                        : 4 ;
        volatile uint32_t REV_MAJOR                        : 4 ;
        volatile uint32_t PRODUCT_ID                       : 24;
    }f;
}DMA_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused0                          : 31;
        volatile uint32_t COMP                             : 1 ;
    }f;
}DMA_REG_IdMisc;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChannelNum                       : 4 ;
        volatile uint32_t FIFODepth                        : 6 ;
        volatile uint32_t ReqNum                           : 5 ;
        volatile uint32_t BusNum                           : 1 ;
        volatile uint32_t CoreNum                          : 1 ;
        volatile uint32_t AddrWidth                        : 7 ;
        volatile uint32_t DataWidth                        : 2 ;
        volatile uint32_t unused1                          : 4 ;
        volatile uint32_t ReqSync                          : 1 ;
        volatile uint32_t ChainXfr                         : 1 ;
    }f;
}DMA_REG_DMACfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Reset                            : 1 ;
        volatile uint32_t unused2                          : 31;
    }f;
}DMA_REG_DMACtrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChAbort                          : 8 ;
        volatile uint32_t unused3                          : 24;
    }f;
}DMA_REG_ChAbort;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Error                            : 8 ;
        volatile uint32_t Abort                            : 8 ;
        volatile uint32_t TC                               : 8 ;
        volatile uint32_t unused4                          : 8 ;
    }f;
}DMA_REG_IntStatus;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChEN                             : 8 ;
        volatile uint32_t unused5                          : 24;
    }f;
}DMA_REG_ChEN;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused6                          : 2 ;
    }f;
}DMA_REG_Ch0Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch0TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch0SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch0DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused7                          : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch0LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused8                          : 2 ;
    }f;
}DMA_REG_Ch1Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch1TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch1SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch1DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused9                          : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch1LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused10                         : 2 ;
    }f;
}DMA_REG_Ch2Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch2TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch2SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch2DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused11                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch2LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused12                         : 2 ;
    }f;
}DMA_REG_Ch3Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch3TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch3SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch3DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused13                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch3LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused14                         : 2 ;
    }f;
}DMA_REG_Ch4Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch4TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch4SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch4DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused15                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch4LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused16                         : 2 ;
    }f;
}DMA_REG_Ch5Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch5TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch5SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch5DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused17                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch5LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused18                         : 2 ;
    }f;
}DMA_REG_Ch6Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch6TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch6SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch6DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused19                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch6LLPointer;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Enable                           : 1 ;
        volatile uint32_t IntTCMask                        : 1 ;
        volatile uint32_t IntErrMask                       : 1 ;
        volatile uint32_t IntAbtMask                       : 1 ;
        volatile uint32_t DstReqSel                        : 4 ;
        volatile uint32_t SrcReqSel                        : 4 ;
        volatile uint32_t DstAddrCtrl                      : 2 ;
        volatile uint32_t SrcAddrCtrl                      : 2 ;
        volatile uint32_t DstMode                          : 1 ;
        volatile uint32_t SrcMode                          : 1 ;
        volatile uint32_t DstWidth                         : 3 ;
        volatile uint32_t SrcWidth                         : 3 ;
        volatile uint32_t SrcBurstSize                     : 4 ;
        volatile uint32_t en_int_chain_single              : 1 ;
        volatile uint32_t Priority                         : 1 ;
        volatile uint32_t unused20                         : 2 ;
    }f;
}DMA_REG_Ch7Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TranSize                         : 32;
    }f;
}DMA_REG_Ch7TranSize;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SrcAddr                          : 32;
    }f;
}DMA_REG_Ch7SrcAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DstAddr                          : 32;
    }f;
}DMA_REG_Ch7DstAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused21                         : 3 ;
        volatile uint32_t LLPointer                        : 29;
    }f;
}DMA_REG_Ch7LLPointer;

typedef struct
{
volatile DMA_REG_IdRev                                               IdRev                         ; //0x0
volatile DMA_REG_IdMisc                                              IdMisc                        ; //0x4
volatile uint32_t                                                    null_reg0                     ; //0x8
volatile uint32_t                                                    null_reg1                     ; //0xC
volatile DMA_REG_DMACfg                                              DMACfg                        ; //0x10
volatile uint32_t                                                    null_reg2                     ; //0x14
volatile uint32_t                                                    null_reg3                     ; //0x18
volatile uint32_t                                                    null_reg4                     ; //0x1C
volatile DMA_REG_DMACtrl                                             DMACtrl                       ; //0x20
volatile DMA_REG_ChAbort                                             ChAbort                       ; //0x24
volatile uint32_t                                                    null_reg5                     ; //0x28
volatile uint32_t                                                    null_reg6                     ; //0x2C
volatile DMA_REG_IntStatus                                           IntStatus                     ; //0x30
volatile DMA_REG_ChEN                                                ChEN                          ; //0x34
volatile uint32_t                                                    null_reg7                     ; //0x38
volatile uint32_t                                                    null_reg8                     ; //0x3C
volatile DMA_REG_Ch0Ctrl                                             Ch0Ctrl                       ; //0x40
volatile DMA_REG_Ch0TranSize                                         Ch0TranSize                   ; //0x44
volatile DMA_REG_Ch0SrcAddr                                          Ch0SrcAddr                    ; //0x48
volatile uint32_t                                                    null_reg9                     ; //0x4C
volatile DMA_REG_Ch0DstAddr                                          Ch0DstAddr                    ; //0x50
volatile uint32_t                                                    null_reg10                    ; //0x54
volatile DMA_REG_Ch0LLPointer                                        Ch0LLPointer                  ; //0x58
volatile uint32_t                                                    null_reg11                    ; //0x5C
volatile DMA_REG_Ch1Ctrl                                             Ch1Ctrl                       ; //0x60
volatile DMA_REG_Ch1TranSize                                         Ch1TranSize                   ; //0x64
volatile DMA_REG_Ch1SrcAddr                                          Ch1SrcAddr                    ; //0x68
volatile uint32_t                                                    null_reg12                    ; //0x6C
volatile DMA_REG_Ch1DstAddr                                          Ch1DstAddr                    ; //0x70
volatile uint32_t                                                    null_reg13                    ; //0x74
volatile DMA_REG_Ch1LLPointer                                        Ch1LLPointer                  ; //0x78
volatile uint32_t                                                    null_reg14                    ; //0x7C
volatile DMA_REG_Ch2Ctrl                                             Ch2Ctrl                       ; //0x80
volatile DMA_REG_Ch2TranSize                                         Ch2TranSize                   ; //0x84
volatile DMA_REG_Ch2SrcAddr                                          Ch2SrcAddr                    ; //0x88
volatile uint32_t                                                    null_reg15                    ; //0x8C
volatile DMA_REG_Ch2DstAddr                                          Ch2DstAddr                    ; //0x90
volatile uint32_t                                                    null_reg16                    ; //0x94
volatile DMA_REG_Ch2LLPointer                                        Ch2LLPointer                  ; //0x98
volatile uint32_t                                                    null_reg17                    ; //0x9C
volatile DMA_REG_Ch3Ctrl                                             Ch3Ctrl                       ; //0xA0
volatile DMA_REG_Ch3TranSize                                         Ch3TranSize                   ; //0xA4
volatile DMA_REG_Ch3SrcAddr                                          Ch3SrcAddr                    ; //0xA8
volatile uint32_t                                                    null_reg18                    ; //0xAC
volatile DMA_REG_Ch3DstAddr                                          Ch3DstAddr                    ; //0xB0
volatile uint32_t                                                    null_reg19                    ; //0xB4
volatile DMA_REG_Ch3LLPointer                                        Ch3LLPointer                  ; //0xB8
volatile uint32_t                                                    null_reg20                    ; //0xBC
volatile DMA_REG_Ch4Ctrl                                             Ch4Ctrl                       ; //0xC0
volatile DMA_REG_Ch4TranSize                                         Ch4TranSize                   ; //0xC4
volatile DMA_REG_Ch4SrcAddr                                          Ch4SrcAddr                    ; //0xC8
volatile uint32_t                                                    null_reg21                    ; //0xCC
volatile DMA_REG_Ch4DstAddr                                          Ch4DstAddr                    ; //0xD0
volatile uint32_t                                                    null_reg22                    ; //0xD4
volatile DMA_REG_Ch4LLPointer                                        Ch4LLPointer                  ; //0xD8
volatile uint32_t                                                    null_reg23                    ; //0xDC
volatile DMA_REG_Ch5Ctrl                                             Ch5Ctrl                       ; //0xE0
volatile DMA_REG_Ch5TranSize                                         Ch5TranSize                   ; //0xE4
volatile DMA_REG_Ch5SrcAddr                                          Ch5SrcAddr                    ; //0xE8
volatile uint32_t                                                    null_reg24                    ; //0xEC
volatile DMA_REG_Ch5DstAddr                                          Ch5DstAddr                    ; //0xF0
volatile uint32_t                                                    null_reg25                    ; //0xF4
volatile DMA_REG_Ch5LLPointer                                        Ch5LLPointer                  ; //0xF8
volatile uint32_t                                                    null_reg26                    ; //0xFC
volatile DMA_REG_Ch6Ctrl                                             Ch6Ctrl                       ; //0x100
volatile DMA_REG_Ch6TranSize                                         Ch6TranSize                   ; //0x104
volatile DMA_REG_Ch6SrcAddr                                          Ch6SrcAddr                    ; //0x108
volatile uint32_t                                                    null_reg27                    ; //0x10C
volatile DMA_REG_Ch6DstAddr                                          Ch6DstAddr                    ; //0x110
volatile uint32_t                                                    null_reg28                    ; //0x114
volatile DMA_REG_Ch6LLPointer                                        Ch6LLPointer                  ; //0x118
volatile uint32_t                                                    null_reg29                    ; //0x11C
volatile DMA_REG_Ch7Ctrl                                             Ch7Ctrl                       ; //0x120
volatile DMA_REG_Ch7TranSize                                         Ch7TranSize                   ; //0x124
volatile DMA_REG_Ch7SrcAddr                                          Ch7SrcAddr                    ; //0x128
volatile uint32_t                                                    null_reg30                    ; //0x12C
volatile DMA_REG_Ch7DstAddr                                          Ch7DstAddr                    ; //0x130
volatile uint32_t                                                    null_reg31                    ; //0x134
volatile DMA_REG_Ch7LLPointer                                        Ch7LLPointer                  ; //0x138
}DMA_REG_H;

#endif //DMA_REG__H
