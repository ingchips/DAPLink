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
// File         : wdt_reg.h
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
#ifndef WDT_REG__H
#define WDT_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 8 ;
        volatile uint32_t ID                               : 20;
    }f;
}WDT_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t En                               : 1 ;
        volatile uint32_t ClkSel                           : 1 ;
        volatile uint32_t IntEn                            : 1 ;
        volatile uint32_t RstEn                            : 1 ;
        volatile uint32_t IntTime                          : 4 ;
        volatile uint32_t RstTime                          : 3 ;
        volatile uint32_t unused0                          : 21;
    }f;
}WDT_REG_Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Restart                          : 16;
        volatile uint32_t unused1                          : 16;
    }f;
}WDT_REG_Restart;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t WEn                              : 16;
        volatile uint32_t unused2                          : 16;
    }f;
}WDT_REG_WrEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t IntExpired                       : 1 ;
        volatile uint32_t unused3                          : 31;
    }f;
}WDT_REG_St;

typedef struct
{
volatile WDT_REG_IdRev                                               IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile WDT_REG_Ctrl                                                Ctrl                          ; //0x10
volatile WDT_REG_Restart                                             Restart                       ; //0x14
volatile WDT_REG_WrEn                                                WrEn                          ; //0x18
volatile WDT_REG_St                                                  St                            ; //0x1C
}WDT_REG_H;

#endif //WDT_REG__H
