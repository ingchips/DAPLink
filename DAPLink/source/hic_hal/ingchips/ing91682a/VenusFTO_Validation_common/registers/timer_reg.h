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
// File         : timer_reg.h
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
#ifndef TIMER_REG__H
#define TIMER_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 8 ;
        volatile uint32_t ID                               : 20;
    }f;
}TIMER_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NumCh                            : 3 ;
        volatile uint32_t unused0                          : 29;
    }f;
}TIMER_REG_Cfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch0Int0En                        : 1 ;
        volatile uint32_t Ch0Int1En                        : 1 ;
        volatile uint32_t Ch0Int2En                        : 1 ;
        volatile uint32_t Ch0Int3En                        : 1 ;
        volatile uint32_t Ch1Int0En                        : 1 ;
        volatile uint32_t Ch1Int1En                        : 1 ;
        volatile uint32_t Ch1Int2En                        : 1 ;
        volatile uint32_t Ch1Int3En                        : 1 ;
        volatile uint32_t Ch2Int0En                        : 1 ;
        volatile uint32_t Ch2Int1En                        : 1 ;
        volatile uint32_t Ch2Int2En                        : 1 ;
        volatile uint32_t Ch2Int3En                        : 1 ;
        volatile uint32_t Ch3Int0En                        : 1 ;
        volatile uint32_t Ch3Int1En                        : 1 ;
        volatile uint32_t Ch3Int2En                        : 1 ;
        volatile uint32_t Ch3Int3En                        : 1 ;
        volatile uint32_t unused1                          : 16;
    }f;
}TIMER_REG_IntEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch0Int0                          : 1 ;
        volatile uint32_t Ch0Int1                          : 1 ;
        volatile uint32_t Ch0Int2                          : 1 ;
        volatile uint32_t Ch0Int3                          : 1 ;
        volatile uint32_t Ch1Int0                          : 1 ;
        volatile uint32_t Ch1Int1                          : 1 ;
        volatile uint32_t Ch1Int2                          : 1 ;
        volatile uint32_t Ch1Int3                          : 1 ;
        volatile uint32_t Ch2Int0                          : 1 ;
        volatile uint32_t Ch2Int1                          : 1 ;
        volatile uint32_t Ch2Int2                          : 1 ;
        volatile uint32_t Ch2Int3                          : 1 ;
        volatile uint32_t Ch3Int0                          : 1 ;
        volatile uint32_t Ch3Int1                          : 1 ;
        volatile uint32_t Ch3Int2                          : 1 ;
        volatile uint32_t Ch3Int3                          : 1 ;
        volatile uint32_t unused2                          : 16;
    }f;
}TIMER_REG_IntSt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch0TMR0En                        : 1 ;
        volatile uint32_t Ch0TMR1En                        : 1 ;
        volatile uint32_t Ch0TMR2En                        : 1 ;
        volatile uint32_t Ch0TMR3En                        : 1 ;
        volatile uint32_t Ch1TMR0En                        : 1 ;
        volatile uint32_t Ch1TMR1En                        : 1 ;
        volatile uint32_t Ch1TMR2En                        : 1 ;
        volatile uint32_t Ch1TMR3En                        : 1 ;
        volatile uint32_t Ch2TMR0En                        : 1 ;
        volatile uint32_t Ch2TMR1En                        : 1 ;
        volatile uint32_t Ch2TMR2En                        : 1 ;
        volatile uint32_t Ch2TMR3En                        : 1 ;
        volatile uint32_t Ch3TMR0En                        : 1 ;
        volatile uint32_t Ch3TMR1En                        : 1 ;
        volatile uint32_t Ch3TMR2En                        : 1 ;
        volatile uint32_t Ch3TMR3En                        : 1 ;
        volatile uint32_t unused3                          : 16;
    }f;
}TIMER_REG_ChEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChMode                           : 3 ;
        volatile uint32_t ChClk                            : 1 ;
        volatile uint32_t PWMPark                          : 1 ;
        volatile uint32_t unused4                          : 27;
    }f;
}TIMER_REG_Ch0Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch0Reload                        : 32;
    }f;
}TIMER_REG_Ch0Reload;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Counter                          : 32;
    }f;
}TIMER_REG_Ch0Cntr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChMode                           : 3 ;
        volatile uint32_t ChClk                            : 1 ;
        volatile uint32_t PWMPark                          : 1 ;
        volatile uint32_t unused5                          : 27;
    }f;
}TIMER_REG_Ch1Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch1Reload                        : 32;
    }f;
}TIMER_REG_Ch1Reload;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Counter                          : 32;
    }f;
}TIMER_REG_Ch1Cntr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChMode                           : 3 ;
        volatile uint32_t ChClk                            : 1 ;
        volatile uint32_t PWMPark                          : 1 ;
        volatile uint32_t unused6                          : 27;
    }f;
}TIMER_REG_Ch2Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch2Reload                        : 32;
    }f;
}TIMER_REG_Ch2Reload;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Counter                          : 32;
    }f;
}TIMER_REG_Ch2Cntr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChMode                           : 3 ;
        volatile uint32_t ChClk                            : 1 ;
        volatile uint32_t PWMPark                          : 1 ;
        volatile uint32_t unused7                          : 27;
    }f;
}TIMER_REG_Ch3Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch3Reload                        : 32;
    }f;
}TIMER_REG_Ch3Reload;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Counter                          : 32;
    }f;
}TIMER_REG_Ch3Cntr;

typedef struct
{
volatile TIMER_REG_IdRev                                             IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile TIMER_REG_Cfg                                               Cfg                           ; //0x10
volatile TIMER_REG_IntEn                                             IntEn                         ; //0x14
volatile TIMER_REG_IntSt                                             IntSt                         ; //0x18
volatile TIMER_REG_ChEn                                              ChEn                          ; //0x1C
volatile TIMER_REG_Ch0Ctrl                                           Ch0Ctrl                       ; //0x20
volatile TIMER_REG_Ch0Reload                                         Ch0Reload                     ; //0x24
volatile TIMER_REG_Ch0Cntr                                           Ch0Cntr                       ; //0x28
volatile uint32_t                                                    null_reg3                     ; //0x2C
volatile TIMER_REG_Ch1Ctrl                                           Ch1Ctrl                       ; //0x30
volatile TIMER_REG_Ch1Reload                                         Ch1Reload                     ; //0x34
volatile TIMER_REG_Ch1Cntr                                           Ch1Cntr                       ; //0x38
volatile uint32_t                                                    null_reg4                     ; //0x3C
volatile TIMER_REG_Ch2Ctrl                                           Ch2Ctrl                       ; //0x40
volatile TIMER_REG_Ch2Reload                                         Ch2Reload                     ; //0x44
volatile TIMER_REG_Ch2Cntr                                           Ch2Cntr                       ; //0x48
volatile uint32_t                                                    null_reg5                     ; //0x4C
volatile TIMER_REG_Ch3Ctrl                                           Ch3Ctrl                       ; //0x50
volatile TIMER_REG_Ch3Reload                                         Ch3Reload                     ; //0x54
volatile TIMER_REG_Ch3Cntr                                           Ch3Cntr                       ; //0x58
}TIMER_REG_H;

#endif //TIMER_REG__H
