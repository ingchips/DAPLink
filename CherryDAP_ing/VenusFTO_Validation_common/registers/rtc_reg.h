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
// File         : rtc_reg.h
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
#ifndef RTC_REG__H
#define RTC_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 4 ;
        volatile uint32_t ID                               : 24;
    }f;
}RTC_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Sec                              : 6 ;
        volatile uint32_t Min                              : 6 ;
        volatile uint32_t Hour                             : 5 ;
        volatile uint32_t Day                              : 15;
    }f;
}RTC_REG_Cntr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Sec                              : 6 ;
        volatile uint32_t Min                              : 6 ;
        volatile uint32_t Hour                             : 5 ;
        volatile uint32_t unused0                          : 15;
    }f;
}RTC_REG_Alarm;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RTC_En                           : 1 ;
        volatile uint32_t Alarm_Wakeup                     : 1 ;
        volatile uint32_t Alarm_Int                        : 1 ;
        volatile uint32_t Day                              : 1 ;
        volatile uint32_t Hour                             : 1 ;
        volatile uint32_t Min                              : 1 ;
        volatile uint32_t Sec                              : 1 ;
        volatile uint32_t Hsec                             : 1 ;
        volatile uint32_t Freq_Test_En                     : 1 ;
        volatile uint32_t unused1                          : 23;
    }f;
}RTC_REG_Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused2                          : 2 ;
        volatile uint32_t Alarm_Int                        : 1 ;
        volatile uint32_t Day                              : 1 ;
        volatile uint32_t Hour                             : 1 ;
        volatile uint32_t Min                              : 1 ;
        volatile uint32_t Sec                              : 1 ;
        volatile uint32_t Hsec                             : 1 ;
        volatile uint32_t unused3                          : 8 ;
        volatile uint32_t WriteDone                        : 1 ;
        volatile uint32_t unused4                          : 15;
    }f;
}RTC_REG_St;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Sec_Trim                         : 5 ;
        volatile uint32_t unused5                          : 2 ;
        volatile uint32_t Sec_Sign                         : 1 ;
        volatile uint32_t Min_Trim                         : 5 ;
        volatile uint32_t unused6                          : 2 ;
        volatile uint32_t Min_Sign                         : 1 ;
        volatile uint32_t Hour_Trim                        : 5 ;
        volatile uint32_t unused7                          : 2 ;
        volatile uint32_t Hour_Sign                        : 1 ;
        volatile uint32_t Day_Trim                         : 5 ;
        volatile uint32_t unused8                          : 2 ;
        volatile uint32_t Day_Sign                         : 1 ;
    }f;
}RTC_REG_Trim;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sec_cnt_cfg                      : 14;
        volatile uint32_t unused9                          : 18;
    }f;
}RTC_REG_SEC_CFG;

typedef struct
{
volatile RTC_REG_IdRev                                               IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile RTC_REG_Cntr                                                Cntr                          ; //0x10
volatile RTC_REG_Alarm                                               Alarm                         ; //0x14
volatile RTC_REG_Ctrl                                                Ctrl                          ; //0x18
volatile RTC_REG_St                                                  St                            ; //0x1C
volatile RTC_REG_Trim                                                Trim                          ; //0x20
volatile RTC_REG_SEC_CFG                                             SEC_CFG                       ; //0x24
}RTC_REG_H;

#endif //RTC_REG__H
