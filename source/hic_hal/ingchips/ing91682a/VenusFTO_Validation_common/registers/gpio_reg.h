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
// File         : gpio_reg.h
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
#ifndef GPIO_REG__H
#define GPIO_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 4 ;
        volatile uint32_t ID                               : 24;
    }f;
}GPIO_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChannelNum                       : 6 ;
        volatile uint32_t unused0                          : 23;
        volatile uint32_t Debounce                         : 1 ;
        volatile uint32_t Intr                             : 1 ;
        volatile uint32_t Pull                             : 1 ;
    }f;
}GPIO_REG_Cfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DataIn                           : 21;
        volatile uint32_t unused1                          : 11;
    }f;
}GPIO_REG_DataIn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DataOut                          : 21;
        volatile uint32_t unused2                          : 11;
    }f;
}GPIO_REG_DataOut;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChannelDir                       : 21;
        volatile uint32_t unused3                          : 11;
    }f;
}GPIO_REG_ChannelDir;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DoutClear                        : 21;
        volatile uint32_t unused4                          : 11;
    }f;
}GPIO_REG_DoutClear;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DoutSet                          : 21;
        volatile uint32_t unused5                          : 11;
    }f;
}GPIO_REG_DoutSet;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t gpio_ie_reg                      : 21;
        volatile uint32_t unused6                          : 11;
    }f;
}GPIO_REG_IOIE;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t IntEn                            : 21;
        volatile uint32_t unused7                          : 11;
    }f;
}GPIO_REG_IntrEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch0IntrM                         : 3 ;
        volatile uint32_t unused8                          : 1 ;
        volatile uint32_t Ch1IntrM                         : 3 ;
        volatile uint32_t unused9                          : 1 ;
        volatile uint32_t Ch2IntrM                         : 3 ;
        volatile uint32_t unused10                         : 1 ;
        volatile uint32_t Ch3IntrM                         : 3 ;
        volatile uint32_t unused11                         : 1 ;
        volatile uint32_t Ch4IntrM                         : 3 ;
        volatile uint32_t unused12                         : 1 ;
        volatile uint32_t Ch5IntrM                         : 3 ;
        volatile uint32_t unused13                         : 1 ;
        volatile uint32_t Ch6IntrM                         : 3 ;
        volatile uint32_t unused14                         : 1 ;
        volatile uint32_t Ch7IntrM                         : 3 ;
        volatile uint32_t unused15                         : 1 ;
    }f;
}GPIO_REG_IntrMode0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch8IntrM                         : 3 ;
        volatile uint32_t unused16                         : 1 ;
        volatile uint32_t Ch9IntrM                         : 3 ;
        volatile uint32_t unused17                         : 1 ;
        volatile uint32_t Ch10IntrM                        : 3 ;
        volatile uint32_t unused18                         : 1 ;
        volatile uint32_t Ch11IntrM                        : 3 ;
        volatile uint32_t unused19                         : 1 ;
        volatile uint32_t Ch12IntrM                        : 3 ;
        volatile uint32_t unused20                         : 1 ;
        volatile uint32_t Ch13IntrM                        : 3 ;
        volatile uint32_t unused21                         : 1 ;
        volatile uint32_t Ch14IntrM                        : 3 ;
        volatile uint32_t unused22                         : 1 ;
        volatile uint32_t Ch15IntrM                        : 3 ;
        volatile uint32_t unused23                         : 1 ;
    }f;
}GPIO_REG_IntrMode1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Ch16IntrM                        : 3 ;
        volatile uint32_t unused24                         : 1 ;
        volatile uint32_t Ch17IntrM                        : 3 ;
        volatile uint32_t unused25                         : 1 ;
        volatile uint32_t Ch18IntrM                        : 3 ;
        volatile uint32_t unused26                         : 1 ;
        volatile uint32_t Ch19IntrM                        : 3 ;
        volatile uint32_t unused27                         : 1 ;
        volatile uint32_t Ch20IntrM                        : 3 ;
        volatile uint32_t unused28                         : 13;
    }f;
}GPIO_REG_IntrMode2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t IntrStatus                       : 21;
        volatile uint32_t unused29                         : 11;
    }f;
}GPIO_REG_IntrStatus;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DeBounceEn                       : 21;
        volatile uint32_t unused30                         : 11;
    }f;
}GPIO_REG_DeBounceEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DBPreScale                       : 8 ;
        volatile uint32_t unused31                         : 23;
        volatile uint32_t DBClkSel                         : 1 ;
    }f;
}GPIO_REG_DeBounceCtrl;

typedef struct
{
volatile GPIO_REG_IdRev                                              IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile GPIO_REG_Cfg                                                Cfg                           ; //0x10
volatile uint32_t                                                    null_reg3                     ; //0x14
volatile uint32_t                                                    null_reg4                     ; //0x18
volatile uint32_t                                                    null_reg5                     ; //0x1C
volatile GPIO_REG_DataIn                                             DataIn                        ; //0x20
volatile GPIO_REG_DataOut                                            DataOut                       ; //0x24
volatile GPIO_REG_ChannelDir                                         ChannelDir                    ; //0x28
volatile GPIO_REG_DoutClear                                          DoutClear                     ; //0x2C
volatile GPIO_REG_DoutSet                                            DoutSet                       ; //0x30
volatile GPIO_REG_IOIE                                               IOIE                          ; //0x34
volatile uint32_t                                                    null_reg6                     ; //0x38
volatile uint32_t                                                    null_reg7                     ; //0x3C
volatile uint32_t                                                    null_reg8                     ; //0x40
volatile uint32_t                                                    null_reg9                     ; //0x44
volatile uint32_t                                                    null_reg10                    ; //0x48
volatile uint32_t                                                    null_reg11                    ; //0x4C
volatile GPIO_REG_IntrEn                                             IntrEn                        ; //0x50
volatile GPIO_REG_IntrMode0                                          IntrMode0                     ; //0x54
volatile GPIO_REG_IntrMode1                                          IntrMode1                     ; //0x58
volatile GPIO_REG_IntrMode2                                          IntrMode2                     ; //0x5C
volatile uint32_t                                                    null_reg12                    ; //0x60
volatile GPIO_REG_IntrStatus                                         IntrStatus                    ; //0x64
volatile uint32_t                                                    null_reg13                    ; //0x68
volatile uint32_t                                                    null_reg14                    ; //0x6C
volatile GPIO_REG_DeBounceEn                                         DeBounceEn                    ; //0x70
volatile GPIO_REG_DeBounceCtrl                                       DeBounceCtrl                  ; //0x74
}GPIO_REG_H;

#endif //GPIO_REG__H
