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
// File         : i2c_reg.h
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
#ifndef I2C_REG__H
#define I2C_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 4 ;
        volatile uint32_t ID                               : 24;
    }f;
}I2C_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FIFOSize                         : 2 ;
        volatile uint32_t unused0                          : 30;
    }f;
}I2C_REG_Cfg;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FIFOEmpty                        : 1 ;
        volatile uint32_t FIFOFull                         : 1 ;
        volatile uint32_t FIFOHalf                         : 1 ;
        volatile uint32_t AddrHit                          : 1 ;
        volatile uint32_t ArbLose                          : 1 ;
        volatile uint32_t Stop                             : 1 ;
        volatile uint32_t Start                            : 1 ;
        volatile uint32_t ByteTrans                        : 1 ;
        volatile uint32_t ByteRecv                         : 1 ;
        volatile uint32_t Cmpl                             : 1 ;
        volatile uint32_t unused1                          : 22;
    }f;
}I2C_REG_IntEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FIFOEmpty                        : 1 ;
        volatile uint32_t FIFOFull                         : 1 ;
        volatile uint32_t FIFOHalf                         : 1 ;
        volatile uint32_t AddrHit                          : 1 ;
        volatile uint32_t ArbLose                          : 1 ;
        volatile uint32_t Stop                             : 1 ;
        volatile uint32_t Start                            : 1 ;
        volatile uint32_t ByteTrans                        : 1 ;
        volatile uint32_t ByteRecv                         : 1 ;
        volatile uint32_t Cmpl                             : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t BusBusy                          : 1 ;
        volatile uint32_t GenCall                          : 1 ;
        volatile uint32_t LineSCL                          : 1 ;
        volatile uint32_t LineSDA                          : 1 ;
        volatile uint32_t unused2                          : 17;
    }f;
}I2C_REG_Status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Addr                             : 10;
        volatile uint32_t unused3                          : 22;
    }f;
}I2C_REG_Addr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Data                             : 8 ;
        volatile uint32_t unused4                          : 24;
    }f;
}I2C_REG_Data;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DataCnt                          : 8 ;
        volatile uint32_t Dir                              : 1 ;
        volatile uint32_t Phase_stop                       : 1 ;
        volatile uint32_t Phase_data                       : 1 ;
        volatile uint32_t Phase_addr                       : 1 ;
        volatile uint32_t Phase_start                      : 1 ;
        volatile uint32_t unused5                          : 19;
    }f;
}I2C_REG_Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CMD                              : 3 ;
        volatile uint32_t unused6                          : 29;
    }f;
}I2C_REG_Cmd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t IICEn                            : 1 ;
        volatile uint32_t Addressing                       : 1 ;
        volatile uint32_t Master                           : 1 ;
        volatile uint32_t DMAEn                            : 1 ;
        volatile uint32_t T_SCLHi                          : 9 ;
        volatile uint32_t T_SCLRatio                       : 1 ;
        volatile uint32_t unused7                          : 2 ;
        volatile uint32_t T_HDDAT                          : 5 ;
        volatile uint32_t T_SP                             : 3 ;
        volatile uint32_t T_SUDAT                          : 5 ;
        volatile uint32_t unused8                          : 3 ;
    }f;
}I2C_REG_Setup;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TPM                              : 5 ;
        volatile uint32_t unused9                          : 27;
    }f;
}I2C_REG_TPM;

typedef struct
{
volatile I2C_REG_IdRev                                               IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile I2C_REG_Cfg                                                 Cfg                           ; //0x10
volatile I2C_REG_IntEn                                               IntEn                         ; //0x14
volatile I2C_REG_Status                                              Status                        ; //0x18
volatile I2C_REG_Addr                                                Addr                          ; //0x1C
volatile I2C_REG_Data                                                Data                          ; //0x20
volatile I2C_REG_Ctrl                                                Ctrl                          ; //0x24
volatile I2C_REG_Cmd                                                 Cmd                           ; //0x28
volatile I2C_REG_Setup                                               Setup                         ; //0x2C
volatile I2C_REG_TPM                                                 TPM                           ; //0x30
}I2C_REG_H;

#endif //I2C_REG__H
