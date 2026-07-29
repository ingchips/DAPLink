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
// File         : qspi_reg.h
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
#ifndef QSPI_REG__H
#define QSPI_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RevMinor                         : 4 ;
        volatile uint32_t RevMajor                         : 8 ;
        volatile uint32_t ID                               : 20;
    }f;
}QSPI_REG_IdRev;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CPHA                             : 1 ;
        volatile uint32_t CPOL                             : 1 ;
        volatile uint32_t SlvMode                          : 1 ;
        volatile uint32_t LSB                              : 1 ;
        volatile uint32_t MOSIBiDir                        : 1 ;
        volatile uint32_t unused0                          : 2 ;
        volatile uint32_t DataMerge                        : 1 ;
        volatile uint32_t DataLen                          : 5 ;
        volatile uint32_t unused1                          : 3 ;
        volatile uint32_t AddrLen                          : 2 ;
        volatile uint32_t unused2                          : 14;
    }f;
}QSPI_REG_TransFmt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CS_I                             : 1 ;
        volatile uint32_t SCLK_I                           : 1 ;
        volatile uint32_t MOSI_I                           : 1 ;
        volatile uint32_t MISO_I                           : 1 ;
        volatile uint32_t WP_I                             : 1 ;
        volatile uint32_t HOLD_I                           : 1 ;
        volatile uint32_t unused3                          : 2 ;
        volatile uint32_t CS_O                             : 1 ;
        volatile uint32_t SCLK_O                           : 1 ;
        volatile uint32_t MOSI_O                           : 1 ;
        volatile uint32_t MISO_O                           : 1 ;
        volatile uint32_t WP_O                             : 1 ;
        volatile uint32_t HOLD_O                           : 1 ;
        volatile uint32_t unused4                          : 2 ;
        volatile uint32_t CS_OE                            : 1 ;
        volatile uint32_t SCLK_OE                          : 1 ;
        volatile uint32_t MOSI_OE                          : 1 ;
        volatile uint32_t MISO_OE                          : 1 ;
        volatile uint32_t WP_OE                            : 1 ;
        volatile uint32_t HOLD_OE                          : 1 ;
        volatile uint32_t unused5                          : 2 ;
        volatile uint32_t DirectIOEn                       : 1 ;
        volatile uint32_t unused6                          : 7 ;
    }f;
}QSPI_REG_DirectIO;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RdTranCnt                        : 9 ;
        volatile uint32_t DummyCnt                         : 2 ;
        volatile uint32_t TokenValue                       : 1 ;
        volatile uint32_t WrTranCnt                        : 9 ;
        volatile uint32_t TokenEn                          : 1 ;
        volatile uint32_t DualQuad                         : 2 ;
        volatile uint32_t TransMode                        : 4 ;
        volatile uint32_t AddrFmt                          : 1 ;
        volatile uint32_t AddrEn                           : 1 ;
        volatile uint32_t CmdEn                            : 1 ;
        volatile uint32_t SlvDataOnly                      : 1 ;
    }f;
}QSPI_REG_TransCtrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CMD                              : 8 ;
        volatile uint32_t unused7                          : 24;
    }f;
}QSPI_REG_Cmd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ADDR                             : 32;
    }f;
}QSPI_REG_Addr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DATA                             : 32;
    }f;
}QSPI_REG_Data;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SPIRST                           : 1 ;
        volatile uint32_t RXFIFORST                        : 1 ;
        volatile uint32_t TXFIFORST                        : 1 ;
        volatile uint32_t RXDMAEN                          : 1 ;
        volatile uint32_t TXDMAEN                          : 1 ;
        volatile uint32_t unused8                          : 3 ;
        volatile uint32_t RXTHRES                          : 8 ;
        volatile uint32_t TXTHRES                          : 8 ;
        volatile uint32_t unused9                          : 8 ;
    }f;
}QSPI_REG_Ctrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SPIActive                        : 1 ;
        volatile uint32_t unused10                         : 7 ;
        volatile uint32_t RXNUMLo                          : 6 ;
        volatile uint32_t RXEMPTY                          : 1 ;
        volatile uint32_t RXFULL                           : 1 ;
        volatile uint32_t TXNUMLo                          : 6 ;
        volatile uint32_t TXEMPTY                          : 1 ;
        volatile uint32_t TXFULL                           : 1 ;
        volatile uint32_t RXNUMHi                          : 2 ;
        volatile uint32_t unused11                         : 2 ;
        volatile uint32_t TXNUMHi                          : 2 ;
        volatile uint32_t unused12                         : 2 ;
    }f;
}QSPI_REG_Status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RXFIFOORIntEn                    : 1 ;
        volatile uint32_t TXFIFOURIntEn                    : 1 ;
        volatile uint32_t RXFIFOIntEn                      : 1 ;
        volatile uint32_t TXFIFOIntEn                      : 1 ;
        volatile uint32_t EndIntEn                         : 1 ;
        volatile uint32_t SlvCmdEn                         : 1 ;
        volatile uint32_t unused13                         : 26;
    }f;
}QSPI_REG_IntrEn;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RXFIFOORInt                      : 1 ;
        volatile uint32_t TXFIFOURInt                      : 1 ;
        volatile uint32_t RXFIFOInt                        : 1 ;
        volatile uint32_t TXFIFOInt                        : 1 ;
        volatile uint32_t EndInt                           : 1 ;
        volatile uint32_t SlvCmdInt                        : 1 ;
        volatile uint32_t unused14                         : 26;
    }f;
}QSPI_REG_IntrSt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SCLK_DIV                         : 8 ;
        volatile uint32_t CSHT                             : 4 ;
        volatile uint32_t CS2SCLK                          : 2 ;
        volatile uint32_t unused15                         : 18;
    }f;
}QSPI_REG_Timing;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MemRdCmd                         : 4 ;
        volatile uint32_t unused16                         : 4 ;
        volatile uint32_t MemCtrlChg                       : 1 ;
        volatile uint32_t unused17                         : 23;
    }f;
}QSPI_REG_MemCtrl;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t USR_Status                       : 16;
        volatile uint32_t Ready                            : 1 ;
        volatile uint32_t OverRun                          : 1 ;
        volatile uint32_t UnderRun                         : 1 ;
        volatile uint32_t unused18                         : 13;
    }f;
}QSPI_REG_SlvSt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RCnt                             : 10;
        volatile uint32_t unused19                         : 6 ;
        volatile uint32_t WCnt                             : 10;
        volatile uint32_t unused20                         : 6 ;
    }f;
}QSPI_REG_SlvDataCnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RxFIFOSize                       : 4 ;
        volatile uint32_t TxFIFOSize                       : 4 ;
        volatile uint32_t DualSPI                          : 1 ;
        volatile uint32_t QuadSPI                          : 1 ;
        volatile uint32_t unused21                         : 1 ;
        volatile uint32_t DirectIO                         : 1 ;
        volatile uint32_t AHBMem                           : 1 ;
        volatile uint32_t EILMMem                          : 1 ;
        volatile uint32_t Slave                            : 1 ;
        volatile uint32_t unused22                         : 17;
    }f;
}QSPI_REG_Config;

typedef struct
{
volatile QSPI_REG_IdRev                                              IdRev                         ; //0x0
volatile uint32_t                                                    null_reg0                     ; //0x4
volatile uint32_t                                                    null_reg1                     ; //0x8
volatile uint32_t                                                    null_reg2                     ; //0xC
volatile QSPI_REG_TransFmt                                           TransFmt                      ; //0x10
volatile QSPI_REG_DirectIO                                           DirectIO                      ; //0x14
volatile uint32_t                                                    null_reg3                     ; //0x18
volatile uint32_t                                                    null_reg4                     ; //0x1C
volatile QSPI_REG_TransCtrl                                          TransCtrl                     ; //0x20
volatile QSPI_REG_Cmd                                                Cmd                           ; //0x24
volatile QSPI_REG_Addr                                               Addr                          ; //0x28
volatile QSPI_REG_Data                                               Data                          ; //0x2C
volatile QSPI_REG_Ctrl                                               Ctrl                          ; //0x30
volatile QSPI_REG_Status                                             Status                        ; //0x34
volatile QSPI_REG_IntrEn                                             IntrEn                        ; //0x38
volatile QSPI_REG_IntrSt                                             IntrSt                        ; //0x3C
volatile QSPI_REG_Timing                                             Timing                        ; //0x40
volatile uint32_t                                                    null_reg5                     ; //0x44
volatile uint32_t                                                    null_reg6                     ; //0x48
volatile uint32_t                                                    null_reg7                     ; //0x4C
volatile QSPI_REG_MemCtrl                                            MemCtrl                       ; //0x50
volatile uint32_t                                                    null_reg8                     ; //0x54
volatile uint32_t                                                    null_reg9                     ; //0x58
volatile uint32_t                                                    null_reg10                    ; //0x5C
volatile QSPI_REG_SlvSt                                              SlvSt                         ; //0x60
volatile QSPI_REG_SlvDataCnt                                         SlvDataCnt                    ; //0x64
volatile uint32_t                                                    null_reg11                    ; //0x68
volatile uint32_t                                                    null_reg12                    ; //0x6C
volatile uint32_t                                                    null_reg13                    ; //0x70
volatile uint32_t                                                    null_reg14                    ; //0x74
volatile uint32_t                                                    null_reg15                    ; //0x78
volatile QSPI_REG_Config                                             Config                        ; //0x7C
}QSPI_REG_H;

#endif //QSPI_REG__H
