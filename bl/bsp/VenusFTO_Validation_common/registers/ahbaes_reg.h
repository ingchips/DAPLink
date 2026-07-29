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
// File         : ahbaes_reg.h
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
#ifndef AHBAES_REG__H
#define AHBAES_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t encrypt_ena                      : 1 ;
        volatile uint32_t encrypt_auto_clkgate_ena         : 1 ;
        volatile uint32_t encrypt_clkgate_sw_ctrl          : 1 ;
        volatile uint32_t encrypt_int_mode                 : 1 ;
        volatile uint32_t encrypt_int_mask                 : 1 ;
        volatile uint32_t encrypt_int_clr                  : 1 ;
        volatile uint32_t encrypt_soft_rst                 : 1 ;
        volatile uint32_t unused0                          : 25;
    }f;
}AHBAES_REG_GLB_CTRL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aes_start                        : 1 ;
        volatile uint32_t meshccm_encrypt_start            : 1 ;
        volatile uint32_t meshccm_decrypt_start            : 1 ;
        volatile uint32_t unused1                          : 29;
    }f;
}AHBAES_REG_START;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t encrypt_int_source               : 2 ;
        volatile uint32_t encrypt_mask_int_stat            : 1 ;
        volatile uint32_t encrypt_saw_int_stat             : 1 ;
        volatile uint32_t meshccm_dec_mic_err              : 1 ;
        volatile uint32_t encrypt_busy                     : 1 ;
        volatile uint32_t unused2                          : 26;
    }f;
}AHBAES_REG_STATUS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t aes_loc                          : 10;
        volatile uint32_t unused3                          : 22;
    }f;
}AHBAES_REG_AES_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key0                             : 32;
    }f;
}AHBAES_REG_KEY0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key1                             : 32;
    }f;
}AHBAES_REG_KEY1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key2                             : 32;
    }f;
}AHBAES_REG_KEY2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key3                             : 32;
    }f;
}AHBAES_REG_KEY3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_mode                 : 1 ;
        volatile uint32_t meshccm_mic_mode                 : 1 ;
        volatile uint32_t meshccm_tx_lth                   : 9 ;
        volatile uint32_t meshccm_rx_lth                   : 9 ;
        volatile uint32_t unused4                          : 12;
    }f;
}AHBAES_REG_MESHCCM_CFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_ptr                      : 10;
        volatile uint32_t unused5                          : 22;
    }f;
}AHBAES_REG_MESHCCM_LOC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data0                : 32;
    }f;
}AHBAES_REG_MESHCCM_AUT_DATA0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data1                : 32;
    }f;
}AHBAES_REG_MESHCCM_AUT_DATA1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data2                : 32;
    }f;
}AHBAES_REG_MESHCCM_AUT_DATA2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_aut_data3                : 32;
    }f;
}AHBAES_REG_MESHCCM_AUT_DATA3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce0                   : 32;
    }f;
}AHBAES_REG_MESHCCM_NONCE0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce1                   : 32;
    }f;
}AHBAES_REG_MESHCCM_NONCE1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce2                   : 32;
    }f;
}AHBAES_REG_MESHCCM_NONCE2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_nonce3                   : 8 ;
        volatile uint32_t unused6                          : 24;
    }f;
}AHBAES_REG_MESHCCM_NONCE3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_decrypt_mic              : 32;
    }f;
}AHBAES_REG_MESHCCM_DECRYPT_MIC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t meshccm_encrypt_mic              : 32;
    }f;
}AHBAES_REG_MESHCCM_ENCRYPT_MIC;

typedef struct
{
volatile AHBAES_REG_GLB_CTRL                                         GLB_CTRL                      ; //0x0
volatile AHBAES_REG_START                                            START                         ; //0x4
volatile AHBAES_REG_STATUS                                           STATUS                        ; //0x8
volatile AHBAES_REG_AES_LOC                                          AES_LOC                       ; //0xC
volatile AHBAES_REG_KEY0                                             KEY0                          ; //0x10
volatile AHBAES_REG_KEY1                                             KEY1                          ; //0x14
volatile AHBAES_REG_KEY2                                             KEY2                          ; //0x18
volatile AHBAES_REG_KEY3                                             KEY3                          ; //0x1C
volatile AHBAES_REG_MESHCCM_CFG                                      MESHCCM_CFG                   ; //0x20
volatile AHBAES_REG_MESHCCM_LOC                                      MESHCCM_LOC                   ; //0x24
volatile AHBAES_REG_MESHCCM_AUT_DATA0                                MESHCCM_AUT_DATA0             ; //0x28
volatile AHBAES_REG_MESHCCM_AUT_DATA1                                MESHCCM_AUT_DATA1             ; //0x2C
volatile AHBAES_REG_MESHCCM_AUT_DATA2                                MESHCCM_AUT_DATA2             ; //0x30
volatile AHBAES_REG_MESHCCM_AUT_DATA3                                MESHCCM_AUT_DATA3             ; //0x34
volatile AHBAES_REG_MESHCCM_NONCE0                                   MESHCCM_NONCE0                ; //0x38
volatile AHBAES_REG_MESHCCM_NONCE1                                   MESHCCM_NONCE1                ; //0x3C
volatile AHBAES_REG_MESHCCM_NONCE2                                   MESHCCM_NONCE2                ; //0x40
volatile AHBAES_REG_MESHCCM_NONCE3                                   MESHCCM_NONCE3                ; //0x44
volatile AHBAES_REG_MESHCCM_DECRYPT_MIC                              MESHCCM_DECRYPT_MIC           ; //0x48
volatile AHBAES_REG_MESHCCM_ENCRYPT_MIC                              MESHCCM_ENCRYPT_MIC           ; //0x4C
}AHBAES_REG_H;

#endif //AHBAES_REG__H
