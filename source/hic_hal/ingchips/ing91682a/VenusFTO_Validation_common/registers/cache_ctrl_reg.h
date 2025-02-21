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
// File         : cache_ctrl_reg.h
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
#ifndef CACHE_CTRL_REG__H
#define CACHE_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t blk_cache_en0                    : 1 ;
        volatile uint32_t blk_cache_en1                    : 1 ;
        volatile uint32_t blk_cache_en2                    : 1 ;
        volatile uint32_t blk_cache_en3                    : 1 ;
        volatile uint32_t blk_cache_en4                    : 1 ;
        volatile uint32_t blk_cache_en5                    : 1 ;
        volatile uint32_t blk_cache_en6                    : 1 ;
        volatile uint32_t blk_cache_en7                    : 1 ;
        volatile uint32_t blk_remap_en0                    : 1 ;
        volatile uint32_t blk_remap_en1                    : 1 ;
        volatile uint32_t blk_remap_en2                    : 1 ;
        volatile uint32_t blk_remap_en3                    : 1 ;
        volatile uint32_t blk_remap_en4                    : 1 ;
        volatile uint32_t blk_remap_en5                    : 1 ;
        volatile uint32_t blk_remap_en6                    : 1 ;
        volatile uint32_t blk_remap_en7                    : 1 ;
        volatile uint32_t blk_prot_en0                     : 1 ;
        volatile uint32_t blk_prot_en1                     : 1 ;
        volatile uint32_t blk_prot_en2                     : 1 ;
        volatile uint32_t blk_prot_en3                     : 1 ;
        volatile uint32_t blk_prot_en4                     : 1 ;
        volatile uint32_t blk_prot_en5                     : 1 ;
        volatile uint32_t blk_prot_en6                     : 1 ;
        volatile uint32_t blk_prot_en7                     : 1 ;
        volatile uint32_t unused0                          : 8 ;
    }f;
}CACHE_CTRL_REG_enable;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused1                          : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused2                          : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused3                          : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused4                          : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused5                          : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused6                          : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused7                          : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused8                          : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused9                          : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused10                         : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused11                         : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused12                         : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused13                         : 12;
        volatile uint32_t base_addr                        : 16;
        volatile uint32_t unused14                         : 4 ;
    }f;
}CACHE_CTRL_REG_base_addr7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused15                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused16                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused17                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused18                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused19                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused20                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused21                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused22                         : 12;
        volatile uint32_t remap_offset                     : 20;
    }f;
}CACHE_CTRL_REG_remap7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t write_mode                       : 2 ;
        volatile uint32_t hprot_byp                        : 4 ;
        volatile uint32_t hprot_bus_reg                    : 4 ;
        volatile uint32_t hprot_cache_reg                  : 4 ;
        volatile uint32_t unused23                         : 14;
        volatile uint32_t cache_size_sel                   : 2 ;
        volatile uint32_t cmd_exec_mode                    : 1 ;
        volatile uint32_t debug_en                         : 1 ;
    }f;
}CACHE_CTRL_REG_cache_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t prot_detect_addr                 : 28;
        volatile uint32_t unused24                         : 4 ;
    }f;
}CACHE_CTRL_REG_bus_sts0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cache_st                         : 4 ;
        volatile uint32_t cmd_st                           : 3 ;
        volatile uint32_t write_ongoing                    : 1 ;
        volatile uint32_t unused25                         : 24;
    }f;
}CACHE_CTRL_REG_cache_sts0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_str_addr                     : 28;
        volatile uint32_t unused26                         : 4 ;
    }f;
}CACHE_CTRL_REG_cmd_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_end_addr                     : 28;
        volatile uint32_t unused27                         : 4 ;
    }f;
}CACHE_CTRL_REG_cmd_cfg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_type                         : 6 ;
        volatile uint32_t unused28                         : 25;
        volatile uint32_t cmd_str                          : 1 ;
    }f;
}CACHE_CTRL_REG_cmd_cfg2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_irq_en                       : 1 ;
        volatile uint32_t prot_irq_en                      : 1 ;
        volatile uint32_t unused29                         : 30;
    }f;
}CACHE_CTRL_REG_int_en;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_irq_raw                      : 1 ;
        volatile uint32_t prot_irq_raw                     : 1 ;
        volatile uint32_t unused30                         : 30;
    }f;
}CACHE_CTRL_REG_int_raw;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_irq_masked                   : 1 ;
        volatile uint32_t prot_irq_masked                  : 1 ;
        volatile uint32_t unused31                         : 30;
    }f;
}CACHE_CTRL_REG_int_mask;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t cmd_irq_clr                      : 1 ;
        volatile uint32_t prot_irq_clr                     : 1 ;
        volatile uint32_t unused32                         : 30;
    }f;
}CACHE_CTRL_REG_int_clr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t write_hit_cnt                    : 32;
    }f;
}CACHE_CTRL_REG_write_hit_cnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t write_miss_cnt                   : 30;
        volatile uint32_t write_cnt_run                    : 1 ;
        volatile uint32_t write_cnt_clr                    : 1 ;
    }f;
}CACHE_CTRL_REG_write_miss_cnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t read_hit_cnt                     : 32;
    }f;
}CACHE_CTRL_REG_read_hit_cnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t read_miss_cnt                    : 30;
        volatile uint32_t read_cnt_run                     : 1 ;
        volatile uint32_t read_cnt_clr                     : 1 ;
    }f;
}CACHE_CTRL_REG_read_miss_cnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t hact_cnt                         : 32;
    }f;
}CACHE_CTRL_REG_hact_cnt;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t hrdy_cnt                         : 30;
        volatile uint32_t hact_hrdy_run                    : 1 ;
        volatile uint32_t hact_hrdy_clr                    : 1 ;
    }f;
}CACHE_CTRL_REG_hrdy_cnt;

typedef struct
{
volatile CACHE_CTRL_REG_enable                                       enable                        ; //0x0
volatile CACHE_CTRL_REG_base_addr1                                   base_addr1                    ; //0x4
volatile CACHE_CTRL_REG_base_addr2                                   base_addr2                    ; //0x8
volatile CACHE_CTRL_REG_base_addr3                                   base_addr3                    ; //0xC
volatile CACHE_CTRL_REG_base_addr4                                   base_addr4                    ; //0x10
volatile CACHE_CTRL_REG_base_addr5                                   base_addr5                    ; //0x14
volatile CACHE_CTRL_REG_base_addr6                                   base_addr6                    ; //0x18
volatile CACHE_CTRL_REG_base_addr7                                   base_addr7                    ; //0x1C
volatile CACHE_CTRL_REG_remap0                                       remap0                        ; //0x20
volatile CACHE_CTRL_REG_remap1                                       remap1                        ; //0x24
volatile CACHE_CTRL_REG_remap2                                       remap2                        ; //0x28
volatile CACHE_CTRL_REG_remap3                                       remap3                        ; //0x2C
volatile CACHE_CTRL_REG_remap4                                       remap4                        ; //0x30
volatile CACHE_CTRL_REG_remap5                                       remap5                        ; //0x34
volatile CACHE_CTRL_REG_remap6                                       remap6                        ; //0x38
volatile CACHE_CTRL_REG_remap7                                       remap7                        ; //0x3C
volatile CACHE_CTRL_REG_cache_cfg0                                   cache_cfg0                    ; //0x40
volatile CACHE_CTRL_REG_bus_sts0                                     bus_sts0                      ; //0x44
volatile CACHE_CTRL_REG_cache_sts0                                   cache_sts0                    ; //0x48
volatile uint32_t                                                    null_reg0                     ; //0x4C
volatile CACHE_CTRL_REG_cmd_cfg0                                     cmd_cfg0                      ; //0x50
volatile CACHE_CTRL_REG_cmd_cfg1                                     cmd_cfg1                      ; //0x54
volatile CACHE_CTRL_REG_cmd_cfg2                                     cmd_cfg2                      ; //0x58
volatile uint32_t                                                    null_reg1                     ; //0x5C
volatile CACHE_CTRL_REG_int_en                                       int_en                        ; //0x60
volatile CACHE_CTRL_REG_int_raw                                      int_raw                       ; //0x64
volatile CACHE_CTRL_REG_int_mask                                     int_mask                      ; //0x68
volatile CACHE_CTRL_REG_int_clr                                      int_clr                       ; //0x6C
volatile CACHE_CTRL_REG_write_hit_cnt                                write_hit_cnt                 ; //0x70
volatile CACHE_CTRL_REG_write_miss_cnt                               write_miss_cnt                ; //0x74
volatile CACHE_CTRL_REG_read_hit_cnt                                 read_hit_cnt                  ; //0x78
volatile CACHE_CTRL_REG_read_miss_cnt                                read_miss_cnt                 ; //0x7C
volatile CACHE_CTRL_REG_hact_cnt                                     hact_cnt                      ; //0x80
volatile CACHE_CTRL_REG_hrdy_cnt                                     hrdy_cnt                      ; //0x84
}CACHE_CTRL_REG_H;

#endif //CACHE_CTRL_REG__H
