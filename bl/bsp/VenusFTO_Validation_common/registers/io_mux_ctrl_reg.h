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
// File         : io_mux_ctrl_reg.h
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
#ifndef IO_MUX_CTRL_REG__H
#define IO_MUX_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io0_func_sel                     : 7 ;
        volatile uint32_t io1_func_sel                     : 7 ;
        volatile uint32_t io2_func_sel                     : 7 ;
        volatile uint32_t io3_func_sel                     : 7 ;
        volatile uint32_t unused0                          : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io4_func_sel                     : 7 ;
        volatile uint32_t io5_func_sel                     : 7 ;
        volatile uint32_t io6_func_sel                     : 7 ;
        volatile uint32_t io7_func_sel                     : 7 ;
        volatile uint32_t unused1                          : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io8_func_sel                     : 7 ;
        volatile uint32_t io9_func_sel                     : 7 ;
        volatile uint32_t io10_func_sel                    : 7 ;
        volatile uint32_t io11_func_sel                    : 7 ;
        volatile uint32_t unused2                          : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io12_func_sel                    : 7 ;
        volatile uint32_t io13_func_sel                    : 7 ;
        volatile uint32_t io14_func_sel                    : 7 ;
        volatile uint32_t io15_func_sel                    : 7 ;
        volatile uint32_t unused3                          : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io16_func_sel                    : 7 ;
        volatile uint32_t io17_func_sel                    : 7 ;
        volatile uint32_t io18_func_sel                    : 2 ;
        volatile uint32_t io19_func_sel                    : 2 ;
        volatile uint32_t io20_func_sel                    : 2 ;
        volatile uint32_t io21_func_sel                    : 7 ;
        volatile uint32_t unused4                          : 5 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io22_func_sel                    : 7 ;
        volatile uint32_t io23_func_sel                    : 2 ;
        volatile uint32_t io24_func_sel                    : 2 ;
        volatile uint32_t io25_func_sel                    : 2 ;
        volatile uint32_t io26_func_sel                    : 2 ;
        volatile uint32_t io27_func_sel                    : 2 ;
        volatile uint32_t io28_func_sel                    : 2 ;
        volatile uint32_t io29_func_sel                    : 2 ;
        volatile uint32_t io30_func_sel                    : 2 ;
        volatile uint32_t io31_func_sel                    : 7 ;
        volatile uint32_t io32_func_sel                    : 2 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io33_func_sel                    : 2 ;
        volatile uint32_t io34_func_sel                    : 7 ;
        volatile uint32_t io35_func_sel                    : 7 ;
        volatile uint32_t io36_func_sel                    : 2 ;
        volatile uint32_t io37_func_sel                    : 2 ;
        volatile uint32_t io38_func_sel                    : 2 ;
        volatile uint32_t io39_func_sel                    : 2 ;
        volatile uint32_t io40_func_sel                    : 2 ;
        volatile uint32_t io41_func_sel                    : 2 ;
        volatile uint32_t unused5                          : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_out_ctrl6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t sw_tms_sel                       : 5 ;
        volatile uint32_t sw_tck_sel                       : 5 ;
        volatile uint32_t spi0_clk_in_sel                  : 5 ;
        volatile uint32_t spi0_csn_in_sel                  : 5 ;
        volatile uint32_t spi0_hold_in_sel                 : 5 ;
        volatile uint32_t unused6                          : 7 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi0_wp_in_sel                   : 5 ;
        volatile uint32_t spi0_miso_in_sel                 : 5 ;
        volatile uint32_t spi0_mosi_in_sel                 : 5 ;
        volatile uint32_t spi1_clk_in_sel                  : 5 ;
        volatile uint32_t spi1_csn_in_sel                  : 5 ;
        volatile uint32_t unused7                          : 7 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi1_miso_in_sel                 : 5 ;
        volatile uint32_t spi1_mosi_in_sel                 : 5 ;
        volatile uint32_t spi1_hold_in_sel                 : 5 ;
        volatile uint32_t spi1_wp_in_sel                   : 5 ;
        volatile uint32_t ir_datin_sel                     : 5 ;
        volatile uint32_t unused8                          : 7 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t i2s_bclk_i_sel                   : 5 ;
        volatile uint32_t i2s_lrclk_i_sel                  : 5 ;
        volatile uint32_t i2s_din_sel                      : 5 ;
        volatile uint32_t uart0_rxd_sel                    : 5 ;
        volatile uint32_t uart0_cts_sel                    : 5 ;
        volatile uint32_t unused9                          : 7 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t uart1_rxd_sel                    : 5 ;
        volatile uint32_t uart1_cts_sel                    : 5 ;
        volatile uint32_t i2c0_scl_i_sel                   : 5 ;
        volatile uint32_t i2c0_sda_i_sel                   : 5 ;
        volatile uint32_t i2c1_scl_i_sel                   : 5 ;
        volatile uint32_t unused10                         : 7 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t i2c1_sda_i_sel                   : 5 ;
        volatile uint32_t pdm_dmic_in_sel                  : 5 ;
        volatile uint32_t key_in_col_0_sel                 : 2 ;
        volatile uint32_t key_in_col_1_sel                 : 2 ;
        volatile uint32_t key_in_col_2_sel                 : 2 ;
        volatile uint32_t unused11                         : 16;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_in_col_3_sel                 : 2 ;
        volatile uint32_t key_in_col_4_sel                 : 2 ;
        volatile uint32_t key_in_col_5_sel                 : 2 ;
        volatile uint32_t key_in_col_6_sel                 : 2 ;
        volatile uint32_t key_in_col_7_sel                 : 2 ;
        volatile uint32_t unused12                         : 22;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t unused13                         : 11;
        volatile uint32_t key_in_col_8_sel                 : 2 ;
        volatile uint32_t key_in_col_9_sel                 : 2 ;
        volatile uint32_t key_in_col_10_sel                : 2 ;
        volatile uint32_t key_in_col_11_sel                : 2 ;
        volatile uint32_t key_in_col_12_sel                : 2 ;
        volatile uint32_t key_in_col_13_sel                : 2 ;
        volatile uint32_t key_in_col_14_sel                : 2 ;
        volatile uint32_t key_in_col_15_sel                : 2 ;
        volatile uint32_t key_in_col_16_sel                : 1 ;
        volatile uint32_t key_in_col_17_sel                : 1 ;
        volatile uint32_t unused14                         : 3 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t key_in_col_18_sel                : 1 ;
        volatile uint32_t key_in_col_19_sel                : 1 ;
        volatile uint32_t spi2ahb_sclk_sel                 : 1 ;
        volatile uint32_t spi2ahb_cs_sel                   : 1 ;
        volatile uint32_t spi2ahb_di_sel                   : 1 ;
        volatile uint32_t qdec_phasea_sel                  : 5 ;
        volatile uint32_t qdec_phaseb_sel                  : 5 ;
        volatile uint32_t qdec_index_sel                   : 5 ;
        volatile uint32_t qdec_ext_in_clk_sel              : 2 ;
        volatile uint32_t qdec_timer_ext_in_a1_sel         : 2 ;
        volatile uint32_t qdec_timer_ext_in_a2_sel         : 2 ;
        volatile uint32_t qdec_timer_ext_in_b2_sel         : 2 ;
        volatile uint32_t unused15                         : 4 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t p_cap_in0_sel                    : 5 ;
        volatile uint32_t p_cap_in1_sel                    : 5 ;
        volatile uint32_t p_cap_in2_sel                    : 5 ;
        volatile uint32_t p_cap_in3_sel                    : 5 ;
        volatile uint32_t p_cap_in4_sel                    : 5 ;
        volatile uint32_t p_cap_in5_sel                    : 5 ;
        volatile uint32_t unused16                         : 2 ;
    }f;
}io_mux_ctrl_REG_io_mux_in_ctrl10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_pe_lsb                        : 32;
    }f;
}io_mux_ctrl_REG_io_pe_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_pe_msb                        : 10;
        volatile uint32_t unused17                         : 22;
    }f;
}io_mux_ctrl_REG_io_pe_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_ps_lsb                        : 32;
    }f;
}io_mux_ctrl_REG_io_ps_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_ps_msb                        : 10;
        volatile uint32_t unused18                         : 22;
    }f;
}io_mux_ctrl_REG_io_ps_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_is_lsb                        : 32;
    }f;
}io_mux_ctrl_REG_io_mux_is_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_is_msb                        : 10;
        volatile uint32_t unused19                         : 22;
    }f;
}io_mux_ctrl_REG_io_mux_is_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io0_dr                           : 2 ;
        volatile uint32_t io1_dr                           : 2 ;
        volatile uint32_t io2_dr                           : 2 ;
        volatile uint32_t io3_dr                           : 2 ;
        volatile uint32_t io4_dr                           : 2 ;
        volatile uint32_t io5_dr                           : 2 ;
        volatile uint32_t io6_dr                           : 2 ;
        volatile uint32_t io7_dr                           : 2 ;
        volatile uint32_t io8_dr                           : 2 ;
        volatile uint32_t io9_dr                           : 2 ;
        volatile uint32_t io10_dr                          : 2 ;
        volatile uint32_t io11_dr                          : 2 ;
        volatile uint32_t io12_dr                          : 2 ;
        volatile uint32_t io13_dr                          : 2 ;
        volatile uint32_t io14_dr                          : 2 ;
        volatile uint32_t io15_dr                          : 2 ;
    }f;
}io_mux_ctrl_REG_io_mux_dr_ctrl0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io16_dr                          : 2 ;
        volatile uint32_t io17_dr                          : 2 ;
        volatile uint32_t io18_dr                          : 2 ;
        volatile uint32_t io19_dr                          : 2 ;
        volatile uint32_t io20_dr                          : 2 ;
        volatile uint32_t io21_dr                          : 2 ;
        volatile uint32_t io22_dr                          : 2 ;
        volatile uint32_t io23_dr                          : 2 ;
        volatile uint32_t io24_dr                          : 2 ;
        volatile uint32_t io25_dr                          : 2 ;
        volatile uint32_t io26_dr                          : 2 ;
        volatile uint32_t io27_dr                          : 2 ;
        volatile uint32_t io28_dr                          : 2 ;
        volatile uint32_t io29_dr                          : 2 ;
        volatile uint32_t io30_dr                          : 2 ;
        volatile uint32_t io31_dr                          : 2 ;
    }f;
}io_mux_ctrl_REG_io_mux_dr_ctrl1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io32_dr                          : 2 ;
        volatile uint32_t io33_dr                          : 2 ;
        volatile uint32_t io34_dr                          : 2 ;
        volatile uint32_t io35_dr                          : 2 ;
        volatile uint32_t io36_dr                          : 2 ;
        volatile uint32_t io37_dr                          : 2 ;
        volatile uint32_t io38_dr                          : 2 ;
        volatile uint32_t io39_dr                          : 2 ;
        volatile uint32_t io40_dr                          : 2 ;
        volatile uint32_t io41_dr                          : 2 ;
        volatile uint32_t unused20                         : 12;
    }f;
}io_mux_ctrl_REG_io_mux_dr_ctrl2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_clk_is                       : 1 ;
        volatile uint32_t spi_clk_dr                       : 3 ;
        volatile uint32_t spi_clk_ps                       : 1 ;
        volatile uint32_t spi_clk_pe                       : 2 ;
        volatile uint32_t unused21                         : 1 ;
        volatile uint32_t spi_cs_is                        : 1 ;
        volatile uint32_t spi_cs_dr                        : 3 ;
        volatile uint32_t spi_cs_ps                        : 1 ;
        volatile uint32_t spi_cs_pe                        : 2 ;
        volatile uint32_t unused22                         : 17;
    }f;
}io_mux_ctrl_REG_io_spiflash_cfg0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_so0_is                       : 1 ;
        volatile uint32_t spi_so0_dr                       : 3 ;
        volatile uint32_t spi_so0_ps                       : 1 ;
        volatile uint32_t spi_so0_pe                       : 2 ;
        volatile uint32_t unused23                         : 1 ;
        volatile uint32_t spi_so1_is                       : 1 ;
        volatile uint32_t spi_so1_dr                       : 3 ;
        volatile uint32_t spi_so1_ps                       : 1 ;
        volatile uint32_t spi_so1_pe                       : 2 ;
        volatile uint32_t unused24                         : 1 ;
        volatile uint32_t spi_so2_is                       : 1 ;
        volatile uint32_t spi_so2_dr                       : 3 ;
        volatile uint32_t spi_so2_ps                       : 1 ;
        volatile uint32_t spi_so2_pe                       : 2 ;
        volatile uint32_t unused25                         : 1 ;
        volatile uint32_t spi_so3_is                       : 1 ;
        volatile uint32_t spi_so3_dr                       : 3 ;
        volatile uint32_t spi_so3_ps                       : 1 ;
        volatile uint32_t spi_so3_pe                       : 2 ;
        volatile uint32_t unused26                         : 1 ;
    }f;
}io_mux_ctrl_REG_io_spiflash_cfg1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t io_bond0_pe                      : 1 ;
        volatile uint32_t io_bond1_pe                      : 1 ;
        volatile uint32_t io_bond2_pe                      : 1 ;
        volatile uint32_t unused27                         : 29;
    }f;
}io_mux_ctrl_REG_io_sys_bond_cfg;

typedef struct
{
volatile io_mux_ctrl_REG_io_mux_out_ctrl0                            io_mux_out_ctrl0              ; //0x0
volatile io_mux_ctrl_REG_io_mux_out_ctrl1                            io_mux_out_ctrl1              ; //0x4
volatile io_mux_ctrl_REG_io_mux_out_ctrl2                            io_mux_out_ctrl2              ; //0x8
volatile io_mux_ctrl_REG_io_mux_out_ctrl3                            io_mux_out_ctrl3              ; //0xC
volatile io_mux_ctrl_REG_io_mux_out_ctrl4                            io_mux_out_ctrl4              ; //0x10
volatile io_mux_ctrl_REG_io_mux_out_ctrl5                            io_mux_out_ctrl5              ; //0x14
volatile io_mux_ctrl_REG_io_mux_out_ctrl6                            io_mux_out_ctrl6              ; //0x18
volatile uint32_t                                                    null_reg0                     ; //0x1C
volatile uint32_t                                                    null_reg1                     ; //0x20
volatile uint32_t                                                    null_reg2                     ; //0x24
volatile uint32_t                                                    null_reg3                     ; //0x28
volatile uint32_t                                                    null_reg4                     ; //0x2C
volatile uint32_t                                                    null_reg5                     ; //0x30
volatile uint32_t                                                    null_reg6                     ; //0x34
volatile uint32_t                                                    null_reg7                     ; //0x38
volatile uint32_t                                                    null_reg8                     ; //0x3C
volatile io_mux_ctrl_REG_io_mux_in_ctrl0                             io_mux_in_ctrl0               ; //0x40
volatile io_mux_ctrl_REG_io_mux_in_ctrl1                             io_mux_in_ctrl1               ; //0x44
volatile io_mux_ctrl_REG_io_mux_in_ctrl2                             io_mux_in_ctrl2               ; //0x48
volatile io_mux_ctrl_REG_io_mux_in_ctrl3                             io_mux_in_ctrl3               ; //0x4C
volatile io_mux_ctrl_REG_io_mux_in_ctrl4                             io_mux_in_ctrl4               ; //0x50
volatile io_mux_ctrl_REG_io_mux_in_ctrl5                             io_mux_in_ctrl5               ; //0x54
volatile io_mux_ctrl_REG_io_mux_in_ctrl6                             io_mux_in_ctrl6               ; //0x58
volatile io_mux_ctrl_REG_io_mux_in_ctrl7                             io_mux_in_ctrl7               ; //0x5C
volatile uint32_t                                                    null_reg9                     ; //0x60
volatile io_mux_ctrl_REG_io_mux_in_ctrl9                             io_mux_in_ctrl9               ; //0x64
volatile io_mux_ctrl_REG_io_mux_in_ctrl10                            io_mux_in_ctrl10              ; //0x68
volatile uint32_t                                                    null_reg10                    ; //0x6C
volatile uint32_t                                                    null_reg11                    ; //0x70
volatile uint32_t                                                    null_reg12                    ; //0x74
volatile uint32_t                                                    null_reg13                    ; //0x78
volatile uint32_t                                                    null_reg14                    ; //0x7C
volatile io_mux_ctrl_REG_io_pe_ctrl0                                 io_pe_ctrl0                   ; //0x80
volatile io_mux_ctrl_REG_io_pe_ctrl1                                 io_pe_ctrl1                   ; //0x84
volatile io_mux_ctrl_REG_io_ps_ctrl0                                 io_ps_ctrl0                   ; //0x88
volatile io_mux_ctrl_REG_io_ps_ctrl1                                 io_ps_ctrl1                   ; //0x8C
volatile uint32_t                                                    null_reg15                    ; //0x90
volatile uint32_t                                                    null_reg16                    ; //0x94
volatile uint32_t                                                    null_reg17                    ; //0x98
volatile uint32_t                                                    null_reg18                    ; //0x9C
volatile uint32_t                                                    null_reg19                    ; //0xA0
volatile uint32_t                                                    null_reg20                    ; //0xA4
volatile uint32_t                                                    null_reg21                    ; //0xA8
volatile uint32_t                                                    null_reg22                    ; //0xAC
volatile uint32_t                                                    null_reg23                    ; //0xB0
volatile uint32_t                                                    null_reg24                    ; //0xB4
volatile uint32_t                                                    null_reg25                    ; //0xB8
volatile uint32_t                                                    null_reg26                    ; //0xBC
volatile uint32_t                                                    null_reg27                    ; //0xC0
volatile uint32_t                                                    null_reg28                    ; //0xC4
volatile uint32_t                                                    null_reg29                    ; //0xC8
volatile uint32_t                                                    null_reg30                    ; //0xCC
volatile uint32_t                                                    null_reg31                    ; //0xD0
volatile uint32_t                                                    null_reg32                    ; //0xD4
volatile uint32_t                                                    null_reg33                    ; //0xD8
volatile uint32_t                                                    null_reg34                    ; //0xDC
volatile uint32_t                                                    null_reg35                    ; //0xE0
volatile uint32_t                                                    null_reg36                    ; //0xE4
volatile uint32_t                                                    null_reg37                    ; //0xE8
volatile uint32_t                                                    null_reg38                    ; //0xEC
volatile uint32_t                                                    null_reg39                    ; //0xF0
volatile uint32_t                                                    null_reg40                    ; //0xF4
volatile uint32_t                                                    null_reg41                    ; //0xF8
volatile uint32_t                                                    null_reg42                    ; //0xFC
volatile io_mux_ctrl_REG_io_mux_is_ctrl0                             io_mux_is_ctrl0               ; //0x100
volatile io_mux_ctrl_REG_io_mux_is_ctrl1                             io_mux_is_ctrl1               ; //0x104
volatile io_mux_ctrl_REG_io_mux_dr_ctrl0                             io_mux_dr_ctrl0               ; //0x108
volatile io_mux_ctrl_REG_io_mux_dr_ctrl1                             io_mux_dr_ctrl1               ; //0x10C
volatile io_mux_ctrl_REG_io_mux_dr_ctrl2                             io_mux_dr_ctrl2               ; //0x110
volatile io_mux_ctrl_REG_io_spiflash_cfg0                            io_spiflash_cfg0              ; //0x114
volatile io_mux_ctrl_REG_io_spiflash_cfg1                            io_spiflash_cfg1              ; //0x118
volatile io_mux_ctrl_REG_io_sys_bond_cfg                             io_sys_bond_cfg               ; //0x11C
}io_mux_ctrl_REG_H;

#endif //io_mux_ctrl_REG__H
