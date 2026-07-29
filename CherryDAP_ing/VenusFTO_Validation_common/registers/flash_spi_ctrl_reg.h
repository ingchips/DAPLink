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
// File         : flash_spi_ctrl_reg.h
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
#ifndef FLASH_SPI_CTRL_REG__H
#define FLASH_SPI_CTRL_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tx_cmd                           : 8 ;
        volatile uint32_t tx_addr                          : 24;
    }f;
}FLASH_SPI_CTRL_REG_spi_cmd_addr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tx_modebit                       : 8 ;
        volatile uint32_t tx_block_size                    : 9 ;
        volatile uint32_t unused0                          : 7 ;
        volatile uint32_t tx_continuous_mode               : 1 ;
        volatile uint32_t unused1                          : 7 ;
    }f;
}FLASH_SPI_CTRL_REG_spi_block_size;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_data_fifo                    : 32;
    }f;
}FLASH_SPI_CTRL_REG_spi_data_fifo;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t busy_stat                        : 1 ;
        volatile uint32_t tx_fifo_empty                    : 1 ;
        volatile uint32_t tx_fifo_full                     : 1 ;
        volatile uint32_t rx_fifo_empty                    : 1 ;
        volatile uint32_t rx_fifo_count                    : 5 ;
        volatile uint32_t read_state_busy                  : 1 ;
        volatile uint32_t main_fsm_idle                    : 1 ;
        volatile uint32_t unused2                          : 21;
    }f;
}FLASH_SPI_CTRL_REG_spi_status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_status                        : 32;
    }f;
}FLASH_SPI_CTRL_REG_rx_status;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t quad_mode                        : 1 ;
        volatile uint32_t wprotect_pin                     : 1 ;
        volatile uint32_t hold_pin                         : 1 ;
        volatile uint32_t unused3                          : 1 ;
        volatile uint32_t sample_delay_reg                 : 3 ;
        volatile uint32_t bypass_start_read                : 1 ;
        volatile uint32_t clk_divider                      : 8 ;
        volatile uint32_t cmd_quad                         : 1 ;
        volatile uint32_t tx_rx_size                       : 2 ;
        volatile uint32_t unused4                          : 13;
    }f;
}FLASH_SPI_CTRL_REG_spi_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t rx_fifo_clr                      : 1 ;
        volatile uint32_t tx_fifo_clr                      : 1 ;
        volatile uint32_t unused5                          : 30;
    }f;
}FLASH_SPI_CTRL_REG_spi_fifo_control;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t spi_cs_num                       : 1 ;
        volatile uint32_t spi_size                         : 2 ;
        volatile uint32_t spi_128m                         : 1 ;
        volatile uint32_t unused6                          : 1 ;
        volatile uint32_t spi_cs1_sel                      : 1 ;
        volatile uint32_t spi_cs1_sel1                     : 1 ;
        volatile uint32_t diff_128m_diff_cmd_en            : 1 ;
        volatile uint32_t unused7                          : 24;
    }f;
}FLASH_SPI_CTRL_REG_spi_cs_size;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t qread_cmd_int                    : 8 ;
        volatile uint32_t fread_cmd                        : 8 ;
        volatile uint32_t read_cmd                         : 8 ;
        volatile uint32_t protect                          : 8 ;
    }f;
}FLASH_SPI_CTRL_REG_spi_read_cmd;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t nand_sel                         : 1 ;
        volatile uint32_t nand_addr                        : 2 ;
        volatile uint32_t unused8                          : 5 ;
        volatile uint32_t page_read_cmd                    : 8 ;
        volatile uint32_t get_sts_cmd                      : 8 ;
        volatile uint32_t ram_read_cmd                     : 8 ;
    }f;
}FLASH_SPI_CTRL_REG_spi_nand_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t get_sts_addr                     : 8 ;
        volatile uint32_t program_exe_cmd                  : 8 ;
        volatile uint32_t sts_qip                          : 8 ;
        volatile uint32_t unused9                          : 8 ;
    }f;
}FLASH_SPI_CTRL_REG_spi_nand_config2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t four_byte_addr                   : 1 ;
        volatile uint32_t dummy_cycle_en                   : 1 ;
        volatile uint32_t unused10                         : 6 ;
        volatile uint32_t dummy_cycle                      : 4 ;
        volatile uint32_t unused11                         : 20;
    }f;
}FLASH_SPI_CTRL_REG_spi_256_512_flash_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t first_128m_cmd                   : 8 ;
        volatile uint32_t second_128m_cmd                  : 8 ;
        volatile uint32_t third_128m_cmd                   : 8 ;
        volatile uint32_t fourth_128m_cmd                  : 8 ;
    }f;
}FLASH_SPI_CTRL_REG_spi_128_flash_config;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t timeout_value                    : 32;
    }f;
}FLASH_SPI_CTRL_REG_timeout_value;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t tsus_value                       : 16;
        volatile uint32_t tres_value                       : 16;
    }f;
}FLASH_SPI_CTRL_REG_tsus_tres;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_irq                          : 1 ;
        volatile uint32_t unused12                         : 15;
        volatile uint32_t reg_irq_raw                      : 1 ;
        volatile uint32_t unused13                         : 15;
    }f;
}FLASH_SPI_CTRL_REG_flash_irq;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t reg_irq_mask                     : 1 ;
        volatile uint32_t unused14                         : 31;
    }f;
}FLASH_SPI_CTRL_REG_flash_irq_mask;

typedef struct
{
volatile FLASH_SPI_CTRL_REG_spi_cmd_addr                             spi_cmd_addr                  ; //0x0
volatile FLASH_SPI_CTRL_REG_spi_block_size                           spi_block_size                ; //0x4
volatile FLASH_SPI_CTRL_REG_spi_data_fifo                            spi_data_fifo                 ; //0x8
volatile FLASH_SPI_CTRL_REG_spi_status                               spi_status                    ; //0xC
volatile FLASH_SPI_CTRL_REG_rx_status                                rx_status                     ; //0x10
volatile FLASH_SPI_CTRL_REG_spi_config                               spi_config                    ; //0x14
volatile FLASH_SPI_CTRL_REG_spi_fifo_control                         spi_fifo_control              ; //0x18
volatile FLASH_SPI_CTRL_REG_spi_cs_size                              spi_cs_size                   ; //0x1C
volatile FLASH_SPI_CTRL_REG_spi_read_cmd                             spi_read_cmd                  ; //0x20
volatile FLASH_SPI_CTRL_REG_spi_nand_config                          spi_nand_config               ; //0x24
volatile FLASH_SPI_CTRL_REG_spi_nand_config2                         spi_nand_config2              ; //0x28
volatile FLASH_SPI_CTRL_REG_spi_256_512_flash_config                 spi_256_512_flash_config      ; //0x2C
volatile FLASH_SPI_CTRL_REG_spi_128_flash_config                     spi_128_flash_config          ; //0x30
volatile FLASH_SPI_CTRL_REG_timeout_value                            timeout_value                 ; //0x34
volatile uint32_t                                                    null_reg0                     ; //0x38
volatile FLASH_SPI_CTRL_REG_tsus_tres                                tsus_tres                     ; //0x3C
volatile FLASH_SPI_CTRL_REG_flash_irq                                flash_irq                     ; //0x40
volatile FLASH_SPI_CTRL_REG_flash_irq_mask                           flash_irq_mask                ; //0x44
}FLASH_SPI_CTRL_REG_H;

#endif //FLASH_SPI_CTRL_REG__H
