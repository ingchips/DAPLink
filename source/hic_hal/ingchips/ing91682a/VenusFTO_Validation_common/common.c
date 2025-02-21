
#ifndef COMMON__C
#define COMMON__C

#include "common.h"

////////////////////////////////////////
//  Flash
////////////////////////////////////////
void FlashWaitBusyDown(void)
{
    while(flash_spi_ctrl_reg->spi_status.f.main_fsm_idle != 0x1);
    while(flash_spi_ctrl_reg->spi_status.f.busy_stat !=0x0);
}

void FlashWaitWIPDown(void)
{
    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x2<<8);//tx_block_size=2
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x05;
    FlashWaitBusyDown();
    while(flash_spi_ctrl_reg->rx_status.r & 0x1){
        flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x2<<8);//tx_block_size=2
        flash_spi_ctrl_reg->spi_cmd_addr.r = 0x05;
        FlashWaitBusyDown();
    }
}

void FlashWriteEnable(void)
{
    flash_spi_ctrl_reg->spi_config.r = (flash_spi_ctrl_reg->spi_config.r & (~(0x3<<17))) | (0x0<<17);//tx_rx_size
    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x0<<8);//tx_block_size
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x06;
    FlashWaitBusyDown();

    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x2<<8);
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x05;
    FlashWaitBusyDown();
}

void FlashWriteDisable(void)
{
    flash_spi_ctrl_reg->spi_config.r = (flash_spi_ctrl_reg->spi_config.r & (~(0x3<<17))) | (0x0<<17);//tx_rx_size
    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x0<<8);//tx_block_size
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x04;
    FlashWaitBusyDown();
}

void FlashSetQE(void)
{
    FlashSetStatusReg((uint16_t)(0x1<<9));
}

void FlashEnableContinuousMode(void)
{
    flash_spi_ctrl_reg->spi_block_size.r = flash_spi_ctrl_reg->spi_block_size.r | (0x1<<24) | (0xa<<4);
}

void FlashDisableContinuousMode(void)
{
    flash_spi_ctrl_reg->spi_config.r = (flash_spi_ctrl_reg->spi_config.r & (~(0x3<<17))) | (0x0<<17);//tx_rx_size
    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x0<<8);//tx_block_size
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0xff;
    FlashWaitBusyDown();

    flash_spi_ctrl_reg->spi_block_size.r = flash_spi_ctrl_reg->spi_block_size.r & (~(0x1<<24)) & 0xffffff00;
}

void FlashPageProgram(uint32_t addr, uint8_t data[256], uint32_t len)
{
    uint32_t i;
    addr = addr-INGSIMD_A_FLASH_BASE;
    for(i=0; i<len; i++){
        flash_spi_ctrl_reg->spi_data_fifo.r = data[i];
    }
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0x02;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashQuadPageProgram(uint32_t addr, uint8_t data[256], uint32_t len)
{
    uint32_t i;
    addr = addr-INGSIMD_A_FLASH_BASE;
    for(i=0; i<len; i++){
        flash_spi_ctrl_reg->spi_data_fifo.r = data[i];
    }
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0x32;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashPageErase(uint32_t addr)
{
    addr = addr-INGSIMD_A_FLASH_BASE;
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0x81;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashSectorErase(uint32_t addr)
{
    addr = addr-INGSIMD_A_FLASH_BASE;
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0x20;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashBlock32Erase(uint32_t addr)
{
    addr = addr-INGSIMD_A_FLASH_BASE;
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0x52;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashBlock64Erase(uint32_t addr)
{
    addr = addr-INGSIMD_A_FLASH_BASE;
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = ((addr&0xffffff)<<8) | 0xd8;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashChipErase(void)
{
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x60;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

void FlashSetStatusReg(uint16_t data)
{
    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x2<<8);//tx_block_size
    FlashWriteEnable();
    flash_spi_ctrl_reg->spi_data_fifo.r = data&0xff;
    flash_spi_ctrl_reg->spi_data_fifo.r = (data>>8)&0xff;
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x01;
    FlashWaitBusyDown();
    FlashWaitWIPDown();
}

uint16_t FlashGetStatusReg(void)
{
    uint8_t SR;
    uint16_t ret;

    flash_spi_ctrl_reg->spi_block_size.r = (flash_spi_ctrl_reg->spi_block_size.r & (~(0x1ff<<8))) | (0x2<<8);//tx_block_size
    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x05;
    FlashWaitBusyDown();
    SR = flash_spi_ctrl_reg->rx_status.r & 0xff;
    ret = SR;

    flash_spi_ctrl_reg->spi_cmd_addr.r = 0x35;
    FlashWaitBusyDown();
    SR = flash_spi_ctrl_reg->rx_status.r & 0xff;
    ret = ret | (SR<<8);
    return ret;
}

////////////////////////////////////////
//  IOMUX
////////////////////////////////////////
uint16_t IOMUXFuncSelMap[42][10] = {
      0xFFFF,   0x57FF,   0x9555,   0xC4,     0x1000,   0xFFFC,   0x3FFF,   0x0,      0xF870,     0x1, // IOMuxFuncSelMap_io0
      0xFFFF,   0xABFF,   0x2AAA,   0x149,    0x2000,   0xFFFC,   0x5FFF,   0x0,      0xF970,     0x1, // IOMuxFuncSelMap_io1
      0xFFFF,   0x57FF,   0x4555,   0x242,    0x4000,   0xFFFC,   0x9FFF,   0x0,      0xFA70,     0x1, // IOMuxFuncSelMap_io2
      0xFFFF,   0xABFF,   0x9AAA,   0x444,    0x8000,   0xFFFC,   0x1FFF,   0x1,      0xF8F0,     0x1, // IOMuxFuncSelMap_io3
      0xFFFF,   0x57FF,   0x2555,   0x859,    0x0,      0xFFFD,   0x1FFF,   0x2,      0xF870,     0x1, // IOMuxFuncSelMap_io4
      0xFFFF,   0xABFF,   0x4AAA,   0x1052,   0x0,      0xFFFE,   0x1FFF,   0x4,      0xFC70,     0x1, // IOMuxFuncSelMap_io5
      0xFFFF,   0x57FF,   0x9555,   0x2054,   0x1000,   0xFFFC,   0x1FFF,   0x8,      0xF870,     0x1, // IOMuxFuncSelMap_io6
      0xFFFF,   0xABFF,   0x2AAA,   0x4059,   0x2000,   0xFFFC,   0x1FFF,   0x10,     0xF970,     0x1, // IOMuxFuncSelMap_io7
      0xFFFF,   0x57FF,   0x4555,   0x8052,   0x4000,   0xFFFC,   0x1FFF,   0x20,     0xFA70,     0x1, // IOMuxFuncSelMap_io8
      0xFFFF,   0xABFF,   0x9AAA,   0x54,     0x8001,   0xFFFC,   0x1FFF,   0x40,     0xF8F0,     0x1, // IOMuxFuncSelMap_io9
      0xFFFF,   0x57FF,   0x2555,   0x59,     0x2,      0xFFFD,   0x1FFF,   0x80,     0xF870,     0x1, // IOMuxFuncSelMap_io10
      0xFFFF,   0xABFF,   0x4AAA,   0x62,     0x4,      0xFFFE,   0x1FFF,   0x100,    0xFC70,     0x1, // IOMuxFuncSelMap_io11
      0xFFFF,   0x57FF,   0x9555,   0x64,     0x8,      0xFFFC,   0x1FFF,   0x200,    0xF870,     0x1, // IOMuxFuncSelMap_io12
      0xFFFF,   0xABFF,   0x2AAA,   0x69,     0x10,     0xFFFC,   0x1FFF,   0x400,    0xF870,     0x1, // IOMuxFuncSelMap_io13
      0xFFFF,   0x57FF,   0x4555,   0x62,     0x20,     0xFFFC,   0x1FFF,   0x800,    0xF870,     0x1, // IOMuxFuncSelMap_io14
      0xFFFF,   0xABFF,   0x9AAA,   0x64,     0x40,     0xFFFC,   0x1FFF,   0x1000,   0xF872,     0x1, // IOMuxFuncSelMap_io15
      0xFFFF,   0x57FF,   0x2555,   0x69,     0x80,     0xFFFC,   0x1FFF,   0x2000,   0xF874,     0x1, // IOMuxFuncSelMap_io16
      0xFFFF,   0xABFF,   0x4AAA,   0x62,     0x900,    0xFFFC,   0x1FFF,   0x4000,   0xF878,     0x1, // IOMuxFuncSelMap_io17
      0x4,      0x0,      0x0,      0x0,      0x0,      0x20,     0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io18
      0x2,      0x0,      0x0,      0x0,      0x0,      0x10,     0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io19
      0x8,      0x0,      0x0,      0x0,      0x0,      0x40,     0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io20
      0xFFFF,   0x57FF,   0x9555,   0x44,     0x200,    0xFFFC,   0x1FFF,   0x8000,   0xF870,     0x1, // IOMuxFuncSelMap_io21
      0xFFFF,   0xABFF,   0x2AAA,   0x49,     0x400,    0xFFFC,   0x1FFF,   0x0,      0xF871,     0x1, // IOMuxFuncSelMap_io22
      0x0,      0x0,      0x0,      0x80,     0x0,      0x0,      0x2000,   0x0,      0x0,        0x0, // IOMuxFuncSelMap_io23
      0x0,      0x0,      0x0,      0x100,    0x0,      0x0,      0x4000,   0x0,      0x0,        0x0, // IOMuxFuncSelMap_io24
      0x0,      0x0,      0x0,      0x200,    0x0,      0x0,      0x8000,   0x0,      0x0,        0x0, // IOMuxFuncSelMap_io25
      0x10,     0x0,      0x0,      0x0,      0x0,      0x80,     0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io26
      0x20,     0x0,      0x0,      0x0,      0x0,      0x100,    0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io27
      0x40,     0x0,      0x0,      0x0,      0x0,      0x200,    0x0,      0x0,      0x0,        0x0, // IOMuxFuncSelMap_io28
      0x0,      0x0,      0x0,      0x400,    0x0,      0x0,      0x0,      0x1,      0x0,        0x0, // IOMuxFuncSelMap_io29
      0x0,      0x0,      0x0,      0x800,    0x0,      0x0,      0x0,      0x2,      0x0,        0x0, // IOMuxFuncSelMap_io30
      0xFFFF,   0x57FF,   0x4555,   0x1042,   0x0,      0xFFFC,   0x1FFF,   0x4,      0xF870,     0x1, // IOMuxFuncSelMap_io31
      0x0,      0x0,      0x0,      0x2000,   0x0,      0x0,      0x0,      0x8,      0x0,        0x0, // IOMuxFuncSelMap_io32
      0x0,      0x0,      0x0,      0x4000,   0x0,      0x0,      0x0,      0x10,     0x0,        0x0, // IOMuxFuncSelMap_io33
      0xFFFF,   0xABFF,   0x9AAA,   0x8074,   0x0,      0xFFFC,   0x1FFF,   0x20,     0xF870,     0x1, // IOMuxFuncSelMap_io34
      0xFFFF,   0x57FF,   0x2555,   0x79,     0x1,      0xFFFC,   0x1FFF,   0x40,     0xF870,     0x1, // IOMuxFuncSelMap_io35
      0x0,      0x0,      0x0,      0x0,      0x2,      0x0,      0x0,      0x80,     0x0,        0x0, // IOMuxFuncSelMap_io36
      0x0,      0x0,      0x0,      0x0,      0x4,      0x0,      0x0,      0x100,    0x0,        0x0, // IOMuxFuncSelMap_io37
      0x0,      0x0,      0x0,      0x0,      0x8,      0x0,      0x0,      0x200,    0x0,        0x0, // IOMuxFuncSelMap_io38
      0x0,      0x0,      0x0,      0x0,      0x10,     0x0,      0x0,      0x400,    0x0,        0x0, // IOMuxFuncSelMap_io39
      0x0,      0x0,      0x0,      0x0,      0x20,     0x0,      0x0,      0x800,    0x0,        0x0, // IOMuxFuncSelMap_io40
      0x0,      0x0,      0x0,      0x0,      0x40,     0x0,      0x0,      0x1000,   0x0,        0x0, // IOMuxFuncSelMap_io41
};

// IO OutputSel Map
uint16_t IOMUXOutputSelMap_swdo[3]                               = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi0_clk_out[3]                       = {        0xFFFF,   0x806B,          0xC};
uint16_t IOMUXOutputSelMap_spi0_csn_out[3]                       = {        0xFFFF,   0x8067,          0xC};
uint16_t IOMUXOutputSelMap_spi0_hold_out[3]                      = {        0xFFFF,   0x8073,          0xC};
uint16_t IOMUXOutputSelMap_spi0_wp_out[3]                        = {        0xFFFF,   0x8463,          0xC};
uint16_t IOMUXOutputSelMap_spi0_miso_out[3]                      = {        0xFFFF,   0x8863,          0xC};
uint16_t IOMUXOutputSelMap_spi0_mosi_out[3]                      = {        0xFFFF,   0x9063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_clk_out[3]                       = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_csn_out[3]                       = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_miso_out[3]                      = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_mosi_out[3]                      = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_hold_out[3]                      = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_spi1_wp_out[3]                        = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_ir_wakeup[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_ir_datout[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2s_bclk_o[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2s_lrclk_o[3]                        = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2s_dout[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_uart0_txd[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_uart0_rts[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_uart1_txd[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_uart1_rts[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2c0_scl_o[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2c0_sda_o[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2c1_scl_o[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_i2c1_sda_o[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_pwm_0a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_0b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_1a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_1b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_2a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_2b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_3a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_3b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_4a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_4b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_5a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_5b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_6a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_6b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_7a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_7b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_pwm_8a[3]                             = {        0x5555,   0x8021,          0x8};
uint16_t IOMUXOutputSelMap_pwm_8b[3]                             = {        0xAAAA,   0x42,            0x4};
uint16_t IOMUXOutputSelMap_lle_ant_sw0[3]                        = {        0x9249,   0x20,            0x4};
uint16_t IOMUXOutputSelMap_lle_ant_sw1[3]                        = {        0x2492,   0x41,            0x8};
uint16_t IOMUXOutputSelMap_lle_ant_sw2[3]                        = {        0x4924,   0x8002,          0x0};
uint16_t IOMUXOutputSelMap_lle_ant_sw3[3]                        = {        0x9249,   0x20,            0x4};
uint16_t IOMUXOutputSelMap_lle_ant_sw4[3]                        = {        0x2492,   0x41,            0x8};
uint16_t IOMUXOutputSelMap_lle_ant_sw5[3]                        = {        0x4924,   0x8002,          0x0};
uint16_t IOMUXOutputSelMap_lle_ant_sw6[3]                        = {        0x9249,   0x20,            0x4};
uint16_t IOMUXOutputSelMap_lle_ant_sw7[3]                        = {        0x2492,   0x41,            0x8};
uint16_t IOMUXOutputSelMap_lle_pa_txen[3]                        = {        0x7F0,    0x0,             0xC};
uint16_t IOMUXOutputSelMap_lle_pa_rxen[3]                        = {        0xF800,   0x3,             0xC};
uint16_t IOMUXOutputSelMap_pdm_dmic_mclk[3]                      = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXOutputSelMap_key_out_row_0[3]                      = {        0x1,      0x80,            0x0};
uint16_t IOMUXOutputSelMap_key_out_row_1[3]                      = {        0x2,      0x100,           0x0};
uint16_t IOMUXOutputSelMap_key_out_row_2[3]                      = {        0x4,      0x200,           0x0};
uint16_t IOMUXOutputSelMap_key_out_row_3[3]                      = {        0x8,      0x2000,          0x0};
uint16_t IOMUXOutputSelMap_key_out_row_4[3]                      = {        0x10,     0x4000,          0x0};
uint16_t IOMUXOutputSelMap_key_out_row_5[3]                      = {        0x20,     0x8000,          0x0};
uint16_t IOMUXOutputSelMap_key_out_row_6[3]                      = {        0x40,     0x0,             0x1};
uint16_t IOMUXOutputSelMap_key_out_row_7[3]                      = {        0x80,     0x0,             0x2};
uint16_t IOMUXOutputSelMap_key_out_row_8[3]                      = {        0x100,    0x0,             0x4};
uint16_t IOMUXOutputSelMap_key_out_row_9[3]                      = {        0x200,    0x0,             0x8};
uint16_t IOMUXOutputSelMap_key_out_row_10[3]                     = {        0x400,    0x0,             0x10};
uint16_t IOMUXOutputSelMap_key_out_row_11[3]                     = {        0x800,    0x0,             0x20};
uint16_t IOMUXOutputSelMap_key_out_row_12[3]                     = {        0x1000,   0x0,             0x40};
uint16_t IOMUXOutputSelMap_key_out_row_13[3]                     = {        0x2000,   0x0,             0x80};
uint16_t IOMUXOutputSelMap_key_out_row_14[3]                     = {        0x4000,   0x0,             0x100};
uint16_t IOMUXOutputSelMap_key_out_row_15[3]                     = {        0x8000,   0x0,             0x200};
uint16_t IOMUXOutputSelMap_key_out_row_16[3]                     = {        0x0,      0x1,             0x0};
uint16_t IOMUXOutputSelMap_key_out_row_17[3]                     = {        0x0,      0x2,             0x0};
uint16_t IOMUXOutputSelMap_key_out_row_18[3]                     = {        0x0,      0x20,            0x0};
uint16_t IOMUXOutputSelMap_key_out_row_19[3]                     = {        0x0,      0x40,            0x0};
uint16_t IOMUXOutputSelMap_spi2ahb_do[3]                         = {        0x0,      0x2,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_a0[3]              = {        0x41,     0x0,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_a1[3]              = {        0x82,     0x0,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_a2[3]              = {        0x104,    0x0,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_b0[3]              = {        0x208,    0x0,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_b1[3]              = {        0x410,    0x0,             0x0};
uint16_t IOMUXOutputSelMap_qdec_timer_ext_out_b2[3]              = {        0x820,    0x0,             0x0};

// IO InputSel Map
uint16_t IOMUXInputSelMap_sw_tms[3]                              = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_sw_tck[3]                              = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi0_clk_in[3]                         = {        0xFFFF,   0x806B,          0xC};
uint16_t IOMUXInputSelMap_spi0_csn_in[3]                         = {        0xFFFF,   0x8067,          0xC};
uint16_t IOMUXInputSelMap_spi0_hold_in[3]                        = {        0xFFFF,   0x8073,          0xC};
uint16_t IOMUXInputSelMap_spi0_wp_in[3]                          = {        0xFFFF,   0x8463,          0xC};
uint16_t IOMUXInputSelMap_spi0_miso_in[3]                        = {        0xFFFF,   0x8863,          0xC};
uint16_t IOMUXInputSelMap_spi0_mosi_in[3]                        = {        0xFFFF,   0x9063,          0xC};
uint16_t IOMUXInputSelMap_spi1_clk_in[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi1_csn_in[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi1_miso_in[3]                        = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi1_mosi_in[3]                        = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi1_hold_in[3]                        = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_spi1_wp_in[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_ir_datin[3]                            = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2s_bclk_i[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2s_lrclk_i[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2s_din[3]                             = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_uart0_rxd[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_uart0_cts[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_uart1_rxd[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_uart1_cts[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2c0_scl_i[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2c0_sda_i[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2c1_scl_i[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_i2c1_sda_i[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_pdm_dmic_in[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_key_in_col_0[3]                        = {        0x1,      0x80,            0x0};
uint16_t IOMUXInputSelMap_key_in_col_1[3]                        = {        0x2,      0x100,           0x0};
uint16_t IOMUXInputSelMap_key_in_col_2[3]                        = {        0x4,      0x200,           0x0};
uint16_t IOMUXInputSelMap_key_in_col_3[3]                        = {        0x8,      0x2000,          0x0};
uint16_t IOMUXInputSelMap_key_in_col_4[3]                        = {        0x10,     0x4000,          0x0};
uint16_t IOMUXInputSelMap_key_in_col_5[3]                        = {        0x20,     0x8000,          0x0};
uint16_t IOMUXInputSelMap_key_in_col_6[3]                        = {        0x40,     0x0,             0x1};
uint16_t IOMUXInputSelMap_key_in_col_7[3]                        = {        0x80,     0x0,             0x2};
uint16_t IOMUXInputSelMap_key_in_col_8[3]                        = {        0x100,    0x0,             0x4};
uint16_t IOMUXInputSelMap_key_in_col_9[3]                        = {        0x200,    0x0,             0x8};
uint16_t IOMUXInputSelMap_key_in_col_10[3]                       = {        0x400,    0x0,             0x10};
uint16_t IOMUXInputSelMap_key_in_col_11[3]                       = {        0x800,    0x0,             0x20};
uint16_t IOMUXInputSelMap_key_in_col_12[3]                       = {        0x1000,   0x0,             0x40};
uint16_t IOMUXInputSelMap_key_in_col_13[3]                       = {        0x2000,   0x0,             0x80};
uint16_t IOMUXInputSelMap_key_in_col_14[3]                       = {        0x4000,   0x0,             0x100};
uint16_t IOMUXInputSelMap_key_in_col_15[3]                       = {        0x8000,   0x0,             0x200};
uint16_t IOMUXInputSelMap_key_in_col_16[3]                       = {        0x0,      0x1,             0x0};
uint16_t IOMUXInputSelMap_key_in_col_17[3]                       = {        0x0,      0x2,             0x0};
uint16_t IOMUXInputSelMap_key_in_col_18[3]                       = {        0x0,      0x20,            0x0};
uint16_t IOMUXInputSelMap_key_in_col_19[3]                       = {        0x0,      0x40,            0x0};
uint16_t IOMUXInputSelMap_spi2ahb_sclk[3]                        = {        0x8000,   0x0,             0x0};
uint16_t IOMUXInputSelMap_spi2ahb_cs[3]                          = {        0x0,      0x1,             0x0};
uint16_t IOMUXInputSelMap_spi2ahb_di[3]                          = {        0x0,      0x2,             0x0};
uint16_t IOMUXInputSelMap_qdec_phasea[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_qdec_phaseb[3]                         = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_qdec_index[3]                          = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_qdec_ext_in_clk[3]                     = {        0x208,    0x0,             0x0};
uint16_t IOMUXInputSelMap_qdec_timer_ext_in_a1[3]                = {        0x82,     0x0,             0x0};
uint16_t IOMUXInputSelMap_qdec_timer_ext_in_a2[3]                = {        0x104,    0x0,             0x0};
uint16_t IOMUXInputSelMap_qdec_timer_ext_in_b2[3]                = {        0x820,    0x0,             0x0};
uint16_t IOMUXInputSelMap_p_cap_in0[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_p_cap_in1[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_p_cap_in2[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_p_cap_in3[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_p_cap_in4[3]                           = {        0xFFFF,   0x8063,          0xC};
uint16_t IOMUXInputSelMap_p_cap_in5[3]                           = {        0xFFFF,   0x8063,          0xC};

void ClearIOMUXFuncSel(void)
{
    io_mux_ctrl_reg->io_mux_out_ctrl0.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl1.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl2.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl3.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl4.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl5.r = 0x0;
    io_mux_ctrl_reg->io_mux_out_ctrl6.r = 0x0;
}

void ClearIOMUXInputSel(void)
{
    io_mux_ctrl_reg->io_mux_in_ctrl0.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl1.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl2.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl3.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl4.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl5.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl6.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl7.r = 0xffffffff;
    io_mux_ctrl_reg->io_mux_in_ctrl9.r = 0xffffffff;
}

void SetGPIOFuncSel(uint8_t GPIOIdx, E_IOMUXFuncSel FuncSel)
{
    uint8_t i,j;
    uint8_t Flag;
    uint16_t Sel;

    Sel = 0;
    Flag = 0;

    if(FuncSel == IOMUXFuncSel_gpio){
        Sel = 0;
    }
    else if(FuncSel == IOMUXFuncSel_dbg){
        Sel = 1;
    }
    else{
        Sel = 2;
        for(i=0; i<9; i++){
            for(j=0; j<16; j++){
                if(((IOMUXFuncSelMap[GPIOIdx][i]>>j) & 0x1)==0x1) {
                    if(FuncSel == (i*16+j+2)) {
                        Flag = 1;
                        break;
                    }
                    Sel++;
                }
            }
            if(Flag==1){
                break;
            }
        }
    }

    switch (GPIOIdx) {
        case 0  : io_mux_ctrl_reg->io_mux_out_ctrl0.f.io0_func_sel  = Sel; break;
        case 1  : io_mux_ctrl_reg->io_mux_out_ctrl0.f.io1_func_sel  = Sel; break;
        case 2  : io_mux_ctrl_reg->io_mux_out_ctrl0.f.io2_func_sel  = Sel; break;
        case 3  : io_mux_ctrl_reg->io_mux_out_ctrl0.f.io3_func_sel  = Sel; break;
        case 4  : io_mux_ctrl_reg->io_mux_out_ctrl1.f.io4_func_sel  = Sel; break;
        case 5  : io_mux_ctrl_reg->io_mux_out_ctrl1.f.io5_func_sel  = Sel; break;
        case 6  : io_mux_ctrl_reg->io_mux_out_ctrl1.f.io6_func_sel  = Sel; break;
        case 7  : io_mux_ctrl_reg->io_mux_out_ctrl1.f.io7_func_sel  = Sel; break;
        case 8  : io_mux_ctrl_reg->io_mux_out_ctrl2.f.io8_func_sel  = Sel; break;
        case 9  : io_mux_ctrl_reg->io_mux_out_ctrl2.f.io9_func_sel  = Sel; break;
        case 10 : io_mux_ctrl_reg->io_mux_out_ctrl2.f.io10_func_sel = Sel; break;
        case 11 : io_mux_ctrl_reg->io_mux_out_ctrl2.f.io11_func_sel = Sel; break;
        case 12 : io_mux_ctrl_reg->io_mux_out_ctrl3.f.io12_func_sel = Sel; break;
        case 13 : io_mux_ctrl_reg->io_mux_out_ctrl3.f.io13_func_sel = Sel; break;
        case 14 : io_mux_ctrl_reg->io_mux_out_ctrl3.f.io14_func_sel = Sel; break;
        case 15 : io_mux_ctrl_reg->io_mux_out_ctrl3.f.io15_func_sel = Sel; break;
        case 16 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io16_func_sel = Sel; break;
        case 17 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io17_func_sel = Sel; break;
        case 18 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io18_func_sel = Sel; break;
        case 19 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io19_func_sel = Sel; break;
        case 20 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io20_func_sel = Sel; break;
        case 21 : io_mux_ctrl_reg->io_mux_out_ctrl4.f.io21_func_sel = Sel; break;
        case 22 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io22_func_sel = Sel; break;
        case 23 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io23_func_sel = Sel; break;
        case 24 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io24_func_sel = Sel; break;
        case 25 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io25_func_sel = Sel; break;
        case 26 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io26_func_sel = Sel; break;
        case 27 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io27_func_sel = Sel; break;
        case 28 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io28_func_sel = Sel; break;
        case 29 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io29_func_sel = Sel; break;
        case 30 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io30_func_sel = Sel; break;
        case 31 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io31_func_sel = Sel; break;
        case 32 : io_mux_ctrl_reg->io_mux_out_ctrl5.f.io32_func_sel = Sel; break;
        case 33 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io33_func_sel = Sel; break;
        case 34 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io34_func_sel = Sel; break;
        case 35 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io35_func_sel = Sel; break;
        case 36 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io36_func_sel = Sel; break;
        case 37 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io37_func_sel = Sel; break;
        case 38 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io38_func_sel = Sel; break;
        case 39 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io39_func_sel = Sel; break;
        case 40 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io40_func_sel = Sel; break;
        case 41 : io_mux_ctrl_reg->io_mux_out_ctrl6.f.io41_func_sel = Sel; break;
        default: break;
    }
}

uint8_t GetIOMUXInputSel(uint16_t Map[3], uint8_t PINIdx)
{
    uint8_t i,j;
    uint8_t Sel;
    Sel = 0;
    for(i=0; i<3; i++) {
        for(j=0; j<16; j++) {
            if(((Map[i]>>j)&0x1)==0x1){
                if((i*16+j)==PINIdx){
                    return Sel;
                }
                Sel++;
            }
        }
    }
    return Sel;
}

void IOMUXPullUp(uint8_t PINIdx)
{
    if(PINIdx<32){
        io_mux_ctrl_reg->io_ps_ctrl0.r = io_mux_ctrl_reg->io_ps_ctrl0.r | (0x1<<PINIdx);
        io_mux_ctrl_reg->io_pe_ctrl0.r = io_mux_ctrl_reg->io_pe_ctrl0.r | (0x1<<PINIdx);
    }
    else{
        io_mux_ctrl_reg->io_ps_ctrl1.r = io_mux_ctrl_reg->io_ps_ctrl1.r | (0x1<<(PINIdx-32));
        io_mux_ctrl_reg->io_pe_ctrl1.r = io_mux_ctrl_reg->io_pe_ctrl1.r | (0x1<<(PINIdx-32));
    }
}

void IOMUXPullDown(uint8_t PINIdx)
{
    if(PINIdx<32){
        io_mux_ctrl_reg->io_ps_ctrl0.r = io_mux_ctrl_reg->io_ps_ctrl0.r & (~(0x1<<PINIdx));
        io_mux_ctrl_reg->io_pe_ctrl0.r = io_mux_ctrl_reg->io_pe_ctrl0.r | (0x1<<PINIdx);
    }
    else{
        io_mux_ctrl_reg->io_ps_ctrl1.r = io_mux_ctrl_reg->io_ps_ctrl1.r & (~(0x1<<(PINIdx-32)));
        io_mux_ctrl_reg->io_pe_ctrl1.r = io_mux_ctrl_reg->io_pe_ctrl1.r | (0x1<<(PINIdx-32));
    }
}

void SetAnaPIN(uint8_t PINIdx)
{
  //set OE/IE = 0
  if(PINIdx<21){
    gpio0_reg->ChannelDir.r = gpio0_reg->ChannelDir.r & (~(0x1<<PINIdx));
    gpio0_reg->IOIE.r = gpio0_reg->IOIE.r & (~(0x1<<PINIdx));
  }
  else{
    gpio1_reg->ChannelDir.r = gpio1_reg->ChannelDir.r & (~(0x1<<(PINIdx-21)));
  }

  //set PE/AON_IE/AON_PE = 0
  if(PINIdx<32){
    io_mux_ctrl_reg->io_pe_ctrl0.r = io_mux_ctrl_reg->io_pe_ctrl0.r & (~(0x1<<PINIdx));
    aon2_ctrl_reg->aon1_reg4.r = aon2_ctrl_reg->aon1_reg4.r & (~(0x1<<PINIdx));//AON_IE
    aon2_ctrl_reg->aon1_reg7.r = aon2_ctrl_reg->aon1_reg7.r & (~(0x1<<PINIdx));//AON_PE
  }
  else{
    io_mux_ctrl_reg->io_pe_ctrl1.r = io_mux_ctrl_reg->io_pe_ctrl1.r & (~(0x1<<(PINIdx-32)));
    aon2_ctrl_reg->aon1_reg6.r = aon2_ctrl_reg->aon1_reg6.r & (~(0x1<<(PINIdx-32)));//AON_IE
    aon2_ctrl_reg->aon1_reg8.r = aon2_ctrl_reg->aon1_reg8.r & (~(0x1<<(PINIdx-32)));//AON_PE
  }
  SetGPIOFuncSel(PINIdx, IOMUXFuncSel_gpio);
}

void SetUart0PIN(uint8_t TxPIN, uint8_t RxPIN)
{
    SetGPIOFuncSel(TxPIN,  IOMUXFuncSel_uart0_txd);
    SetGPIOFuncSel(RxPIN,  IOMUXFuncSel_uart0_rxd);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.uart0_rxd_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart0_rxd, RxPIN);
}

void SetUart1PIN(uint8_t TxPIN, uint8_t RxPIN)
{
    SetGPIOFuncSel(TxPIN,  IOMUXFuncSel_uart1_txd);
    SetGPIOFuncSel(RxPIN,  IOMUXFuncSel_uart1_rxd);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.uart1_rxd_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart1_rxd, RxPIN);
}

void SetUart0PINFlowCtrl(uint8_t TxPIN, uint8_t RxPIN, uint8_t RTSPIN, uint8_t CTSPIN)
{
    SetGPIOFuncSel(TxPIN,  IOMUXFuncSel_uart0_txd);
    SetGPIOFuncSel(RxPIN,  IOMUXFuncSel_uart0_rxd);
    SetGPIOFuncSel(RTSPIN, IOMUXFuncSel_uart0_rts);
    SetGPIOFuncSel(CTSPIN, IOMUXFuncSel_uart0_cts);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.uart0_rxd_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart0_rxd, RxPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.uart0_cts_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart0_cts, CTSPIN);
}

void SetUart1PINFlowCtrl(uint8_t TxPIN, uint8_t RxPIN, uint8_t RTSPIN, uint8_t CTSPIN)
{
    SetGPIOFuncSel(TxPIN,  IOMUXFuncSel_uart1_txd);
    SetGPIOFuncSel(RxPIN,  IOMUXFuncSel_uart1_rxd);
    SetGPIOFuncSel(RTSPIN, IOMUXFuncSel_uart1_rts);
    SetGPIOFuncSel(CTSPIN, IOMUXFuncSel_uart1_cts);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.uart1_rxd_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart1_rxd, RxPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.uart1_cts_sel = GetIOMUXInputSel(IOMUXInputSelMap_uart1_cts, CTSPIN);
}

void SetI2C0PIN(uint8_t SclPIN, uint8_t SdaPIN)
{
    IOMUXPullUp(SclPIN);
    IOMUXPullUp(SdaPIN);
    SetGPIOFuncSel(SclPIN,  IOMUXFuncSel_i2c0_scl_o);
    SetGPIOFuncSel(SdaPIN,  IOMUXFuncSel_i2c0_sda_o);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.i2c0_scl_i_sel = GetIOMUXInputSel(IOMUXInputSelMap_i2c0_scl_i, SclPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.i2c0_sda_i_sel = GetIOMUXInputSel(IOMUXInputSelMap_i2c0_sda_i, SdaPIN);
}

void SetI2C1PIN(uint8_t SclPIN, uint8_t SdaPIN)
{
    IOMUXPullUp(SclPIN);
    IOMUXPullUp(SdaPIN);
    SetGPIOFuncSel(SclPIN,  IOMUXFuncSel_i2c1_scl_o);
    SetGPIOFuncSel(SdaPIN,  IOMUXFuncSel_i2c1_sda_o);
    io_mux_ctrl_reg->io_mux_in_ctrl4.f.i2c1_scl_i_sel = GetIOMUXInputSel(IOMUXInputSelMap_i2c1_scl_i, SclPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl5.f.i2c1_sda_i_sel = GetIOMUXInputSel(IOMUXInputSelMap_i2c1_sda_i, SdaPIN);
}

void SetKeyScannerPINRow(uint32_t RowIdx, uint32_t PINIdx)
{
    if(PINIdx<32){
        io_mux_ctrl_reg->io_pe_ctrl0.r = io_mux_ctrl_reg->io_pe_ctrl0.r & (~(0x1<<PINIdx));
    }
    else{
        io_mux_ctrl_reg->io_pe_ctrl1.r = io_mux_ctrl_reg->io_pe_ctrl1.r & (~(0x1<<(PINIdx-32)));
    }

    SetGPIOFuncSel(PINIdx,  (E_IOMUXFuncSel)(IOMUXFuncSel_key_out_row_0+RowIdx));
}

void SetKeyScannerPINCol(uint32_t ColIdx, uint32_t PINIdx)
{
    if(PINIdx<32){
        io_mux_ctrl_reg->io_ps_ctrl0.r = io_mux_ctrl_reg->io_ps_ctrl0.r & (~(0x1<<PINIdx));
        io_mux_ctrl_reg->io_pe_ctrl0.r = io_mux_ctrl_reg->io_pe_ctrl0.r | (0x1<<PINIdx);
    }
    else{
        io_mux_ctrl_reg->io_ps_ctrl1.r = io_mux_ctrl_reg->io_ps_ctrl1.r & (~(0x1<<(PINIdx-32)));
        io_mux_ctrl_reg->io_pe_ctrl1.r = io_mux_ctrl_reg->io_pe_ctrl1.r | (0x1<<(PINIdx-32));
    }

    SetGPIOFuncSel(PINIdx,  (E_IOMUXFuncSel)(IOMUXFuncSel_key_in_col_0+ColIdx));
    switch (ColIdx)
    {
    case 0  : io_mux_ctrl_reg->io_mux_in_ctrl5.f.key_in_col_0_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_0 , PINIdx); break;
    case 1  : io_mux_ctrl_reg->io_mux_in_ctrl5.f.key_in_col_1_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_1 , PINIdx); break;
    case 2  : io_mux_ctrl_reg->io_mux_in_ctrl5.f.key_in_col_2_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_2 , PINIdx); break;
    case 3  : io_mux_ctrl_reg->io_mux_in_ctrl6.f.key_in_col_3_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_3 , PINIdx); break;
    case 4  : io_mux_ctrl_reg->io_mux_in_ctrl6.f.key_in_col_4_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_4 , PINIdx); break;
    case 5  : io_mux_ctrl_reg->io_mux_in_ctrl6.f.key_in_col_5_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_5 , PINIdx); break;
    case 6  : io_mux_ctrl_reg->io_mux_in_ctrl6.f.key_in_col_6_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_6 , PINIdx); break;
    case 7  : io_mux_ctrl_reg->io_mux_in_ctrl6.f.key_in_col_7_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_7 , PINIdx); break;
    case 8  : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_8_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_8 , PINIdx); break;
    case 9  : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_9_sel  = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_9 , PINIdx); break;
    case 10 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_10_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_10, PINIdx); break;
    case 11 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_11_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_11, PINIdx); break;
    case 12 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_12_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_12, PINIdx); break;
    case 13 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_13_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_13, PINIdx); break;
    case 14 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_14_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_14, PINIdx); break;
    case 15 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_15_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_15, PINIdx); break;
    case 16 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_16_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_16, PINIdx); break;
    case 17 : io_mux_ctrl_reg->io_mux_in_ctrl7.f.key_in_col_17_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_17, PINIdx); break;
    case 18 : io_mux_ctrl_reg->io_mux_in_ctrl9.f.key_in_col_18_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_18, PINIdx); break;
    case 19 : io_mux_ctrl_reg->io_mux_in_ctrl9.f.key_in_col_19_sel = GetIOMUXInputSel(IOMUXInputSelMap_key_in_col_19, PINIdx); break;
    default: break;
    }
}

void SetQSPIPIN(uint8_t CSPIN, uint8_t CLKPIN, uint8_t MOSIPIN, uint8_t MISOPIN, uint8_t WPPIN, uint8_t HOLDPIN)
{
    SetGPIOFuncSel(CSPIN  , IOMUXFuncSel_spi0_csn_out);
    SetGPIOFuncSel(CLKPIN , IOMUXFuncSel_spi0_clk_out);
    SetGPIOFuncSel(MOSIPIN, IOMUXFuncSel_spi0_mosi_out);
    SetGPIOFuncSel(MISOPIN, IOMUXFuncSel_spi0_miso_out);
    SetGPIOFuncSel(WPPIN  , IOMUXFuncSel_spi0_wp_out);
    SetGPIOFuncSel(HOLDPIN, IOMUXFuncSel_spi0_hold_out);
    io_mux_ctrl_reg->io_mux_in_ctrl0.f.spi0_csn_in_sel  = GetIOMUXInputSel(IOMUXInputSelMap_spi0_csn_in , CSPIN  );
    io_mux_ctrl_reg->io_mux_in_ctrl0.f.spi0_clk_in_sel  = GetIOMUXInputSel(IOMUXInputSelMap_spi0_clk_in , CLKPIN );
    io_mux_ctrl_reg->io_mux_in_ctrl1.f.spi0_mosi_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi0_mosi_in, MOSIPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl1.f.spi0_miso_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi0_miso_in, MISOPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl1.f.spi0_wp_in_sel   = GetIOMUXInputSel(IOMUXInputSelMap_spi0_wp_in  , WPPIN  );
    io_mux_ctrl_reg->io_mux_in_ctrl0.f.spi0_hold_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi0_hold_in, HOLDPIN);
}

void SetAPBSPIPIN(uint8_t CSPIN, uint8_t CLKPIN, uint8_t MOSIPIN, uint8_t MISOPIN, uint8_t WPPIN, uint8_t HOLDPIN)
{
    SetGPIOFuncSel(CSPIN  , IOMUXFuncSel_spi1_csn_out);
    SetGPIOFuncSel(CLKPIN , IOMUXFuncSel_spi1_clk_out);
    SetGPIOFuncSel(MOSIPIN, IOMUXFuncSel_spi1_mosi_out);
    SetGPIOFuncSel(MISOPIN, IOMUXFuncSel_spi1_miso_out);
    SetGPIOFuncSel(WPPIN  , IOMUXFuncSel_spi1_wp_out);
    SetGPIOFuncSel(HOLDPIN, IOMUXFuncSel_spi1_hold_out);
    io_mux_ctrl_reg->io_mux_in_ctrl1.f.spi1_csn_in_sel  = GetIOMUXInputSel(IOMUXInputSelMap_spi1_csn_in , CSPIN  );
    io_mux_ctrl_reg->io_mux_in_ctrl1.f.spi1_clk_in_sel  = GetIOMUXInputSel(IOMUXInputSelMap_spi1_clk_in , CLKPIN );
    io_mux_ctrl_reg->io_mux_in_ctrl2.f.spi1_mosi_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi1_mosi_in, MOSIPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl2.f.spi1_miso_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi1_miso_in, MISOPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl2.f.spi1_wp_in_sel   = GetIOMUXInputSel(IOMUXInputSelMap_spi1_wp_in  , WPPIN  );
    io_mux_ctrl_reg->io_mux_in_ctrl2.f.spi1_hold_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_spi1_hold_in, HOLDPIN);
}

void SetI2SPIN(uint8_t BClkPIN, uint8_t LrClkPIN, uint8_t DinPIN, uint8_t DoutPIN)
{
    SetGPIOFuncSel(BClkPIN,  IOMUXFuncSel_i2s_bclk_o);
    SetGPIOFuncSel(LrClkPIN, IOMUXFuncSel_i2s_lrclk_o);
    SetGPIOFuncSel(DinPIN,   IOMUXFuncSel_i2s_din);
    SetGPIOFuncSel(DoutPIN,  IOMUXFuncSel_i2s_dout);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.i2s_bclk_i_sel  = GetIOMUXInputSel(IOMUXInputSelMap_i2s_bclk_i , BClkPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.i2s_lrclk_i_sel = GetIOMUXInputSel(IOMUXInputSelMap_i2s_lrclk_i, LrClkPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl3.f.i2s_din_sel     = GetIOMUXInputSel(IOMUXInputSelMap_i2s_din    , DinPIN);
}

void SetSWPIN(uint8_t SWCLKPIN, uint8_t SWDIOPIN)
{
    IOMUXPullUp(SWCLKPIN);
    IOMUXPullUp(SWDIOPIN);
    SetGPIOFuncSel(SWCLKPIN,  IOMUXFuncSel_sw_tck);
    SetGPIOFuncSel(SWDIOPIN,  IOMUXFuncSel_swdo);
    io_mux_ctrl_reg->io_mux_in_ctrl0.f.sw_tck_sel = GetIOMUXInputSel(IOMUXInputSelMap_sw_tck, SWCLKPIN);
    io_mux_ctrl_reg->io_mux_in_ctrl0.f.sw_tms_sel = GetIOMUXInputSel(IOMUXInputSelMap_sw_tms, SWDIOPIN);
}

void SetIRPIN(uint8_t DOUTPIN, uint8_t DINPIN)
{
    SetGPIOFuncSel(DOUTPIN,  IOMUXFuncSel_ir_datout);
    SetGPIOFuncSel(DINPIN,  IOMUXFuncSel_ir_datin);
    io_mux_ctrl_reg->io_mux_in_ctrl2.f.ir_datin_sel = GetIOMUXInputSel(IOMUXInputSelMap_ir_datin, DINPIN);
}

void SetPDMPIN(uint8_t CLKOUTPIN, uint8_t DINPIN)
{
    SetGPIOFuncSel(CLKOUTPIN,  IOMUXFuncSel_pdm_dmic_mclk);
    SetGPIOFuncSel(DINPIN,  IOMUXFuncSel_pdm_dmic_in);
    io_mux_ctrl_reg->io_mux_in_ctrl5.f.pdm_dmic_in_sel = GetIOMUXInputSel(IOMUXInputSelMap_pdm_dmic_in, DINPIN);
}

void SetLLEAntPIN(uint8_t PINNum, uint32_t AntPIN[8])
{
    uint8_t i;
    for(i=0; i<PINNum; i++){
        SetGPIOFuncSel(AntPIN[i], (E_IOMUXFuncSel)(IOMUXFuncSel_lle_ant_sw0+i));
    }
}

void SetSlpWkupSource(uint8_t Idx, uint8_t Pol, uint8_t Mode)
{
  //set pull
  if(Pol==0x0){
    IOMUXPullDown(Idx);
  }
  else{
    IOMUXPullUp(Idx);
  }

  //set gpio input
  if(Idx<21){
    gpio0_reg->ChannelDir.r = gpio0_reg->ChannelDir.r & (~(0x1<<Idx));
    gpio0_reg->IOIE.r = gpio0_reg->IOIE.r | (0x1<<Idx);
  }
  else{
    gpio1_reg->ChannelDir.r = gpio1_reg->ChannelDir.r & (~(0x1<<(Idx-21)));
    gpio1_reg->IOIE.r = gpio1_reg->IOIE.r | (0x1<<(Idx-21));
  }
  SetGPIOFuncSel(Idx, IOMUXFuncSel_gpio);

  //set mode
  if(Idx<16){
    aon2_ctrl_reg->sys_wakeup_mode0.r = aon2_ctrl_reg->sys_wakeup_mode0.r & (~(0x3<<(Idx*2))) | (Mode<<(Idx*2));
  }
  else if(Idx<32){
    aon2_ctrl_reg->sys_wakeup_mode1.r = aon2_ctrl_reg->sys_wakeup_mode1.r & (~(0x3<<((Idx-16)*2))) | (Mode<<((Idx-16)*2));
  }
  else {
    aon2_ctrl_reg->sys_wakeup_mode2.r = aon2_ctrl_reg->sys_wakeup_mode2.r & (~(0x3<<((Idx-32)*2))) | (Mode<<((Idx-32)*2));
  }

  //set PAD AON
  if(Idx<32){
    aon2_ctrl_reg->sys_wakeup_pol0.r = aon2_ctrl_reg->sys_wakeup_pol0.r & (~(0x1<<Idx)) | (Pol<<Idx);
    if((Mode==0x0) || (Mode==0x1) || (Mode==0x3)){
      if(Pol==0x0){
        aon2_ctrl_reg->aon1_reg9.r = aon2_ctrl_reg->aon1_reg9.r & (~(0x1<<Idx));
      }
      else if(Pol==0x1){
        aon2_ctrl_reg->aon1_reg9.r = aon2_ctrl_reg->aon1_reg9.r | (0x1<<Idx);
      }
    }
    else{
      if(Pol==0x0){
        aon2_ctrl_reg->aon1_reg9.r = aon2_ctrl_reg->aon1_reg9.r | (0x1<<Idx);
      }
      else if(Pol==0x1){
        aon2_ctrl_reg->aon1_reg9.r = aon2_ctrl_reg->aon1_reg9.r & (~(0x1<<Idx));
      }
    }
    aon2_ctrl_reg->aon1_reg7.r = aon2_ctrl_reg->aon1_reg7.r | (0x1<<Idx);
    aon2_ctrl_reg->aon1_reg4.r = aon2_ctrl_reg->aon1_reg4.r | (0x1<<Idx);
    aon2_ctrl_reg->sys_wakeup_en0.r = aon2_ctrl_reg->sys_wakeup_en0.r | (0x1<<Idx);
  }
  else{
    aon2_ctrl_reg->sys_wakeup_pol1.r = aon2_ctrl_reg->sys_wakeup_pol1.r & (~(0x1<<(Idx-32))) | (Pol<<(Idx-32));
    if((Mode==0x0) || (Mode==0x1) || (Mode==0x3)){
      if(Pol==0x0){
        aon2_ctrl_reg->aon1_reg10.r = aon2_ctrl_reg->aon1_reg10.r & (~(0x1<<(Idx-32)));
      }
      else if(Pol==0x1){
        aon2_ctrl_reg->aon1_reg10.r = aon2_ctrl_reg->aon1_reg10.r | (0x1<<(Idx-32));
      }
    }
    else{
      if(Pol==0x0){
        aon2_ctrl_reg->aon1_reg10.r = aon2_ctrl_reg->aon1_reg10.r | (0x1<<(Idx-32));
      }
      else if(Pol==0x1){
        aon2_ctrl_reg->aon1_reg10.r = aon2_ctrl_reg->aon1_reg10.r & (~(0x1<<(Idx-32)));
      }
    }
    aon2_ctrl_reg->aon1_reg8.r = aon2_ctrl_reg->aon1_reg8.r | (0x1<<(Idx-32));
    aon2_ctrl_reg->aon1_reg6.r = aon2_ctrl_reg->aon1_reg6.r | (0x1<<(Idx-32));
    aon2_ctrl_reg->sys_wakeup_en1.r = aon2_ctrl_reg->sys_wakeup_en1.r | (0x1<<(Idx-32));
  }
}

void SetDpslpWkupSource(uint8_t Idx, uint8_t Pol)
{
  //set pull
  if(Pol==0x0){
    IOMUXPullDown(Idx);
  }
  else{
    IOMUXPullUp(Idx);
  }

  //set gpio input
  if(Idx<21) {
    gpio0_reg->ChannelDir.r = gpio0_reg->ChannelDir.r & (~(0x1<<Idx));
    gpio0_reg->IOIE.r = gpio0_reg->IOIE.r | (0x1<<Idx);
  }
  else{
    gpio1_reg->ChannelDir.r = gpio1_reg->ChannelDir.r & (~(0x1<<(Idx-21)));
    gpio1_reg->IOIE.r = gpio1_reg->IOIE.r | (0x1<<(Idx-21));
  }
  SetGPIOFuncSel(Idx, IOMUXFuncSel_gpio);

  aon1_ctrl_reg->aon1_reg4.f.int_reg_ext_int_pol = Pol;

  if(Idx<32){
    aon2_ctrl_reg->sys_wakeup_en0.r = aon2_ctrl_reg->sys_wakeup_en0.r | (0x1<<Idx);
    if(Pol==0x0){
      aon2_ctrl_reg->sys_wakeup_pol0.r = aon2_ctrl_reg->sys_wakeup_pol0.r & (~(0x1<<Idx));
    }
    else{
      aon2_ctrl_reg->sys_wakeup_pol0.r = aon2_ctrl_reg->sys_wakeup_pol0.r | (0x1<<Idx);
    }
  }
  else {
    aon2_ctrl_reg->sys_wakeup_en1.r = aon2_ctrl_reg->sys_wakeup_en1.r | (0x1<<(Idx-32));
    if(Pol==0x0){
      aon2_ctrl_reg->sys_wakeup_pol1.r = aon2_ctrl_reg->sys_wakeup_pol1.r & (~(0x1<<(Idx-32)));
    }
    else{
      aon2_ctrl_reg->sys_wakeup_pol1.r = aon2_ctrl_reg->sys_wakeup_pol1.r | (0x1<<(Idx-32));
    }
  }
}

void SetGpioInput(uint8_t GPIOIdx)
{
  if(GPIOIdx<21){
    gpio0_reg->ChannelDir.r = gpio0_reg->ChannelDir.r&(~(0x1<<GPIOIdx));
    gpio0_reg->IOIE.r = gpio0_reg->IOIE.r | (0x1<<GPIOIdx);
  }
  else{
    gpio1_reg->ChannelDir.r = gpio1_reg->ChannelDir.r&(~(0x1<<(GPIOIdx-21)));
    gpio1_reg->IOIE.r = gpio1_reg->IOIE.r | (0x1<<(GPIOIdx-21));
  }
}

void SetGpioOutput(uint8_t GPIOIdx)
{
  if(GPIOIdx<21){
    gpio0_reg->ChannelDir.r = gpio0_reg->ChannelDir.r | (0x1<<GPIOIdx);
    gpio0_reg->IOIE.r = gpio0_reg->IOIE.r & (~(0x1<<GPIOIdx));
  }
  else{
    gpio1_reg->ChannelDir.r = gpio1_reg->ChannelDir.r | (0x1<<GPIOIdx);
    gpio1_reg->IOIE.r = gpio1_reg->IOIE.r & (~(0x1<<GPIOIdx));
  }
}

void SetGpioOutputValue(uint8_t GPIOIdx, uint8_t Value)
{
  if(Value==0x1){
    if(GPIOIdx<21){
      gpio0_reg->DoutSet.r = 0x1<<GPIOIdx;
    }
    else{
      gpio1_reg->DoutSet.r = 0x1<<(GPIOIdx-21);
    }
  }
  else if(Value==0x0){
    if(GPIOIdx<21){
      gpio0_reg->DoutClear.r = 0x1<<GPIOIdx;
    }
    else{
      gpio1_reg->DoutClear.r = 0x1<<(GPIOIdx-21);
    }
  }
}

uint8_t GetGpioInputValue(uint8_t GPIOIdx)
{
  if(GPIOIdx<21){
    return (gpio0_reg->DataIn.r>>GPIOIdx)&0x1;
  }
  else {
    return (gpio1_reg->DataIn.r>>(GPIOIdx-21))&0x1;
  }
}

////////////////////////////////////////
//  Cache
////////////////////////////////////////
void ICacheIni(uint8_t CacheSizeSel)
{
    icache_ctrl_reg->cache_cfg0.f.cache_size_sel = CacheSizeSel;
    icache_ctrl_reg->cmd_cfg2.r = (1UL<<31) | 0x4;
    icache_ctrl_reg->base_addr1.r = INGSIMD_A_FLASH_BASE;
    icache_ctrl_reg->base_addr2.r = INGSIMD_A_FLASH_BASE+INGSIMD_FLASH_SIZE;
    icache_ctrl_reg->base_addr3.r = 0xffffffff;
    icache_ctrl_reg->base_addr4.r = 0xffffffff;
    icache_ctrl_reg->base_addr5.r = 0xffffffff;
    icache_ctrl_reg->base_addr6.r = 0xffffffff;
    icache_ctrl_reg->base_addr7.r = 0xffffffff;
    //icache_ctrl_reg->int_en.r = 0x3;
    icache_ctrl_reg->cache_cfg0.f.hprot_byp = 0xf;
    icache_ctrl_reg->enable.f.blk_cache_en1 = 0x1;
}

void DCacheIni(uint8_t CacheSizeSel)
{
    dcache_ctrl_reg->cache_cfg0.f.cache_size_sel = CacheSizeSel;
    dcache_ctrl_reg->cmd_cfg2.r = (1UL<<31) | 0x4;
    dcache_ctrl_reg->base_addr1.r = INGSIMD_A_FLASH_BASE;
    dcache_ctrl_reg->base_addr2.r = INGSIMD_A_FLASH_BASE+INGSIMD_FLASH_SIZE;
    dcache_ctrl_reg->base_addr3.r = 0xffffffff;
    dcache_ctrl_reg->base_addr4.r = 0xffffffff;
    dcache_ctrl_reg->base_addr5.r = 0xffffffff;
    dcache_ctrl_reg->base_addr6.r = 0xffffffff;
    dcache_ctrl_reg->base_addr7.r = 0xffffffff;
    //dcache_ctrl_reg->int_en.r = 0x3;
    dcache_ctrl_reg->cache_cfg0.f.hprot_byp = 0xf;
    dcache_ctrl_reg->enable.f.blk_cache_en1 = 0x1;
}

void ICacheEnable(void)
{
    icache_ctrl_reg->enable.f.blk_cache_en1 = 0x1;
}

void ICacheDisable(void)
{
    icache_ctrl_reg->enable.f.blk_cache_en1 = 0x0;
}

void ICacheFlush(void)
{
    icache_ctrl_reg->cmd_cfg2.r = (1UL<<31) | 0x4;
}

void DCacheEnable(void)
{
    dcache_ctrl_reg->enable.f.blk_cache_en1 = 0x1;
}

void DCacheDisable(void)
{
    dcache_ctrl_reg->enable.f.blk_cache_en1 = 0x0;
}

void DCacheFlush(void)
{
    dcache_ctrl_reg->cmd_cfg2.r = (1UL<<31) | 0x4;
}

////////////////////////////////////////
//  PAD
////////////////////////////////////////
void Aon1IOLatch(void)
{
    aon1_ctrl_reg->aon1_reg4.f.reg_aon1_io_latch = 0x1;
    while(aon2_ctrl_reg->pwr_ctrl_status0.f.aon1_reg_latch_en == 0x0);
}

void Aon1IORelease(void)
{
    aon1_ctrl_reg->aon1_reg4.f.reg_aon1_io_latch = 0x0;
    while(aon2_ctrl_reg->pwr_ctrl_status0.f.aon1_reg_latch_en == 0x1);
}

void Aon2IORetentionRelease(void)
{
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_ds = 0x0;
    nop(10);
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_latch = 0x0;
    nop(10);
}

void Aon2IORetention(void)
{
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_latch = 0x1;
    nop(10);
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_ds = 0x1;
    nop(10);
}

void Aon2IOHighZRelease(void)
{
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_latch = 0x0;
    nop(10);
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_ds = 0x0;
    nop(10);
}

void Aon2IOHighZ(void)
{
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_ds = 0x1;
    nop(10);
    aon2_ctrl_reg->aon2_sleep_ctrl_1.f.io_latch = 0x1;
    nop(10);
}

////////////////////////////////////////
//  UART
////////////////////////////////////////
void UartInit(volatile UART_REG_H* uart_reg, uint32_t FlowCtrl, uint32_t ClkFreq, uint32_t BaudRate)
{
    uint32_t BaudRateDiv;
    uint32_t BaudIntDiv;
    uint32_t BaudFracDiv;

    BaudRateDiv = (ClkFreq<<3)/BaudRate;
    BaudIntDiv = BaudRateDiv>>7;
    BaudRateDiv -= (BaudIntDiv << 7);
    BaudFracDiv =  (BaudRateDiv + 1) / 2;

    uart_reg->UARTIMSC.f.RXIM = 0x1;
    uart_reg->UARTIMSC.f.TXIM = 0x1;
    uart_reg->UARTIBRD.f.BAUD_DIVINT = BaudIntDiv;
    uart_reg->UARTFBRD.f.BAUD_DIVFRAC = BaudFracDiv;
    uart_reg->UARTLCR_H.f.FEN = 0x1;
    uart_reg->UARTLCR_H.f.WLEN = 0x3;
    uart_reg->UARTIFLS.f.TXIFLSEL = 0x0;
    uart_reg->UARTIFLS.f.RXIFLSEL = 0x1;
    if(FlowCtrl&0x1){
        uart_reg->UARTCR.f.RTSEn = 0x1;
        uart_reg->UARTCR.f.CTSEn = 0x1;
    }
    uart_reg->UARTCR.f.UARTEN = 0x1;
    uart_reg->UARTCR.f.TXE = 0x1;
    uart_reg->UARTCR.f.RXE = 0x1;
}

void UartTx(volatile UART_REG_H* uart_reg, uint8_t ch)
{
  while(uart_reg->UARTFR.f.TXFF == 1);
  uart_reg->UARTDR.f.DATA = ch;
}

uint8_t UartRx(volatile UART_REG_H* uart_reg)
{
  while(uart_reg->UARTFR.f.RXFE == 1);
  return (uint8_t)uart_reg->UARTDR.f.DATA;
}

char UartString[100];
char* UartRxString(volatile UART_REG_H* uart_reg)
{
  uint8_t i;
  i=0;
  while(1){
    UartString[i] = UartRx(uart_reg);
    if(UartString[i] == '\n'){
      break;
    }
    i++;
  }
  return UartString;
}

void WaitKeyboard(void)
{
  printf("\n");
  printf("Press keyboard to Continue");
  printf("\n");
  UartRx(uart0_reg);
}

////////////////////////////////////////
//  TIMER
////////////////////////////////////////
void DelayS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt)
{
  timer_reg->ChEn.r = 0x0;
  timer_reg->Ch0Ctrl.r = (0x1<<3) | 0x1;//ChMode 32-bit timer, ChClk APB clock
  timer_reg->Ch0Reload.r = Cnt * Freq;
  timer_reg->IntEn.r = 0x1;
  timer_reg->ChEn.r = 0x1;
  while(timer0_reg->IntSt.r==0x0);
  timer0_reg->ChEn.r = 0x0;
  timer0_reg->IntSt.r = 0x1;
}

void DelayMS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt)
{
  timer_reg->ChEn.r = 0x0;
  timer_reg->Ch0Ctrl.r = (0x1<<3) | 0x1;//ChMode 32-bit timer, ChClk APB clock
  timer_reg->Ch0Reload.r = Cnt * (Freq/1000);
  timer_reg->IntEn.r = 0x1;
  timer_reg->ChEn.r = 0x1;
  while(timer0_reg->IntSt.r==0x0);
  timer0_reg->ChEn.r = 0x0;
  timer0_reg->IntSt.r = 0x1;
}

void DelayUS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt)
{
  timer_reg->ChEn.r = 0x0;
  timer_reg->Ch0Ctrl.r = (0x1<<3) | 0x1;//ChMode 32-bit timer, ChClk APB clock
  timer_reg->Ch0Reload.r = Cnt * (Freq/1000000);
  timer_reg->IntEn.r = 0x1;
  timer_reg->ChEn.r = 0x1;
  while(timer0_reg->IntSt.r==0x0);
  timer0_reg->ChEn.r = 0x0;
  timer0_reg->IntSt.r = 0x1;
}

////////////////////////////////////////
//  Clock
////////////////////////////////////////
void Sel32KSource(uint8_t Sel)
{
  aon1_ctrl_reg->aon1_reg0.f.int_reg_en_rc32k = 0x1;
  aon1_ctrl_reg->aon1_reg0.f.int_reg_io_osc_en = 0x1;
  aon1_ctrl_reg->aon1_reg0.f.int_reg_sel_clk_32k_source = Sel;
  if(Sel==0x0){
    while(sys_ctrl_reg->vld_clk_rd.f.vld_clk0_clk_32k_sw==0x0);
    aon1_ctrl_reg->aon1_reg0.f.int_reg_io_osc_en = 0x0;
  }
  else{
    while(sys_ctrl_reg->vld_clk_rd.f.vld_clk1_clk_32k_sw==0x0);
    aon1_ctrl_reg->aon1_reg0.f.int_reg_en_rc32k = 0x0;
  }
}

////////////////////////////////////////
//  MISC
////////////////////////////////////////
void LLECalibration(void)
{
  lle_reg->LLE_TBCU_CALIB.f.calib_ena = 0x1;
  while(lle_reg->LLE_TBCU_CALIB.f.calib_ena==0x1);
  nop(1000); //delay at least 2 32K cycle
}

void nop(uint32_t n)
{
  uint32_t i;
  for(i=0;i<n;i++){
    __asm volatile ("nop");
  }
}

typedef  void (*pFunction)(void);
void jump(int addr)
{
  uint32_t J;
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = *(uint32_t*)(addr);
  J =  *(uint32_t*) (addr + 4);
  ((pFunction)(J))();
  while(1);
}
#if 0

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// polling
void sendchar(volatile UART_REG_H* pBase, uint8_t ch)
{
  while(pBase->UARTFR.f.TXFF == 1);
  pBase->UARTDR.f.DATA = ch;

}

//??'?�?????g?
#if 0 //???�????S????????target????????'??USE microLIB
#pragma import(__use_no_semihosting)
struct __FILE  
{  
  int handle;  
};  
FILE __stdout;  

_sys_exit(int x)  
{  
  x = x;  
}
#endif

PUTCHAR_PROTOTYPE
{
//   /* Place your implementation of fputc here */
//   /* e.g. write a character to the USART */
//   //USART_SendData(USART1,(u8)ch);
#if(USE_UART1)
sendchar(uart1_reg, (uint8_t)ch);
#else
sendchar(uart0_reg, (uint8_t)ch);
#endif

//   /* Loop until the end of transmission */
//   //while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

   return ch;
}
#endif
#endif
