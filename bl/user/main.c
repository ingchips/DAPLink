#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ingsoc.h"
#include "sys_init.h"
#include "vfs_manager.h"


typedef void (*pv_t)(void);
pv_t pv;

static int app_vector_is_valid(uint32_t user_sp, uint32_t user_pc)
{
    uint32_t reset_addr = user_pc & ~1UL;

    return ((user_sp & 0x7UL) == 0) &&
           (user_sp >= APP_RAM_START) && (user_sp <= APP_RAM_END) &&
           ((user_pc & 1UL) != 0) &&
           (reset_addr >= APP_ADDR) && (reset_addr < APP_END_ADDR);
}

static int jump_to_app(void)
{
    uint32_t user_sp;
    uint32_t user_pc;

    user_sp = *(volatile uint32_t *) (APP_ADDR + 0x0000);
    user_pc = *(volatile uint32_t *) (APP_ADDR + 0x0004);

    if (!app_vector_is_valid(user_sp, user_pc)) {
        return 0;
    }

    pv = (pv_t)user_pc;
    __disable_irq();
    __set_MSP(user_sp);
    __set_PSP(user_sp);
    __enable_irq();
    pv();
    return 1;
}

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(APB_UART0) == 1);
    UART_SendData(APB_UART0, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(PRINT_PORT, &config, 0);
}

void init_memory(void)
{
    SYSCTRL_CacheControl(SYSCTRL_MEM_BLOCK_AS_CACHE, SYSCTRL_MEM_BLOCK_AS_SYS_MEM);
}


extern volatile uint8_t reset_flag;
int main(void)
{
    volatile uint32_t timeout = 1000;
    uint8_t reset_cnt = 0;
    SysInit();
//    jump_to_app();
    
    *(uint32_t*)(0x40102010) &= ~0x80;
    SYSCTRL_ClearClkGate(SYSCTRL_ClkGate_APB_PinCtrl);
    
    #if USE_RX_PIN
    APB_PINCTRL->OUT_CTRL[0] &= ~(0x7f<<14);
    APB_PINCTRL->IN_CTRL[3] |= 0x1f<<15;
    #endif
    
    PINCTRL_Pull(BOOT_KEY, PINCTRL_PULL_DOWN);
    while(timeout--);
    SYSCTRL_ClearClkGate(SYSCTRL_ClkGate_APB_GPIO0);
    SYSCTRL_ClearClkGate(SYSCTRL_ClkGate_APB_GPIO1);
    GIO_SetDirection(BOOT_KEY, GIO_DIR_INPUT);
    

    if(!GIO_ReadValue(BOOT_KEY))
    {
        jump_to_app();
    }

    SYSCTRL_SelectUartClk(UART_PORT_0, SYSCTRL_CLK_HCLK);
    config_uart(SYSCTRL_GetHClk(), 921600);

    uint32_t pull = SYSCTRL_GetPLLClk();
    printf("data:%d\r\n",pull);
    
    void msc_ram_init(uint8_t busid, uintptr_t reg_base);
    
    msc_ram_init(0, AHB_USB_BASE);

    while(1)
    {
        timeout = 10000;
        vfs_mngr_periodic(1000);
        while(timeout--);
        if(reset_flag)
        {
            reset_cnt++;
            if(reset_cnt > 3)
                system_reset();
        }
        else
                reset_cnt = 0;
    }
}
