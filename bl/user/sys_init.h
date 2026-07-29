#ifndef SYS_INIT_H
#define SYS_INIT_H

#define USE_RX_PIN  1

#if USE_RX_PIN
#define BOOT_KEY    GIO_GPIO_2
#else
#define BOOT_KEY    GIO_GPIO_21
#endif
#define APP_ADDR        0x0200F000UL
#define APP_END_ADDR    0x02200000UL
#define APP_RAM_START   0x20000000UL
#define APP_RAM_END     0x20008000UL
#define PRINT_PORT    APB_UART0

void SysInit(void);
void system_reset(void);

#endif //SYS_INIT_H
