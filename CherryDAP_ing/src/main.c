#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "swd_spi.h"
#include "platform.h"
#include "dap_main.h"
#include "eflash.h"

//**************************************************************************************************
// System configuration
//**************************************************************************************************

uint32_t SystemCoreClock;

// Cache initialization (called from startup)
void init_memory(void)
{
    SYSCTRL_CacheControl(SYSCTRL_MEM_BLOCK_AS_CACHE, SYSCTRL_MEM_BLOCK_AS_SYS_MEM);
}

void assert_failed(const char *file, int line)
{
    printf("ASSERT: %s:%d\n", file, line);
    for (;;);
}

//**************************************************************************************************
// System initialization (based on DAPLink_End sdk.c)
//**************************************************************************************************

static void SysInit(void)
{
    if (aon2_ctrl_reg->pwr_ctrl_status0.f.boot_power_up == 0x1U) {
        aon1_ctrl_reg->aon1_reg3.f.reg_boot_pin_clr = 0x1U;
        aon1_ctrl_reg->aon1_boot.r = ((0x1UL << 0)  |
                                      (0x1UL << 1)  |
                                      (0x1UL << 2)  |
                                      (70UL  << 5)  |
                                      (0x1UL << 13) |
                                      (3UL   << 14) |
                                      (0x1UL << 18) |
                                      (2UL   << 19) |
                                      (0x1UL << 23) |
                                      (0x2UL << 24) |
                                      (0x1UL << 27) |
                                      (7UL   << 29));
        NVIC_SystemReset();
    }

    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO1)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0)
                            | (1 << SYSCTRL_ITEM_APB_SPI1));

    SYSCTRL_ConfigPLLClk(5,70,1);

    (void)flash_prepare_factory_data();
    (void)SYSCTRL_Init();

    extern uint32_t __Vectors;
    SCB->VTOR = (uint32_t)&__Vectors;

    SystemCoreClock = SYSCTRL_GetPLLClk();
}

//**************************************************************************************************
// CherryDAP task
//**************************************************************************************************

static void chry_dap_task(void *pdata)
{
    (void)pdata;

    // Initialize SPI-based SWD
    swd_spi_init();

    // Initialize CherryDAP: USB device + DAP protocol
    chry_dap_init(0, AHB_USB_BASE);

    printf("CherryDAP initialized\n");

    // Main loop: process DAP commands
    while (1) {
        platform_uart_poll();
        platform_led_poll();
        chry_dap_handle();
        chry_dap_usb2uart_handle();
//        vTaskDelay(1);
    }
}

//**************************************************************************************************
// Application entry point (based on DAPLink_End main_interface.c)
//**************************************************************************************************

int main(void)
{
    // Initialize system (PLL, clocks, vector table)
    SysInit();
    SysTick_Config(configCPU_CLOCK_HZ/configTICK_RATE_HZ);
    // Initialize platform (UART, LEDs)
    platform_uart_init();
    led_init();

    printf("CherryDAP ing916 starting...\n");

    // Create CherryDAP task
    xTaskCreate(chry_dap_task,
                "chry_dap",
                configMINIMAL_STACK_SIZE * 4,
                NULL,
                (configMAX_PRIORITIES - 1),
                NULL);

    // Start FreeRTOS scheduler (never returns)
    vTaskStartScheduler();

    // Should never reach here
    for (;;) {}
}
