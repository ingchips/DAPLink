#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#include "stdint.h"
#include "IO_Config.h"
#include "swd_spi.h"

#ifndef   __STATIC_INLINE
#define __STATIC_INLINE                        static inline
#endif
#ifndef   __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
#endif
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

extern uint32_t SystemCoreClock;

// Processor Clock (used for SWD/JTAG clock calculation)
#define CPU_CLOCK               (SystemCoreClock / 10)

// I/O Port write cycles
#define IO_PORT_WRITE_CYCLES    1U

// SWD/JTAG mode support
#define DAP_SWD                 1
#define DAP_JTAG                0
#define DAP_JTAG_DEV_CNT        8U
#define DAP_DEFAULT_PORT        1U    // SWD
#define DAP_DEFAULT_SWJ_CLOCK   10000000U

// Packet size: 64 for Full-Speed USB
#define DAP_PACKET_SIZE         64U
#define DAP_PACKET_COUNT        8U

// SWO (not available)
#define SWO_UART                0
#define SWO_UART_DRIVER         0
#define SWO_UART_MAX_BAUDRATE   10000000U
#define SWO_MANCHESTER          0
#define SWO_BUFFER_SIZE         512U
#define SWO_STREAM              0
#define TIMESTAMP_CLOCK         0U

// DAP UART (USB COM port available)
#define DAP_UART                0
#define DAP_UART_DRIVER         1
#define DAP_UART_RX_BUFFER_SIZE 4096U
#define DAP_UART_TX_BUFFER_SIZE 4096U
#define DAP_UART_USB_COM_PORT   1

// Target
#define TARGET_FIXED            0

//**************************************************************************************************
// Hardware I/O Pin Access
// SPI-based SWD: all PIN_* macros are stubs since SWD is handled by SPI hardware
//**************************************************************************************************

__STATIC_INLINE void PORT_JTAG_SETUP(void) {}
__STATIC_INLINE void PORT_SWD_SETUP(void)  { swd_spi_init(); }
__STATIC_INLINE void PORT_OFF(void)
{
    PINCTRL_SetPadMux(SWCLK_PIN, IO_SOURCE_GPIO);
    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GPIO);
    GIO_SetDirection(SWCLK_PIN, GIO_DIR_INPUT);
    GIO_SetDirection(SWDIO_PIN, GIO_DIR_INPUT);
    PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);
}

// SWCLK/TCK
__STATIC_FORCEINLINE uint32_t PIN_SWCLK_TCK_IN(void)  { return GIO_ReadValue(SWCLK_PIN); }
__STATIC_FORCEINLINE void     PIN_SWCLK_TCK_SET(void) {}
__STATIC_FORCEINLINE void     PIN_SWCLK_TCK_CLR(void) {}

// SWDIO/TMS
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_TMS_IN(void)  { return GIO_ReadValue(SWDIO_PIN); }
__STATIC_FORCEINLINE void     PIN_SWDIO_TMS_SET(void) {}
__STATIC_FORCEINLINE void     PIN_SWDIO_TMS_CLR(void) {}

// SWDIO
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_IN(void)      { return GIO_ReadValue(SWDIO_PIN); }
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT(uint32_t bit) { (void)bit; }
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT_ENABLE(void)  {}
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT_DISABLE(void) {}

// TDI
__STATIC_FORCEINLINE uint32_t PIN_TDI_IN(void)  { return 0; }
__STATIC_FORCEINLINE void     PIN_TDI_OUT(uint32_t bit) { (void)bit; }

// TDO
__STATIC_FORCEINLINE uint32_t PIN_TDO_IN(void) { return 0; }

// nTRST
__STATIC_FORCEINLINE uint32_t PIN_nTRST_IN(void)  { return 0; }
__STATIC_FORCEINLINE void     PIN_nTRST_OUT(uint32_t bit) { (void)bit; }

// nRESET
__STATIC_FORCEINLINE uint32_t PIN_nRESET_IN(void)  { return GIO_ReadValue(nRESET_PIN); }
__STATIC_FORCEINLINE void     PIN_nRESET_OUT(uint32_t bit)
{
    PINCTRL_SetPadMux(nRESET_PIN, IO_SOURCE_GPIO);
    PINCTRL_Pull(nRESET_PIN, PINCTRL_PULL_UP);
    if (bit) {
        GIO_SetDirection(nRESET_PIN, GIO_DIR_INPUT);
    } else {
        swd_target_reset_aircr();
        GIO_WriteValue(nRESET_PIN, 0U);
        GIO_SetDirection(nRESET_PIN, GIO_DIR_OUTPUT);
    }
}

//**************************************************************************************************
// LEDs
//**************************************************************************************************

__STATIC_INLINE void LED_CONNECTED_OUT(uint32_t bit)
{
    if (bit) {
        APB_GPIO0->DoutSet = 1 << LED_CONNECTED_PIN;
    } else {
        APB_GPIO0->DoutClear = 1 << LED_CONNECTED_PIN;
    }
}

__STATIC_INLINE void LED_RUNNING_OUT(uint32_t bit) { (void)bit; }

//**************************************************************************************************
// Timestamp
//**************************************************************************************************

__STATIC_INLINE uint32_t TIMESTAMP_GET(void) { return 0; }

//**************************************************************************************************
// Initialization
//**************************************************************************************************

__STATIC_INLINE void DAP_SETUP(void)
{
    uint32_t mask = 1 << LED_CONNECTED_PIN;
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO1)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0));

    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << LED_CONNECTED_PIN);
    APB_GPIO0->IOIE  = (APB_GPIO0->IOIE  & (~mask)) | (0 << LED_CONNECTED_PIN);
    APB_GPIO0->DoutClear = 1 << LED_CONNECTED_PIN;
}

__STATIC_INLINE uint32_t RESET_TARGET(void)
{
    return swd_target_reset_aircr();
}

//**************************************************************************************************
// String functions (for CMSIS-DAP identification)
//**************************************************************************************************

__STATIC_INLINE uint8_t DAP_GetVendorString(char *str)   { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetProductString(char *str)  { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetSerNumString(char *str)   { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetTargetDeviceVendorString(char *str) { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetTargetDeviceNameString(char *str)   { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetTargetBoardVendorString(char *str)  { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetTargetBoardNameString(char *str)    { (void)str; return 0; }
__STATIC_INLINE uint8_t DAP_GetProductFirmwareVersionString(char *str) { (void)str; return 0; }

#endif
