#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "ingsoc.h"
#include "soc.h"

// SWD Pins
#define SWCLK_PIN               GIO_GPIO_7
#define SWDIO_PIN               GIO_GPIO_8

// nRESET Pin
#define nRESET_PIN              GIO_GPIO_9

// SPI pins (SPI0 used for hardware SWD)
#define SPI_MIC_CLK             GIO_GPIO_7   // = SWCLK_PIN
#define SPI_MIC_MOSI            GIO_GPIO_8   // = SWDIO_PIN (bidirectional)

// Debug UART
#define PIN_UART_RX             GIO_GPIO_11
#define PIN_UART_TX             GIO_GPIO_12

// CDC UART (for USB-to-UART bridge)
#define PIN_CDC_UART_RX         GIO_GPIO_11
#define PIN_CDC_UART_TX         GIO_GPIO_12
#define CDC_UART_BASE           APB_UART1
#define CDC_UART_IRQ            IRQn_Uart1

// Status LEDs
#define LED_USB_PIN             GIO_GPIO_18
#define LED_SWD_PIN             GIO_GPIO_10
#define LED_CDC_PIN             GIO_GPIO_15
#define LED_MSC_PIN             GIO_GPIO_14

// CMSIS-DAP configuration compatibility
#define LED_CONNECTED_PIN       LED_USB_PIN

// Private target-board controls carried by CDC SET_CONTROL_LINE_STATE
#define PIN_UART_RTS            GIO_GPIO_19
#define PIN_UART_DTR            GIO_GPIO_20

// SPI hardware peripheral for SWD
#define SPI_MODE                AHB_SSP0
#define SPI_PORT                SPI_PORT_0

#endif
