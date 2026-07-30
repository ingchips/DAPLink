#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "IO_Config.h"
#include "ingsoc.h"
#include "peripheral_uart.h"
#include "peripheral_sysctrl.h"
#include "peripheral_pinctrl.h"
#include "chry_ringbuffer.h"
#include "usbd_cdc.h"
#include "dap_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ram_code.h"

//**************************************************************************************************
// Debug UART (UART0) - for printf output
//**************************************************************************************************

#define PRINT_PORT  APB_UART0
#define UART_TX_FIFO_SIZE 32U

static volatile uint32_t cdc_rx_overflow;
static volatile uint16_t cdc_control_line_state;

typedef struct {
    volatile uint32_t requests;
    uint32_t handled;
    volatile uint8_t value;
} activity_led_t;

static activity_led_t dap_activity_led;
static activity_led_t cdc_activity_led;

uint32_t cb_putc(char *c, void *dummy)
{
    (void)dummy;
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    cb_putc((char *)&ch, NULL);
    return ch;
}

//**************************************************************************************************
// CDC UART (UART1) - for USB-to-UART bridge
//**************************************************************************************************

extern chry_ringbuffer_t g_uartrx;
extern chry_ringbuffer_t g_usbrx;
extern volatile uint8_t config_uart_transfer;

static void config_cdc_uart(const struct cdc_line_coding *line_coding)
{
    UART_sStateStruct config;
    uint8_t data_bits = line_coding->bDataBits;

    if ((data_bits < 5U) || (data_bits > 8U)) {
        data_bits = 8U;
    }

    config.word_length       = (UART_eWLEN)(data_bits - 5U);
    switch (line_coding->bParityType) {
        case 1U: config.parity = UART_PARITY_ODD_PARITY;  break;
        case 2U: config.parity = UART_PARITY_EVEN_PARITY; break;
        case 3U: config.parity = UART_PARITY_FIX_ONE;     break;
        case 4U: config.parity = UART_PARITY_FIX_ZERO;    break;
        default: config.parity = UART_PARITY_NOT_CHECK;   break;
    }
    config.fifo_enable       = 1;
    config.two_stop_bits     = (line_coding->bCharFormat == 0U) ? 0U : 1U;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = SYSCTRL_GetHClk();
    config.BaudRate          = line_coding->dwDTERate ? line_coding->dwDTERate : 115200U;

    NVIC_DisableIRQ(CDC_UART_IRQ);
    uart_reset(CDC_UART_BASE);
    CDC_UART_BASE->IntClear = CDC_UART_BASE->IntRaw;
    apUART_Initialize(CDC_UART_BASE, &config,
                      (1UL << bsUART_RECEIVE_INTENAB) |
                      (1UL << bsUART_TIMEOUT_INTENAB) |
                      UART_INTBIT_ERROR);
    NVIC_ClearPendingIRQ(CDC_UART_IRQ);
    /* Keep UART above USB (priority 2) so sustained USB traffic cannot starve RX. */
    NVIC_SetPriority(CDC_UART_IRQ, 1U);
    NVIC_EnableIRQ(CDC_UART_IRQ);
}

void platform_uart_init(void)
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_UART0)
                            | (1 << SYSCTRL_ITEM_APB_UART1)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0));

    // Debug UART0
    SYSCTRL_SelectUartClk(UART_PORT_0, SYSCTRL_CLK_HCLK);
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
        config.ClockFrequency    = SYSCTRL_GetHClk();
        config.BaudRate          = 921600;
        apUART_Initialize(PRINT_PORT, &config, 0);
    }

    // CDC UART1
    SYSCTRL_SelectUartClk(UART_PORT_1, SYSCTRL_CLK_HCLK);
    PINCTRL_SelUartIn(UART_PORT_1, PIN_CDC_UART_RX, IO_NOT_A_PIN);
    PINCTRL_SetPadMux(PIN_CDC_UART_TX, IO_SOURCE_UART1_TXD);
    PINCTRL_Pull(PIN_CDC_UART_RX, PINCTRL_PULL_UP);

    PINCTRL_SetPadMux(PIN_UART_RTS, IO_SOURCE_GPIO);
    PINCTRL_SetPadMux(PIN_UART_DTR, IO_SOURCE_GPIO);
    platform_cdc_control_reset();

    /* CDC hosts are allowed to omit SET_LINE_CODING when opening at the
     * device default. Initialize UART1 so 115200/8N1 works immediately. */
    {
        struct cdc_line_coding default_line_coding = {
            115200U, 0U, 0U, 8U
        };
        config_cdc_uart(&default_line_coding);
    }
}

//**************************************************************************************************
// CDC UART callbacks (called by CherryDAP)
//**************************************************************************************************

static void apply_cdc_control_line_state(uint16_t state);

void chry_dap_usb2uart_uart_config_callback(struct cdc_line_coding *line_coding)
{
    config_cdc_uart(line_coding);
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    uint32_t state;

    (void)busid;
    (void)intf;

    state = __get_PRIMASK();
    __disable_irq();
    if (dtr) {
        cdc_control_line_state |= 0x01U;
    } else {
        cdc_control_line_state &= (uint16_t)~0x01U;
    }
    apply_cdc_control_line_state(cdc_control_line_state);
    __DMB();
    if (state == 0U) {
        __enable_irq();
    }
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    uint32_t state;

    (void)busid;
    (void)intf;

    state = __get_PRIMASK();
    __disable_irq();
    if (rts) {
        cdc_control_line_state |= 0x02U;
    } else {
        cdc_control_line_state &= (uint16_t)~0x02U;
    }
    apply_cdc_control_line_state(cdc_control_line_state);
    __DMB();
    if (state == 0U) {
        __enable_irq();
    }
}

static void apply_cdc_control_line_state(uint16_t state)
{
    if ((state & 0x02U) != 0U) {
        GIO_WriteValue(PIN_UART_RTS, 1U);
        GIO_SetDirection(PIN_UART_RTS, GIO_DIR_OUTPUT);
    } else {
        GIO_SetDirection(PIN_UART_RTS, GIO_DIR_INPUT);
        GIO_WriteValue(PIN_UART_RTS, 0U);
    }

    if ((state & 0x01U) != 0U) {
        GIO_WriteValue(PIN_UART_DTR, 0U);
        GIO_SetDirection(PIN_UART_DTR, GIO_DIR_OUTPUT);
    } else {
        GIO_SetDirection(PIN_UART_DTR, GIO_DIR_INPUT);
        GIO_WriteValue(PIN_UART_DTR, 1U);
    }
}

void platform_cdc_control_reset(void)
{
    cdc_control_line_state = 0U;
    apply_cdc_control_line_state(0U);
}

RAM_CODE void chry_dap_usb2uart_uart_send_bydma(uint8_t *data, uint16_t len)
{
    uint32_t count;
    uint32_t state;
    uint8_t byte;

    (void)data;
    (void)len;

    state = __get_PRIMASK();
    __disable_irq();
    if ((CDC_UART_BASE->IntMask & (1UL << bsUART_TRANSMIT_INTENAB)) == 0U) {
        count = chry_ringbuffer_get_used(&g_usbrx);
        if (count < 4U) {
            while ((count != 0U) &&
                   (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 0U)) {
                chry_ringbuffer_read_byte(&g_usbrx, &byte);
                UART_SendData(CDC_UART_BASE, byte);
                count--;
            }
        } else {
            if (count > UART_TX_FIFO_SIZE) {
                count = UART_TX_FIFO_SIZE;
            }
            /* Match DAPLink: keep software data pending before enabling TX IRQ. */
            while ((count > 1U) &&
                   (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 0U)) {
                chry_ringbuffer_read_byte(&g_usbrx, &byte);
                UART_SendData(CDC_UART_BASE, byte);
                count--;
            }
            apUART_Enable_TRANSMIT_INT(CDC_UART_BASE);
        }
    }
    __DMB();
    if (state == 0U) {
        __enable_irq();
    }
}

RAM_CODE void IRQHandler_Uart1(void)
{
    uint32_t status = CDC_UART_BASE->IntRaw;

    CDC_UART_BASE->IntClear = status;

    if (((status & ((1UL << bsUART_RECEIVE_INTENAB) |
                    (1UL << bsUART_TIMEOUT_INTENAB))) != 0U) ||
        (apUART_Check_RXFIFO_EMPTY(CDC_UART_BASE) == 0U)) {
        while (apUART_Check_RXFIFO_EMPTY(CDC_UART_BASE) == 0U) {
            if (!chry_ringbuffer_write_byte(&g_uartrx, UART_ReceData(CDC_UART_BASE))) {
                cdc_rx_overflow++;
            }
        }
        __DMB();
    }

    if ((status & UART_INTBIT_ERROR) != 0U) {
        CDC_UART_BASE->StatusClear = 1U;
    }

    if ((status & (1UL << bsUART_TRANSMIT_INTENAB)) != 0U) {
        uint32_t count = chry_ringbuffer_get_used(&g_usbrx);
        uint8_t byte;

        if (count > 4U) {
            if (count > UART_TX_FIFO_SIZE) {
                count = UART_TX_FIFO_SIZE;
            }
            while ((count != 0U) &&
                   (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 0U)) {
                chry_ringbuffer_read_byte(&g_usbrx, &byte);
                UART_SendData(CDC_UART_BASE, byte);
                count--;
            }
        } else {
            apUART_Disable_TRANSMIT_INT(CDC_UART_BASE);
            while (count != 0U) {
                while (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) != 0U) {
                }
                chry_ringbuffer_read_byte(&g_usbrx, &byte);
                UART_SendData(CDC_UART_BASE, byte);
                count--;
            }
        }
        __DMB();
    }
}

void platform_uart_poll(void)
{
}

//**************************************************************************************************
// LED control
//**************************************************************************************************

void led_init(void)
{
    uint32_t mask;

    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_SysCtrl)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO1)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0));

    // Connected LED
    mask = 1 << LED_USB_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << LED_USB_PIN);
    APB_GPIO0->IOIE  = (APB_GPIO0->IOIE  & (~mask)) | (0 << LED_USB_PIN);
    APB_GPIO0->DoutClear = 1 << LED_USB_PIN;

    // SWD activity LED
    mask = 1 << LED_SWD_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << LED_SWD_PIN);
    APB_GPIO0->IOIE  = (APB_GPIO0->IOIE  & (~mask)) | (0 << LED_SWD_PIN);
    APB_GPIO0->DoutClear = 1 << LED_SWD_PIN;

    // CDC activity LED
    mask = 1 << LED_CDC_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << LED_CDC_PIN);
    APB_GPIO0->IOIE  = (APB_GPIO0->IOIE  & (~mask)) | (0 << LED_CDC_PIN);
    APB_GPIO0->DoutSet = 1 << LED_CDC_PIN;
}

void platform_dap_led_activity(void)
{
    dap_activity_led.requests++;
}

void platform_cdc_led_activity(void)
{
    cdc_activity_led.requests++;
}

void platform_led_activity_reset(void)
{
    dap_activity_led.requests = 0U;
    dap_activity_led.handled = 0U;
    dap_activity_led.value = 0U;
    cdc_activity_led.requests = 0U;
    cdc_activity_led.handled = 0U;
    cdc_activity_led.value = 0U;
    APB_GPIO0->DoutClear = (1UL << LED_SWD_PIN) | (1UL << LED_CDC_PIN);
}

static void update_activity_led(activity_led_t *led, uint32_t pin)
{
    if (led->value != 0U) {
        led->value = 0U;
        APB_GPIO0->DoutClear = 1UL << pin;
    } else if (led->handled != led->requests) {
        led->handled = led->requests;
        led->value = 1U;
        APB_GPIO0->DoutSet = 1UL << pin;
    }
}

void platform_led_poll(void)
{
    static TickType_t last_update;
    TickType_t now = xTaskGetTickCount();

    if ((TickType_t)(now - last_update) >= pdMS_TO_TICKS(30U)) {
        last_update = now;
        update_activity_led(&dap_activity_led, LED_SWD_PIN);
        update_activity_led(&cdc_activity_led, LED_CDC_PIN);
    }
}

void led_connected_on(void)  { APB_GPIO0->DoutSet   = 1 << LED_USB_PIN; }
void led_connected_off(void) { APB_GPIO0->DoutClear = 1 << LED_USB_PIN; }
void led_hid_on(void)        { APB_GPIO0->DoutSet   = 1 << LED_SWD_PIN; }
void led_hid_off(void)       { APB_GPIO0->DoutClear = 1 << LED_SWD_PIN; }
