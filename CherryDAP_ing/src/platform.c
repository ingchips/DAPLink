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

static const uint8_t *cdc_tx_data;
static volatile uint16_t cdc_tx_length;
static volatile uint16_t cdc_tx_offset;
static volatile uint16_t cdc_tx_complete;
static volatile uint32_t cdc_rx_overflow;
static volatile uint16_t cdc_control_line_state;
static volatile uint8_t cdc_control_line_dirty;

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
    cdc_tx_length = 0U;
    cdc_tx_offset = 0U;
    CDC_UART_BASE->IntClear = CDC_UART_BASE->IntRaw;
    CDC_UART_BASE->Control = 0U;
    apUART_Initialize(CDC_UART_BASE, &config, (1UL << bsUART_RECEIVE_INTENAB));
    NVIC_ClearPendingIRQ(CDC_UART_IRQ);
    NVIC_SetPriority(CDC_UART_IRQ, 3U);
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

void chry_dap_usb2uart_uart_config_callback(struct cdc_line_coding *line_coding)
{
    config_cdc_uart(line_coding);
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;

    if (dtr) {
        cdc_control_line_state |= 0x01U;
    } else {
        cdc_control_line_state &= (uint16_t)~0x01U;
    }
    cdc_control_line_dirty = 1U;
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    (void)busid;
    (void)intf;

    if (rts) {
        cdc_control_line_state |= 0x02U;
    } else {
        cdc_control_line_state &= (uint16_t)~0x02U;
    }
    cdc_control_line_dirty = 1U;
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
    cdc_control_line_dirty = 0U;
    apply_cdc_control_line_state(0U);
}

RAM_CODE void chry_dap_usb2uart_uart_send_bydma(uint8_t *data, uint16_t len)
{
    uint32_t state;

    if ((data == NULL) || (len == 0U)) {
        chry_dap_usb2uart_uart_send_complete(0U);
        return;
    }

    state = __get_PRIMASK();
    __disable_irq();
    cdc_tx_data = data;
    cdc_tx_length = len;
    cdc_tx_offset = 0U;
    while ((cdc_tx_offset < cdc_tx_length) &&
           (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 0U)) {
        UART_SendData(CDC_UART_BASE, cdc_tx_data[cdc_tx_offset++]);
    }
    apUART_Enable_TRANSMIT_INT(CDC_UART_BASE);
    if (state == 0U) {
        __enable_irq();
    }
}

RAM_CODE void IRQHandler_Uart1(void)
{
    uint32_t status = CDC_UART_BASE->IntRaw;

    CDC_UART_BASE->IntClear = status;

    if ((status & (1UL << bsUART_RECEIVE_INTENAB)) != 0U) {
        while (apUART_Check_RXFIFO_EMPTY(CDC_UART_BASE) == 0U) {
            if (!chry_ringbuffer_write_byte(&g_uartrx, UART_ReceData(CDC_UART_BASE))) {
                cdc_rx_overflow++;
            }
        }
        __DMB();
    }

    if ((status & (1UL << bsUART_TRANSMIT_INTENAB)) != 0U) {
        while ((cdc_tx_offset < cdc_tx_length) &&
               (apUART_Check_TXFIFO_FULL(CDC_UART_BASE) == 0U)) {
            UART_SendData(CDC_UART_BASE, cdc_tx_data[cdc_tx_offset++]);
        }
        if (cdc_tx_offset == cdc_tx_length) {
            apUART_Disable_TRANSMIT_INT(CDC_UART_BASE);
            cdc_tx_complete = cdc_tx_length;
            cdc_tx_length = 0U;
            __DMB();
        }
    }

    if ((status & (1UL << bsUART_ERROR_INTENAB)) != 0U) {
        CDC_UART_BASE->StatusClear = 1U;
    }
}

void platform_uart_poll(void)
{
    uint16_t completed;
    uint16_t control_line_state = 0U;
    uint8_t control_line_dirty;
    uint32_t state = __get_PRIMASK();

    __disable_irq();
    completed = cdc_tx_complete;
    cdc_tx_complete = 0U;
    control_line_dirty = cdc_control_line_dirty;
    if (control_line_dirty != 0U) {
        control_line_state = cdc_control_line_state;
        cdc_control_line_dirty = 0U;
        apply_cdc_control_line_state(control_line_state);
    }
    if (state == 0U) {
        __enable_irq();
    }

    if (completed != 0U) {
        chry_dap_usb2uart_uart_send_complete(completed);
    }
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
