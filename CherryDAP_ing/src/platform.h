#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include "stdint.h"

void platform_uart_init(void);
void platform_uart_poll(void);
void led_init(void);
void platform_led_poll(void);
void platform_dap_led_activity(void);
void platform_cdc_led_activity(void);
void platform_led_activity_reset(void);
void platform_cdc_control_reset(void);
void led_connected_on(void);
void led_connected_off(void);
void led_hid_on(void);
void led_hid_off(void);

#endif
