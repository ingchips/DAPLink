#include <stdint.h>
#include "usb_config.h"
#include "usb_dwc2_reg.h"
#include "ingsoc.h"
#include "soc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_def.h"
#include "usb_dc.h"

#define USB_PIN_DP GIO_GPIO_16
#define USB_PIN_DM GIO_GPIO_17

#if (CONFIG_USE_USB_DEVICE == 0) && (CONFIG_USE_USB_HOST == 0)
#error "Define CONFIG_USE_USB_DEVICE or CONFIG_USE_USB_HOST"
#endif

typedef enum {
    BSP_USB_PHY_DISABLE,
    BSP_USB_PHY_ENABLE
} BSP_USB_PHY_ENABLE_e;

typedef enum {
    BSP_USB_PHY_DP_PULL_UP = 1,
    BSP_USB_PHY_DM_PULL_UP,
    BSP_USB_PHY_DP_DM_PULL_DOWN
} BSP_USB_PHY_PULL_e;

extern uint32_t SystemCoreClock;

void IRQHandler_usb(void)
{
    #if CONFIG_USE_USB_DEVICE
    USBD_IRQHandler(0);
    #elif CONFIG_USE_USB_HOST
    USBH_IRQHandler(0);
    #endif
}

void usb_dc_low_level_init(uint8_t busid)
{
    (void)busid;

    SystemCoreClock = SYSCTRL_GetPLLClk();

    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_USB)
                            | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                            | (1 << SYSCTRL_ITEM_APB_GPIO0));

    SYSCTRL_SelectUSBClk((SYSCTRL_ClkMode)(SYSCTRL_GetPLLClk() / 48000000));

    NVIC_SetPriority(IRQn_usb, 2U);
    NVIC_ClearPendingIRQ(IRQn_usb);
    NVIC_EnableIRQ(IRQn_usb);

    PINCTRL_SelUSB(USB_PIN_DP, USB_PIN_DM);
    SYSCTRL_USBPhyConfig(BSP_USB_PHY_ENABLE, BSP_USB_PHY_DP_PULL_UP);
}

void usb_dc_low_level_deinit(uint8_t busid)
{
    (void)busid;
    USB_Close();
    SYSCTRL_SetClkGateMulti(1 << SYSCTRL_ITEM_APB_USB);
    SYSCTRL_USBPhyConfig(BSP_USB_PHY_DISABLE, 0);
}

void usbd_dwc2_delay_ms(uint8_t ms)
{
    vTaskDelay(ms);
}

uint32_t usbd_get_dwc2_gccfg_conf(uint32_t reg_base)
{
    return ((1 << 16) | (1 << 21));
}
