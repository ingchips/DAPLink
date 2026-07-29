#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

/* ================ USB common Configuration ================ */

#define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)

#define usb_malloc(size) malloc(size)
#define usb_free(ptr)    free(ptr)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

/* Device mode for CMSIS-DAP */
#define CONFIG_USE_USB_HOST     0
#define CONFIG_USE_USB_DEVICE   1
#define CONFIG_USBDEV_ADVANCE_DESC

/* DWC2 FIFO configuration for Full-Speed USB device */
#define CONFIG_USB_DWC2_RXALL_FIFO_SIZE (128)
#define CONFIG_USB_DWC2_TX0_FIFO_SIZE (64 / 4)
#define CONFIG_USB_DWC2_TX1_FIFO_SIZE (128 / 4)   /* DAP bulk IN EP */
#define CONFIG_USB_DWC2_TX2_FIFO_SIZE (128 / 4)   /* CDC data IN EP */
#define CONFIG_USB_DWC2_TX3_FIFO_SIZE (32 / 4)   /* CDC notification IN EP */
#define CONFIG_USB_DWC2_TX4_FIFO_SIZE (64 / 4)  /* CMSIS-DAP HID IN EP */
#define CONFIG_USB_DWC2_TX5_FIFO_SIZE (32 / 4)   
#define CONFIG_USB_DWC2_TX6_FIFO_SIZE (64 / 4)   
#define CONFIG_USB_DWC2_TX7_FIFO_SIZE (0 / 4)
#define CONFIG_USB_DWC2_TX8_FIFO_SIZE (0 / 4)

#define CONFIG_USB_PRINTF_COLOR_ENABLE

#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION //__attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 256
#endif

#ifndef CONFIG_USBDEV_MSC_MAX_LUN
#define CONFIG_USBDEV_MSC_MAX_LUN 1
#endif

#ifndef CONFIG_USBDEV_MSC_MAX_BUFSIZE
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 512
#endif

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 5
#endif

#endif
