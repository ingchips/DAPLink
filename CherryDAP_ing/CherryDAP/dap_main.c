#include "dap_main.h"
#include "platform.h"
#include "eflash.h"

#define CMSIS_DAP_INTERFACE_SIZE (9 + 7 + 7)
#define CMSIS_DAP_HID_LEN        (9 + 9 + 7 + 7)

#define HIDRAW_INTERVAL 1

#define HID_CUSTOM_REPORT_DESC_SIZE 53
#define CMSIS_DAP_HID_REPORT_DESC_SIZE 27

#define USBD_WINUSB_VENDOR_CODE 0x20
#define USBD_WEBUSB_VENDOR_CODE 0x21

#define USBD_WEBUSB_ENABLE 0
#define USBD_BULK_ENABLE   1
#define USBD_WINUSB_ENABLE 1

/* WinUSB Microsoft OS 2.0 descriptor sizes */
#define WINUSB_DESCRIPTOR_SET_HEADER_SIZE  10
#define WINUSB_FUNCTION_SUBSET_HEADER_SIZE 8
#define WINUSB_FEATURE_COMPATIBLE_ID_SIZE  20

#define FUNCTION_SUBSET_LEN                160
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132

#define USBD_WINUSB_DESC_SET_LEN (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + USBD_WEBUSB_ENABLE * FUNCTION_SUBSET_LEN + USBD_BULK_ENABLE * FUNCTION_SUBSET_LEN)

#define USBD_NUM_DEV_CAPABILITIES (USBD_WEBUSB_ENABLE + USBD_WINUSB_ENABLE)

#define USBD_WEBUSB_DESC_LEN 24
#define USBD_WINUSB_DESC_LEN 28

#define USBD_BOS_WTOTALLENGTH (0x05 +                                      \
                               USBD_WEBUSB_DESC_LEN * USBD_WEBUSB_ENABLE + \
                               USBD_WINUSB_DESC_LEN * USBD_WINUSB_ENABLE)

#define USB_CONFIG_SIZE (9 + CMSIS_DAP_INTERFACE_SIZE + CDC_ACM_DESCRIPTOR_LEN + \
                         CONFIG_CHERRYDAP_USE_HID * CMSIS_DAP_HID_LEN +          \
                         CONFIG_CHERRYDAP_USE_MSC * MSC_DESCRIPTOR_LEN + USBD_WEBUSB_ENABLE * 9)

#define INTF_NUM (1 + 2 + CONFIG_CHERRYDAP_USE_HID + CONFIG_CHERRYDAP_USE_MSC + USBD_WEBUSB_ENABLE)

#define MSC_INTF_NUM (3 + CONFIG_CHERRYDAP_USE_HID)

#define WEBUSB_INTF_NUM (3 + CONFIG_CHERRYDAP_USE_HID + CONFIG_CHERRYDAP_USE_MSC)

enum usb_string_index {
    USB_STRING_LANGID = 0,
    USB_STRING_MANUFACTURER,
    USB_STRING_PRODUCT,
    USB_STRING_SERIAL_NUMBER,
    USB_STRING_WEBUSB,
    USB_STRING_CMSIS_DAP_V2,
    USB_STRING_CMSIS_DAP_V1,
};

#define WEBUSB_URL_STRINGS                                 \
    'c', 'h', 'e', 'r', 'r', 'y', 'd', 'a', 'p', '.', 'c', 'h', 'e', 'r', 'r', 'y', '-', 'e', 'm', 'b', 'e', 'd', 'd', 'e', 'd', '.', 'o', 'r', 'g',

USB_MEM_ALIGNX const uint8_t USBD_WinUSBDescriptorSetDescriptor[] = {
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE), /* wDescriptorType */
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */  /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),          /* wDescriptorSetTotalLength */
#if (USBD_WEBUSB_ENABLE)
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), // wLength
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), // wDescriptorType
    WEBUSB_INTF_NUM,                           // bFirstInterface USBD_WINUSB_IF_NUM
    0,                                         // bReserved
    WBVAL(FUNCTION_SUBSET_LEN),                // wSubsetLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  // wLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  // wDescriptorType
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        // CompatibleId
    0, 0, 0, 0, 0, 0, 0, 0,                    // SubCompatibleId
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), // wLength
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   // wDescriptorType
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), // wPropertyDataType
    WBVAL(42),                                 // wPropertyNameLength
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), // wPropertyDataLength
    '{', 0,
    '9', 0, '2', 0, 'C', 0, 'E', 0, '6', 0, '4', 0, '6', 0, '2', 0, '-', 0,
    '9', 0, 'C', 0, '7', 0, '7', 0, '-', 0,
    '4', 0, '6', 0, 'F', 0, 'E', 0, '-', 0,
    '9', 0, '3', 0, '3', 0, 'B', 0, '-',
    0, '3', 0, '1', 0, 'C', 0, 'B', 0, '9', 0, 'C', 0, '5', 0, 'A', 0, 'A', 0, '3', 0, 'B', 0, '9', 0,
    '}', 0, 0, 0, 0, 0,
#endif
#if USBD_BULK_ENABLE
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), /* wDescriptorType */
    0,                                         /* bFirstInterface USBD_BULK_IF_NUM*/
    0,                                         /* bReserved */
    WBVAL(FUNCTION_SUBSET_LEN),                /* wSubsetLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  /* wLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  /* wDescriptorType */
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        /* CompatibleId*/
    0, 0, 0, 0, 0, 0, 0, 0,                    /* SubCompatibleId*/
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), /* wLength */
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   /* wDescriptorType */
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), /* wPropertyDataType */
    WBVAL(42),                                 /* wPropertyNameLength */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), /* wPropertyDataLength */
    '{', 0,
    'C', 0, 'D', 0, 'B', 0, '3', 0, 'B', 0, '5', 0, 'A', 0, 'D', 0, '-', 0,
    '2', 0, '9', 0, '3', 0, 'B', 0, '-', 0,
    '4', 0, '6', 0, '6', 0, '3', 0, '-', 0,
    'A', 0, 'A', 0, '3', 0, '6', 0, '-',
    0, '1', 0, 'A', 0, 'A', 0, 'E', 0, '4', 0, '6', 0, '4', 0, '6', 0, '3', 0, '7', 0, '7', 0, '6', 0,
    '}', 0, 0, 0, 0, 0
#endif
};

USB_MEM_ALIGNX const uint8_t USBD_BinaryObjectStoreDescriptor[] = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    USBD_NUM_DEV_CAPABILITIES,    /* bNumDeviceCaps */
#if (USBD_WEBUSB_ENABLE)
    USBD_WEBUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0x38, 0xB6, 0x08, 0x34,         /* PlatformCapabilityUUID */
    0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,
    WBVAL(0x0100), /* 1.00 */ /* bcdVersion */
    USBD_WEBUSB_VENDOR_CODE,  /* bVendorCode */
    1,                        /* iLandingPage */
#endif
#if (USBD_WINUSB_ENABLE)
    USBD_WINUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,         /* PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D,
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06, /* >= Win 8.1 */ /* dwWindowsVersion*/
    WBVAL(USBD_WINUSB_DESC_SET_LEN),         /* wDescriptorSetTotalLength */
    USBD_WINUSB_VENDOR_CODE,                 /* bVendorCode */
    0,                                       /* bAltEnumCode */
#endif
};

#define URL_DESCRIPTOR_LENGTH    (3 + 29)

const uint8_t USBD_WebUSBURLDescriptor[URL_DESCRIPTOR_LENGTH] = {
    URL_DESCRIPTOR_LENGTH,
    WEBUSB_URL_TYPE,
    WEBUSB_URL_SCHEME_HTTPS,
    WEBUSB_URL_STRINGS
};

// clang-format off
#define HID_DESC()                                                                                                                       \
    /************** Descriptor of Custom interface *****************/                                                                    \
    0x09,                                               /* bLength: Interface Descriptor size */                                         \
    USB_DESCRIPTOR_TYPE_INTERFACE,                  /* bDescriptorType: Interface descriptor type */                                 \
    0X03,                                           /* bInterfaceNumber: Number of Interface */                                      \
    0x00,                                           /* bAlternateSetting: Alternate setting */                                       \
    0x02,                                           /* bNumEndpoints */                                                              \
    0x03,                                           /* bInterfaceClass: HID */                                                       \
    0x00,                                           /* bInterfaceSubClass: no boot */                                               \
    0x00,                                           /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */                           \
    USB_STRING_CMSIS_DAP_V1, /* iInterface: Index of string descriptor */ /******************** Descriptor of Custom HID ********************/ \
    0x09,                                           /* bLength: HID Descriptor size */                                               \
    HID_DESCRIPTOR_TYPE_HID,                        /* bDescriptorType: HID */                                                       \
    0x11,                                           /* bcdHID: HID Class Spec release number */                                      \
    0x01,                                                                                                                            \
    0x00,                                              /* bCountryCode: Hardware target country */                                   \
    0x01,                                              /* bNumDescriptors: Number of HID class descriptors to follow */              \
    0x22,                                              /* bDescriptorType */                                                         \
    CMSIS_DAP_HID_REPORT_DESC_SIZE,                    /* wItemLength: Total length of Report descriptor */                          \
    0x00,                                              /******************** Descriptor of Custom in endpoint ********************/  \
    0x07,                                              /* bLength: Endpoint Descriptor size */                                       \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                      /* bDescriptorType: */                                                        \
    HID_IN_EP,                                         /* bEndpointAddress: Endpoint Address (IN) */                                 \
    0x03,                                              /* bmAttributes: Interrupt endpoint */                                        \
    WBVAL(HID_PACKET_SIZE),                            /* wMaxPacketSize: 4 Byte max */                                              \
    HIDRAW_INTERVAL, /* bInterval: Polling Interval */ /******************** Descriptor of Custom out endpoint ********************/ \
    0x07,                                              /* bLength: Endpoint Descriptor size */                                       \
    USB_DESCRIPTOR_TYPE_ENDPOINT,                      /* bDescriptorType: */                                                        \
    HID_OUT_EP,                                        /* bEndpointAddress: Endpoint Address (IN) */                                 \
    0x03,                                              /* bmAttributes: Interrupt endpoint */                                        \
    WBVAL(HID_PACKET_SIZE),                            /* wMaxPacketSize: 4 Byte max */                                              \
    HIDRAW_INTERVAL                                    /* bInterval: Polling Interval */
// clang-format on

USB_MEM_ALIGNX static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

USB_MEM_ALIGNX static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* Interface 0 */
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, USB_STRING_CMSIS_DAP_V2),
    /* Endpoint OUT 2 */
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    /* Endpoint IN 1 */
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, DAP_PACKET_SIZE, 0x00),
#if CONFIG_CHERRYDAP_USE_HID
    HID_DESC(),
#endif
#if CONFIG_CHERRYDAP_USE_MSC
    MSC_DESCRIPTOR_INIT(MSC_INTF_NUM, MSC_OUT_EP, MSC_IN_EP, DAP_PACKET_SIZE, 0x00),
#endif
#if USBD_WEBUSB_ENABLE
    USB_INTERFACE_DESCRIPTOR_INIT(WEBUSB_INTF_NUM, 0x00, 0x00, 0xff, 0x00, 0x00, USB_STRING_WEBUSB),
#endif
};

USB_MEM_ALIGNX static const uint8_t other_speed_config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* Interface 0 */
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, USB_STRING_CMSIS_DAP_V2),
    /* Endpoint OUT 2 */
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_OUT_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    /* Endpoint IN 1 */
    USB_ENDPOINT_DESCRIPTOR_INIT(DAP_IN_EP, USB_ENDPOINT_TYPE_BULK, DAP_PACKET_SIZE, 0x00),
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, DAP_PACKET_SIZE, 0x00),
#if CONFIG_CHERRYDAP_USE_HID
    HID_DESC(),
#endif
#if CONFIG_CHERRYDAP_USE_MSC
    MSC_DESCRIPTOR_INIT(0x04, MSC_OUT_EP, MSC_IN_EP, DAP_PACKET_SIZE, 0x00),
#endif
#if USBD_WEBUSB_ENABLE
    USB_INTERFACE_DESCRIPTOR_INIT(WEBUSB_INTF_NUM, 0x00, 0x00, 0xff, 0x00, 0x00, USB_STRING_WEBUSB),
#endif
};

/*!< custom hid report descriptor */
const uint8_t hid_custom_report_desc[HID_CUSTOM_REPORT_DESC_SIZE] = {
        /* USER CODE BEGIN 0 */
        0x06, 0x00, 0xff, /* USAGE_PAGE (Vendor Defined Page 1) */
        0x09, 0x01, /* USAGE (Vendor Usage 1) */
        0xa1, 0x01, /* COLLECTION (Application) */
        0x85, 0x02, /*   REPORT ID (0x02) */
        0x09, 0x02, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0x81, 0x02, /*   INPUT (Data,Var,Abs) */
        /* <___________________________________________________> */
        0x85, 0x01, /*   REPORT ID (0x01) */
        0x09, 0x03, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*   LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0x91, 0x02, /*   OUTPUT (Data,Var,Abs) */

        /* <___________________________________________________> */
        0x85, 0x03, /*   REPORT ID (0x03) */
        0x09, 0x04, /*   USAGE (Vendor Usage 1) */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0) */
        0x25, 0xff, /*   LOGICAL_MAXIMUM (255) */
        0x75, 0x08, /*   REPORT_SIZE (8) */
        0x96, 0xff, 0x03, /*   REPORT_COUNT (1023) */
        0xb1, 0x02, /*   FEATURE (Data,Var,Abs) */
        /* USER CODE END 0 */
        0xC0 /*     END_COLLECTION	             */
};

USB_MEM_ALIGNX const uint8_t cmsis_dap_hid_report_desc[CMSIS_DAP_HID_REPORT_DESC_SIZE] = {
    0x06, 0x00, 0xff,
    0x09, 0x01,
    0xa1, 0x01,
    0x15, 0x00,
    0x26, 0xff, 0x00,
    0x75, 0x08,
    0x95, DAP_PACKET_SIZE,
    0x09, 0x01,
    0x81, 0x02,
    0x95, DAP_PACKET_SIZE,
    0x09, 0x01,
    0x91, 0x02,
    0xc0
};

char serial_number_dynamic[9] = "00000000"; // Dynamic serial number

static void serial_number_init(void)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t uid[4] = { 0U, 0U, 0U, 0U };
    uint32_t uuid;
    uint32_t i;

    flash_read_uid(uid);
    uuid = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];

    for (i = 0U; i < 8U; i++) {
        serial_number_dynamic[i] = hex[(uuid >> (28U - (i * 4U))) & 0x0FU];
    }
    serial_number_dynamic[8] = '\0';
}

char *string_descriptors[] = {
    (char[]){ 0x09, 0x04 },             /* Langid */
    "INGCHIPS",                        /* Manufacturer */
    "CMSIS-DAP",              /* Product */
    "00000000000000000123456789ABCDEF", /* Serial Number */
    "INGCHIPS WebUSB",
    "CMSIS-DAP v2",
    "CMSIS-DAP v1",
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_1, 0x00, 0x00, 0x00, 0x01),
};

__WEAK const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

__WEAK const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor;
}

__WEAK const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

__WEAK const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return other_speed_config_descriptor;
}

__WEAK const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index == USB_STRING_SERIAL_NUMBER) {
        return serial_number_dynamic;
    }

    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

typedef struct {
    volatile uint16_t request_index_in;
    volatile uint16_t request_index_out;
    volatile uint16_t request_count_in;
    volatile uint16_t request_count_out;
    volatile uint8_t request_idle;
    volatile uint16_t response_index_in;
    volatile uint16_t response_index_out;
    volatile uint16_t response_count_in;
    volatile uint16_t response_count_out;
    volatile uint8_t response_in_flight;
    volatile uint8_t transfer_abort;
    uint8_t ep_out;
    uint8_t ep_in;
    uint16_t packet_size;
    uint16_t receive_length;
    uint8_t aggregate_packets;
    uint8_t fixed_response_size;
    USB_MEM_ALIGNX uint8_t request[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
    uint16_t request_size[DAP_PACKET_COUNT];
    USB_MEM_ALIGNX uint8_t response[DAP_PACKET_COUNT][DAP_PACKET_SIZE];
    uint16_t response_size[DAP_PACKET_COUNT];
} dap_transport_t;

static USB_NOCACHE_RAM_SECTION dap_transport_t dap_bulk = {
    .request_idle = 1U,
    .ep_out = DAP_OUT_EP,
    .ep_in = DAP_IN_EP,
    .packet_size = DAP_PACKET_SIZE,
    .aggregate_packets = 1U,
    .fixed_response_size = 0U
};

static USB_NOCACHE_RAM_SECTION dap_transport_t dap_hid = {
    .request_idle = 1U,
    .ep_out = HID_OUT_EP,
    .ep_in = HID_IN_EP,
    .packet_size = HID_PACKET_SIZE,
    .aggregate_packets = 0U,
    .fixed_response_size = 1U
};

static volatile uint8_t usb_configured;
static volatile uint32_t usb_generation;
static uint8_t dap_execute_request[DAP_PACKET_SIZE];

#define CDC_DEFAULT_BAUDRATE 115200U
#define CDC_DEFAULT_STOPBITS 0U
#define CDC_DEFAULT_PARITY   0U
#define CDC_DEFAULT_DATABITS 8U

volatile struct cdc_line_coding g_cdc_lincoding = {
    CDC_DEFAULT_BAUDRATE,
    CDC_DEFAULT_STOPBITS,
    CDC_DEFAULT_PARITY,
    CDC_DEFAULT_DATABITS
};
volatile uint8_t config_uart = 0;
volatile uint8_t config_uart_transfer = 0;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t uartrx_ringbuffer[CONFIG_UARTRX_RINGBUF_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usbrx_ringbuffer[CONFIG_USBRX_RINGBUF_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usb_tmpbuffer[DAP_PACKET_SIZE];
/* DWC2 requires the address of every transfer buffer to be 4-byte aligned. */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_usb_tx_buffer[DAP_PACKET_SIZE];
USB_NOCACHE_RAM_SECTION chry_ringbuffer_t g_uartrx;
USB_NOCACHE_RAM_SECTION chry_ringbuffer_t g_usbrx;

static volatile uint8_t usbrx_idle_flag = 0;
static volatile uint8_t usbtx_idle_flag = 0;
static volatile uint8_t uarttx_idle_flag = 0;

static void cdc_start_usb_tx(void)
{
    uint32_t size = chry_ringbuffer_get_used(&g_uartrx);

    if (size > DAP_PACKET_SIZE) {
        size = DAP_PACKET_SIZE;
    }
    if (size != 0U) {
        chry_ringbuffer_peek(&g_uartrx, cdc_usb_tx_buffer, size);
        usbd_ep_start_write(0, CDC_IN_EP, cdc_usb_tx_buffer, size);
    } else {
        usbtx_idle_flag = 1U;
    }
}

static void dap_transport_reset(dap_transport_t *transport)
{
    transport->request_index_in = 0U;
    transport->request_index_out = 0U;
    transport->request_count_in = 0U;
    transport->request_count_out = 0U;
    transport->request_idle = 1U;
    transport->response_index_in = 0U;
    transport->response_index_out = 0U;
    transport->response_count_in = 0U;
    transport->response_count_out = 0U;
    transport->response_in_flight = 0U;
    transport->transfer_abort = 0U;
    transport->receive_length = 0U;
    memset(transport->request_size, 0, sizeof(transport->request_size));
}

static uint32_t dap_transport_lock(void)
{
    uint32_t state = __get_PRIMASK();

    __disable_irq();
    return state;
}

static void dap_transport_unlock(uint32_t state)
{
    __DMB();
    if (state == 0U) {
        __enable_irq();
    }
}

static uint16_t dap_transport_outstanding(const dap_transport_t *transport)
{
    return (uint16_t)(transport->request_count_in - transport->response_count_out);
}

static uint16_t dap_transport_response_pending(const dap_transport_t *transport)
{
    return (uint16_t)(transport->response_count_in - transport->response_count_out);
}

static void dap_transport_start_read(dap_transport_t *transport)
{
    uint32_t state;
    int ret;
    uint16_t remaining = (uint16_t)(DAP_PACKET_SIZE - transport->receive_length);
    uint16_t read_length = transport->packet_size;

    state = dap_transport_lock();
    if ((usb_configured == 0U) || (transport->request_idle == 0U) ||
        (dap_transport_outstanding(transport) >= DAP_PACKET_COUNT)) {
        dap_transport_unlock(state);
        return;
    }

    transport->request_idle = 0U;
    if (transport->receive_length == 0U) {
        /* Clear the unused tail so a short command cannot inherit bytes from
         * the previous request. Bulk may fill this buffer over several USB
         * packets. */
        memset(transport->request[transport->request_index_in], 0, DAP_PACKET_SIZE);
    }
    if (read_length > remaining) {
        read_length = remaining;
    }
    ret = usbd_ep_start_read(0, transport->ep_out,
                             transport->request[transport->request_index_in] + transport->receive_length,
                             read_length);
    if (ret < 0) {
        transport->request_idle = 1U;
    }
    dap_transport_unlock(state);
}

static void dap_transport_out_complete(dap_transport_t *transport, uint32_t nbytes)
{
    uint16_t received;
    uint8_t complete;

    transport->request_idle = 1U;
    if (usb_configured == 0U) {
        transport->receive_length = 0U;
        return;
    }

    if (nbytes == 0U) {
        dap_transport_start_read(transport);
        return;
    }

    received = (nbytes > (uint32_t)(DAP_PACKET_SIZE - transport->receive_length)) ?
               (uint16_t)(DAP_PACKET_SIZE - transport->receive_length) : (uint16_t)nbytes;
    transport->receive_length = (uint16_t)(transport->receive_length + received);
    complete = (transport->aggregate_packets == 0U) ||
               (received < transport->packet_size) ||
               (transport->receive_length == DAP_PACKET_SIZE);

    if (complete == 0U) {
        dap_transport_start_read(transport);
        return;
    }

    platform_dap_led_activity();
    transport->request_size[transport->request_index_in] = transport->receive_length;
    transport->receive_length = 0U;

    if (transport->request[transport->request_index_in][0] == ID_DAP_TransferAbort) {
        transport->transfer_abort = 1U;
        /* The DAP core may be busy in a SWD transfer while the USB ISR runs. */
        DAP_TransferAbort = 1U;
    } else {
        transport->request_index_in++;
        if (transport->request_index_in == DAP_PACKET_COUNT) {
            transport->request_index_in = 0U;
        }
        transport->request_count_in++;
    }

    if (dap_transport_outstanding(transport) < DAP_PACKET_COUNT) {
        dap_transport_start_read(transport);
    }
}

static void dap_transport_start_write(dap_transport_t *transport)
{
    uint32_t state;
    int ret;

    state = dap_transport_lock();
    if ((usb_configured == 0U) || (transport->response_in_flight != 0U) ||
        (dap_transport_response_pending(transport) == 0U)) {
        dap_transport_unlock(state);
        return;
    }

    transport->response_in_flight = 1U;
    ret = usbd_ep_start_write(0, transport->ep_in,
                              transport->response[transport->response_index_out],
                              transport->response_size[transport->response_index_out]);
    if (ret < 0) {
        transport->response_in_flight = 0U;
    }
    dap_transport_unlock(state);
}

static void dap_transport_in_complete(dap_transport_t *transport)
{
    uint32_t state;

    state = dap_transport_lock();
    if (transport->response_in_flight != 0U) {
        transport->response_in_flight = 0U;
        transport->response_index_out++;
        if (transport->response_index_out == DAP_PACKET_COUNT) {
            transport->response_index_out = 0U;
        }
        transport->response_count_out++;
    }
    dap_transport_unlock(state);

    dap_transport_start_write(transport);
    dap_transport_start_read(transport);
}

static uint32_t dap_transport_process_one(dap_transport_t *transport)
{
    uint32_t generation;
    uint32_t state;
    uint16_t request_index;
    uint16_t pending;
    uint32_t response_index;
    uint32_t response_size;

    if ((usb_configured == 0U) ||
        (transport->request_count_in == transport->request_count_out) ||
        (dap_transport_response_pending(transport) >= DAP_PACKET_COUNT)) {
        return 0U;
    }

    generation = usb_generation;

    /* Match DAPLink: convert all queued commands that are already in this
     * transport queue before executing the oldest request. */
    request_index = transport->request_index_out;
    pending = (uint16_t)(transport->request_count_in - transport->request_count_out);
    while (pending != 0U &&
           transport->request[request_index][0] == ID_DAP_QueueCommands) {
        transport->request[request_index][0] = ID_DAP_ExecuteCommands;
        request_index++;
        if (request_index == DAP_PACKET_COUNT) {
            request_index = 0U;
        }
        pending--;
    }

    if (transport->transfer_abort) {
        DAP_TransferAbort = 1U;
        transport->transfer_abort = 0U;
    }

    memcpy(dap_execute_request,
           transport->request[transport->request_index_out],
           DAP_PACKET_SIZE);
    if ((usb_configured == 0U) || (generation != usb_generation)) {
        return 1U;
    }

    response_index = transport->response_index_in;
    memset(transport->response[response_index], 0, DAP_PACKET_SIZE);
    response_size = (uint16_t)DAP_ExecuteCommand(
        dap_execute_request,
        transport->response[response_index]);
    transport->response_size[response_index] = transport->fixed_response_size ?
                                               transport->packet_size : (uint16_t)response_size;

    state = dap_transport_lock();
    if ((usb_configured == 0U) || (generation != usb_generation)) {
        dap_transport_unlock(state);
        return 1U;
    }

    transport->request_index_out++;
    if (transport->request_index_out == DAP_PACKET_COUNT) {
        transport->request_index_out = 0U;
    }
    transport->request_count_out++;

    transport->response_index_in++;
    if (transport->response_index_in == DAP_PACKET_COUNT) {
        transport->response_index_in = 0U;
    }
    transport->response_count_in++;
    dap_transport_unlock(state);

    dap_transport_start_write(transport);
    dap_transport_start_read(transport);
    return 1U;
}

void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    switch (event) {
        case USBD_EVENT_RESET:
            usb_configured = 0U;
            __DMB();
            usb_generation++;
            DAP_TransferAbort = 1U;
            led_connected_off();
            platform_led_activity_reset();
            dap_transport_reset(&dap_bulk);
            dap_transport_reset(&dap_hid);
            usbrx_idle_flag = 0;
            usbtx_idle_flag = 0;
            uarttx_idle_flag = 0;
            config_uart_transfer = 0;
            g_cdc_lincoding.dwDTERate = CDC_DEFAULT_BAUDRATE;
            g_cdc_lincoding.bCharFormat = CDC_DEFAULT_STOPBITS;
            g_cdc_lincoding.bParityType = CDC_DEFAULT_PARITY;
            g_cdc_lincoding.bDataBits = CDC_DEFAULT_DATABITS;
            platform_cdc_control_reset();
            break;
        case USBD_EVENT_CONNECTED:
            led_connected_on();
            break;
        case USBD_EVENT_DISCONNECTED:
            usb_configured = 0U;
            __DMB();
            usb_generation++;
            DAP_TransferAbort = 1U;
            dap_transport_reset(&dap_bulk);
            dap_transport_reset(&dap_hid);
            led_connected_off();
            platform_led_activity_reset();
            platform_cdc_control_reset();
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            usb_configured = 0U;
            __DMB();
            usb_generation++;
            DAP_TransferAbort = 1U;
            dap_transport_reset(&dap_bulk);
            dap_transport_reset(&dap_hid);
            led_connected_on();
            /* Use the default 115200/8N1 UART configuration until the host
             * sends a different CDC line coding. */
            config_uart_transfer = 1;
            usbtx_idle_flag = 1;
            uarttx_idle_flag = 1;
            __DMB();
            usb_configured = 1U;
            dap_transport_start_read(&dap_bulk);
            dap_transport_start_read(&dap_hid);
            usbd_ep_start_read(0, CDC_OUT_EP, usb_tmpbuffer, DAP_PACKET_SIZE);

            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void dap_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    dap_transport_out_complete(&dap_bulk, nbytes);
}

void dap_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    dap_transport_in_complete(&dap_bulk);
}

void dap_hid_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    dap_transport_out_complete(&dap_hid, nbytes);
}

void dap_hid_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    dap_transport_in_complete(&dap_hid);
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    if (nbytes != 0U) {
        platform_cdc_led_activity();
    }
    chry_ringbuffer_write(&g_usbrx, usb_tmpbuffer, nbytes);
    if (chry_ringbuffer_get_free(&g_usbrx) >= DAP_PACKET_SIZE) {
        usbd_ep_start_read(0, CDC_OUT_EP, usb_tmpbuffer, DAP_PACKET_SIZE);
    } else {
        usbrx_idle_flag = 1;
    }
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;

    if (nbytes != 0U) {
        platform_cdc_led_activity();
    }

    chry_ringbuffer_drop(&g_uartrx, nbytes);
    if ((nbytes % DAP_PACKET_SIZE) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(0, CDC_IN_EP, NULL, 0);
    } else {
        cdc_start_usb_tx();
    }
}

struct usbd_endpoint dap_out_ep = {
    .ep_addr = DAP_OUT_EP,
    .ep_cb = dap_out_callback
};

struct usbd_endpoint dap_in_ep = {
    .ep_addr = DAP_IN_EP,
    .ep_cb = dap_in_callback
};

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

#if CONFIG_CHERRYDAP_USE_HID
struct usbd_endpoint hid_custom_in_ep = {
        .ep_addr = HID_IN_EP,
        .ep_cb = dap_hid_in_callback,
};

struct usbd_endpoint hid_custom_out_ep = {
        .ep_addr = HID_OUT_EP,
        .ep_cb = dap_hid_out_callback,
};
#endif

struct usbd_interface dap_intf;
struct usbd_interface intf1;
struct usbd_interface intf2;
#if CONFIG_CHERRYDAP_USE_HID
struct usbd_interface hid_intf;
#endif

#if CONFIG_CHERRYDAP_USE_MSC
struct usbd_interface intf3;
#endif

struct usb_msosv2_descriptor msosv2_desc = {
    .vendor_code = USBD_WINUSB_VENDOR_CODE,
    .compat_id = USBD_WinUSBDescriptorSetDescriptor,
    .compat_id_len = USBD_WINUSB_DESC_SET_LEN,
};

struct usb_bos_descriptor bos_desc = {
    .string = USBD_BinaryObjectStoreDescriptor,
    .string_len = USBD_BOS_WTOTALLENGTH
};

struct usb_webusb_descriptor webusb_url_desc = {
    .vendor_code = USBD_WEBUSB_VENDOR_CODE,
    .string = USBD_WebUSBURLDescriptor,
    .string_len = URL_DESCRIPTOR_LENGTH
};

const struct usb_descriptor cmsisdap_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .bos_descriptor = &bos_desc,
    .msosv2_descriptor = &msosv2_desc,
    .webusb_url_descriptor = &webusb_url_desc
};

void chry_dap_init(uint8_t busid, uint32_t reg_base)
{
    chry_ringbuffer_init(&g_uartrx, uartrx_ringbuffer, CONFIG_UARTRX_RINGBUF_SIZE);
    chry_ringbuffer_init(&g_usbrx, usbrx_ringbuffer, CONFIG_USBRX_RINGBUF_SIZE);

    serial_number_init();
    DAP_Setup();

    usbd_desc_register(0, &cmsisdap_descriptor);

    /*!< winusb */
    usbd_add_interface(0, &dap_intf);
    usbd_add_endpoint(0, &dap_out_ep);
    usbd_add_endpoint(0, &dap_in_ep);

    /*!< cdc acm */
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf1));
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf2));
    usbd_add_endpoint(0, &cdc_out_ep);
    usbd_add_endpoint(0, &cdc_in_ep);

#if CONFIG_CHERRYDAP_USE_HID
    /*!< hid */
    usbd_add_interface(0, usbd_hid_init_intf(0, &hid_intf, cmsis_dap_hid_report_desc, CMSIS_DAP_HID_REPORT_DESC_SIZE));
    usbd_add_endpoint(0, &hid_custom_in_ep);
    usbd_add_endpoint(0, &hid_custom_out_ep);
#endif

#if CONFIG_CHERRYDAP_USE_MSC
    usbd_add_interface(0, usbd_msc_init_intf(0, &intf3, MSC_OUT_EP, MSC_IN_EP));
#endif
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

void chry_dap_handle(void)
{
    uint32_t progress;

    do {
        progress = dap_transport_process_one(&dap_bulk);
        progress |= dap_transport_process_one(&dap_hid);
    } while (progress != 0U);

    dap_transport_start_write(&dap_bulk);
    dap_transport_start_write(&dap_hid);
    dap_transport_start_read(&dap_bulk);
    dap_transport_start_read(&dap_hid);
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    if (memcmp(line_coding, (uint8_t *)&g_cdc_lincoding, sizeof(struct cdc_line_coding)) != 0) {
        memcpy((uint8_t *)&g_cdc_lincoding, line_coding, sizeof(struct cdc_line_coding));
        config_uart = 1;
        config_uart_transfer = 0;
    }
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    memcpy(line_coding, (uint8_t *)&g_cdc_lincoding, sizeof(struct cdc_line_coding));
}

void chry_dap_usb2uart_handle(void)
{
    uint32_t size;
    uint8_t *buffer;

    if (config_uart) {
        /* disable irq here */
        config_uart = 0;
        /* config uart here */
        chry_dap_usb2uart_uart_config_callback((struct cdc_line_coding *)&g_cdc_lincoding);
        usbtx_idle_flag = 1;
        uarttx_idle_flag = 1;
        config_uart_transfer = 1;
        //chry_ringbuffer_reset_read(&g_uartrx);
        /* enable irq here */
    }

    if (config_uart_transfer == 0) {
        return;
    }

    /* why we use chry_ringbuffer_linear_read_setup?
     * becase we use dma and we do not want to use temp buffer to memcpy from ringbuffer
     *
    */

    /* uartrx to usb tx */
    if (usbtx_idle_flag) {
        if (chry_ringbuffer_get_used(&g_uartrx)) {
            usbtx_idle_flag = 0;
            /* start first transfer */
            cdc_start_usb_tx();
        }
    }

    /* usbrx to uart tx */
    if (uarttx_idle_flag) {
        if (chry_ringbuffer_get_used(&g_usbrx)) {
            uarttx_idle_flag = 0;
            /* start first transfer */
            buffer = chry_ringbuffer_linear_read_setup(&g_usbrx, &size);
            chry_dap_usb2uart_uart_send_bydma(buffer, size);
        }
    }

    /* check whether usb rx ringbuffer have space to store */
    if (usbrx_idle_flag) {
        if (chry_ringbuffer_get_free(&g_usbrx) >= DAP_PACKET_SIZE) {
            usbrx_idle_flag = 0;
            usbd_ep_start_read(0, CDC_OUT_EP, usb_tmpbuffer, DAP_PACKET_SIZE);
        }
    }
}

/* implment by user */
__WEAK void chry_dap_usb2uart_uart_config_callback(struct cdc_line_coding *line_coding)
{
}

/* called by user */
void chry_dap_usb2uart_uart_send_complete(uint32_t size)
{
    uint8_t *buffer;

    chry_ringbuffer_linear_read_done(&g_usbrx, size);

    if (chry_ringbuffer_get_used(&g_usbrx)) {
        buffer = chry_ringbuffer_linear_read_setup(&g_usbrx, &size);
        chry_dap_usb2uart_uart_send_bydma(buffer, size);
    } else {
        uarttx_idle_flag = 1;
    }
}

/* implment by user */
__WEAK void chry_dap_usb2uart_uart_send_bydma(uint8_t *data, uint16_t len)
{
}

#if CONFIG_CHERRYDAP_USE_MSC
#define BLOCK_SIZE  512
#define BLOCK_COUNT 10

typedef struct
{
    uint8_t BlockSpace[BLOCK_SIZE];
} BLOCK_TYPE;

BLOCK_TYPE mass_block[BLOCK_COUNT];

void usbd_msc_get_cap(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    *block_num = 1000; //Pretend having so many buffer,not has actually.
    *block_size = BLOCK_SIZE;
}
int usbd_msc_sector_read(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (sector < 10)
        memcpy(buffer, mass_block[sector].BlockSpace, length);
    return 0;
}

int usbd_msc_sector_write(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (sector < 10)
        memcpy(mass_block[sector].BlockSpace, buffer, length);
    return 0;
}
#endif
