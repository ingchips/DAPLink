/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usb_util.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "vfs_manager.h"


#define MSC_IN_EP  0x81
#define MSC_OUT_EP 0x02

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

#define MSC_MAX_MPS 64

const uint8_t msc_ram_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'I', 0x00,                  /* wcChar0 */
    'N', 0x00,                  /* wcChar1 */
    'G', 0x00,                  /* wcChar2 */
    'C', 0x00,                  /* wcChar3 */
    'H', 0x00,                  /* wcChar4 */
    'I', 0x00,                  /* wcChar5 */
    'P', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x24,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'I', 0x00,                  /* wcChar0 */
    'N', 0x00,                  /* wcChar1 */
    'G', 0x00,                  /* wcChar2 */
    'C', 0x00,                  /* wcChar3 */
    'H', 0x00,                  /* wcChar4 */
    'I', 0x00,                  /* wcChar5 */
    'P', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'M', 0x00,                  /* wcChar9 */
    'S', 0x00,                  /* wcChar10 */
    'C', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    'B', 0x00,                  /* wcChar13 */
    'O', 0x00,                  /* wcChar14 */
    'O', 0x00,                  /* wcChar15 */
    'T', 0x00,                  /* wcChar16 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0xe,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'V', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '.', 0x00,                  /* wcChar2 */
    '1', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '0', 0x00,                  /* wcChar5 */
    0x00
};

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

#define BLOCK_SIZE  512
#define BLOCK_COUNT 10

typedef struct
{
    uint8_t BlockSpace[BLOCK_SIZE];
} BLOCK_TYPE;

BLOCK_TYPE mass_block[BLOCK_COUNT];

__WEAK void msc_write_handler(uint32_t sector,uint8_t *buffer, uint32_t num_of_sectors)
{

}

void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
//    *block_num = 1000; //Pretend having so many buffer,not has actually.
//    *block_size = BLOCK_SIZE;
    *block_size = BLOCK_SIZE;
    *block_num = vfs_get_total_size() / *block_size; //Pretend having so many buffer,not has actually.
}
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    vfs_read(sector, buffer, length/BLOCK_SIZE);
    return 0;
}

int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
    // if (sector < BLOCK_COUNT)
    //     memcpy(mass_block[sector].BlockSpace, buffer, length);
    // vfs_write(sector, buffer, length/BLOCK_SIZE);
    
    msc_write_handler(sector, buffer, length/BLOCK_SIZE);
    return 0;
}

static struct usbd_interface intf0;

void msc_ram_init(uint8_t busid, uintptr_t reg_base)
{

    vfs_mngr_init(true);

    vfs_user_build_filesystem();

    usbd_desc_register(busid, msc_ram_descriptor);

    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));

    usbd_initialize(busid, reg_base, usbd_event_handler);
}
