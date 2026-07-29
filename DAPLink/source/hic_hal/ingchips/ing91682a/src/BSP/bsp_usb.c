#include <stdio.h>
#include <string.h>
#include "ingsoc.h"
#include "bsp_usb.h"
#include "soc.h"
#include "ing91682a.h"

#ifdef   __DAPLINK
#include "usb.h"
#else
#include <RTL.h>
#endif
#include "rl_usb.h"

#define __NO_USB_LIB_C
#include "usb_config.c"

/* Definition for overriding maximum number of endpoints supported by 
   this driver, used for reducing RAM memory consumption. */
#ifndef USBD_EP_NUM_MAX
#define USBD_EP_NUM_MAX                 (6)
#elif  (USBD_EP_NUM_MAX == 0)
#error  Maximum endpoints cannot be 0! Please increase USBD_EP_NUM_MAX!
#elif  (USBD_EP_NUM_MAX > 5)
#error  Maximum endpoints supported is 5! Please reduce USBD_EP_NUM_MAX!
#endif

BSP_USB_VAR_s UsbVar;
uint8_t DataRecvBuf[USBD_EP_NUM_MAX][EP_X_MPS_BYTES] __attribute__ ((aligned (4)));
uint8_t DataSendBuf[USBD_EP_NUM_MAX][EP_X_MPS_BYTES] __attribute__ ((aligned (4)));

#define DATA_TEMP_BUF_LEN (128)
uint8_t DataTempBuf[DATA_TEMP_BUF_LEN] __attribute__ ((aligned (4)));

uint32_t bsp_debug_flag = 0;
typedef struct
{
  #define BSP_DEBUG_BUF_LEN (0x200)
  uint32_t BufIdx;
  uint32_t Buf[BSP_DEBUG_BUF_LEN];
}BSP_DEBUG_BUF_STRUCTURE_S;
  
BSP_DEBUG_BUF_STRUCTURE_S BspDebugBuf;

typedef struct
{
  uint32_t enable:1;
  uint32_t config:1;
  uint32_t unused:30;
}BSP_EP_STATE_S;

BSP_EP_STATE_S EpState[USBD_EP_NUM_MAX];

#define DEBUG_MASK ((1<<0)|(1<<2)|(1<<4)|(1<<30))
void BSP_DEBUG_HISTORY(uint32_t data, uint32_t mask)
{
  // if(DEBUG_MASK & mask)
  // {
  //   if((BspDebugBuf.BufIdx++)>=BSP_DEBUG_BUF_LEN)
  //   {
  //     BspDebugBuf.BufIdx = 0;
  //   }
  //   BspDebugBuf.Buf[BspDebugBuf.BufIdx] = data;
  // }
}

void bsp_debug_history_init(void)
{
  BspDebugBuf.BufIdx = 0;
  memset(BspDebugBuf.Buf, 0xff, sizeof(BspDebugBuf.Buf));
  BspDebugBuf.Buf[BspDebugBuf.BufIdx] = 0x34353639;
}

static uint32_t bsp_usb_event_handler(USB_EVNET_HANDLER_T *event)
{
  uint32_t status = USB_ERROR_NONE;

  switch(event->id)
  {
    case USB_EVENT_DEVICE_RESET:
    {
        BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_EVENT_ISR|(event->id)|(event->data.ep)<<8|(event->data.type)<<16, 1<<0);
        usbd_reset_core();
        
    #ifdef __RTX
        if (USBD_RTX_DevTask) {
          isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
        }
    #else
        if (USBD_P_Reset_Event) {
          USBD_P_Reset_Event();
        }
    #endif
    }break;
    case USB_EVENT_DEVICE_SOF:
    {
      // handle sof, need enable interrupt in config.intmask
      
    
    #ifdef __RTX
        if (USBD_RTX_DevTask) {
          isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
        }
    #else
        if (USBD_P_SOF_Event) {
          USBD_P_SOF_Event();
        }
    #endif
    }break;
    case USB_EVENT_DEVICE_SUSPEND:
    {
      // handle suspend, need enable interrupt in config.intmask
      
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
      }
#else
      if (USBD_P_Suspend_Event) {
        USBD_P_Suspend_Event();
      }
#endif
    }break;
    case USB_EVENT_DEVICE_RESUME:
    {
      // handle resume, need enable interrupt in config.intmask
      
    
#ifdef __RTX
      if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_RESUME,  USBD_RTX_DevTask);
      }
#else
      if (USBD_P_Resume_Event) {
        USBD_P_Resume_Event();
      }
#endif
    }break;
    case USB_EVENT_EP0_SETUP:
    {
        BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_EVENT_ISR|(event->id)|(event->data.ep)<<8|(event->data.type)<<16, 1<<0);
        USB_SETUP_T* setup = USB_GetEp0SetupData();

    #ifdef __RTX
        if (USBD_RTX_EPTask[0]) {
          isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[0]);
        }
    #else
        if (USBD_P_EP[0]) {
          USBD_P_EP[0](USBD_EVT_SETUP);
        }
    #endif

    }break;
    
    case USB_EVENT_EP_DATA_TRANSFER:
    {
        BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_EVENT_ISR|(event->id)|(event->data.ep)<<8|(event->data.type)<<16, 1<<event->data.ep);
    #ifdef __RTX
        if (USBD_RTX_EPTask[event->data.ep]) {
          if (event->data.type == USB_CALLBACK_TYPE_RECEIVE_END) {
            isr_evt_set(USBD_EVT_IN, USBD_RTX_EPTask[event->data.ep]);
          } else {
            isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[event->data.ep]);
          }
        }
    #else
        if (USBD_P_EP[event->data.ep]) {
          if (event->data.type == USB_CALLBACK_TYPE_TRANSMIT_END) {
            USBD_P_EP[event->data.ep](USBD_EVT_IN);
          } else {
            USBD_P_EP[event->data.ep](USBD_EVT_OUT);
          }
        }
    #endif
    }break;
    default:
      break;
  }

  return status;
}

void bsp_usb_init(void)
{
  USB_INIT_CONFIG_T config;
  
  SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_USB)|(1<<SYSCTRL_ITEM_APB_PinCtrl));
  //use SYSCTRL_GetClk(SYSCTRL_ITEM_APB_USB) to confirm, USB module clock has to be 48M.
  SYSCTRL_SelectUSBClk((SYSCTRL_ClkMode)(SYSCTRL_GetPLLClk()/48000000));
  
  //platform_set_irq_callback(PLATFORM_CB_IRQ_USB, USB_IrqHandler, NULL);

  PINCTRL_SelUSB(USB_PIN_DP,USB_PIN_DM);
    PINCTRL_SetDriveStrength(USB_PIN_DP,PINCTRL_DRIVE_12mA);
    PINCTRL_SetDriveStrength(USB_PIN_DM,PINCTRL_DRIVE_12mA);
//  GIO_EnableAnalog(USB_PIN_DP);
//  GIO_EnableAnalog(USB_PIN_DM);
  
  SYSCTRL_USBPhyConfig(BSP_USB_PHY_ENABLE,BSP_USB_PHY_DP_PULL_UP);
  memset(EpState, 0x00, sizeof(EpState));
  memset(&config, 0x00, sizeof(USB_INIT_CONFIG_T));
  config.intmask = USBINTMASK_SOF;//USBINTMASK_SUSP | USBINTMASK_RESUME;
  config.handler = bsp_usb_event_handler;
  USB_InitConfig(&config);
  
  bsp_debug_history_init();
  
//  NVIC_SetPriority(IRQn_usb, 0);
  NVIC_ClearPendingIRQ(IRQn_usb);
  NVIC_EnableIRQ(IRQn_usb);
}

void bsp_usb_disable(void)
{
  USB_Close();
  SYSCTRL_SetClkGateMulti(1 << SYSCTRL_ITEM_APB_USB);

  SYSCTRL_USBPhyConfig(BSP_USB_PHY_DISABLE,0);
}

void bsp_usb_device_remote_wakeup(void)
{

}

#ifdef FEATURE_DISCONN_DETECT
void bsp_usb_device_disconn_timeout(void)
{
  bsp_usb_disable();
}
#endif

/* Driver functions */

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */

void USBD_Init (void) {

    bsp_usb_init();
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

#ifdef   __DAPLINK
void USBD_Connect (BOOL con) {
#else
void USBD_Connect (uint32_t con) {
#endif
  if (con != 0U) {                      /* Connect */
    SYSCTRL_USBPhyConfig(BSP_USB_PHY_ENABLE,BSP_USB_PHY_DP_PULL_UP);
  } else {                              /* Disconnect */
    SYSCTRL_USBPhyConfig(BSP_USB_PHY_DISABLE,0);
  }

}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {

}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume (void) {
  /* Performed by Hardware */
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp (void) {

}

/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

#ifdef   __DAPLINK
void USBD_WakeUpCfg (BOOL cfg) {
#else
void USBD_WakeUpCfg (uint32_t cfg) {
#endif
  /* Not needed */
}

/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

void USBD_SetAddress (uint32_t adr, uint32_t setup) {
  /* Performed by Hardware */
  #ifdef BSP_DEBUG_V0
  printf("addr 0x%x\n",adr);
  #endif
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */
#ifdef   __DAPLINK
void USBD_Configure (BOOL cfg) {
#else
void USBD_Configure (uint32_t cfg) {
#endif
  /* Not needed */
}

/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
  uint8_t  ep_num, ep_type;
  uint16_t ep_max_len;
  USB_EP_DESCRIPTOR_REAL_T l_ep;

  //ep_dir     = (pEPD->bEndpointAddress &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num     =  pEPD->bEndpointAddress & ~USB_ENDPOINT_DIRECTION_MASK;
  ep_type    =  pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
  ep_max_len =  pEPD->wMaxPacketSize;

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return;
  }
  if (ep_type == USB_ENDPOINT_TYPE_ISOCHRONOUS) {
    /* Isochronous Endpoint is not supported */
    return;
  }

  if (ep_max_len > 64U) {
    ep_max_len = 64U;                   /* Maximum packet size is 64 */
  }

  BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_SET_CONFIG|(pEPD->bEndpointAddress)|(pEPD->bmAttributes << 8), 1<<0);
  
  l_ep.ep = pEPD->bEndpointAddress;
  l_ep.type = 5;
  l_ep.mps = ep_max_len;
  l_ep.attributes = ep_type;
  USB_ConfigureEp(&l_ep);
  
  if(USB_ENDPOINT_OUT(pEPD->bEndpointAddress))
  {
    USB_RecvData(pEPD->bEndpointAddress, DataRecvBuf[ep_num], sizeof(DataRecvBuf[ep_num]), 1<<USB_TRANSFERT_FLAG_FLEXIBLE_RECV_LEN);
  }
  
  EpState[ep_num].config = 1;
}

/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP (uint32_t dir) {

}

/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (uint32_t EPNum) {
  uint8_t  ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;

  EpState[ep_num].enable = 1;
}

/*
 *  Disable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (uint32_t EPNum) {
  uint8_t  ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return;
  }

  USB_DisableEp(EPNum);
  EpState[ep_num].enable = 0;
  EpState[ep_num].config = 0;
}

/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (uint32_t EPNum) {
  uint8_t ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return;
  }


}

/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP (uint32_t EPNum) {
  uint8_t ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return;
  }

  BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_STALL|(EPNum)|(1<<8), 1<<ep_num);
  
  #ifdef BSP_DEBUG_V0
  printf("stall %x\n",EPNum);
  #endif
  
  if(0 == ep_num)
  {
    USB_SetEp0Stall(EPNum);
  }
  else
  {
    USB_SetStallEp(EPNum, U_TRUE);
  }
}

/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP (uint32_t EPNum) {
  uint8_t ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return;
  }
  
  #ifdef BSP_DEBUG_V0
  printf("clear stall %x\n",EPNum);
  #endif
  
  BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_STALL|(EPNum)|(0<<8), 1<<ep_num);
  /* Clear stall */
  USB_SetStallEp(EPNum, U_FALSE);
}

/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf (uint32_t EPNum) {
  /* Not needed */
}

/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

uint32_t USBD_ReadEP (uint32_t EPNum, uint8_t *pData, U32 size) {
  uint8_t ep_dir, ep_num;
  uint32_t cnt;
  ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;
  USB_SETUP_T* setup = USB_GetEp0SetupData();

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return 0;
  }
  
  if(ep_dir != 0){
    return 0;
  }
  
  BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_READ_EP|(EPNum)|(size<<8), 1<<ep_num);
  
  if(0 == ep_num)
  {
    cnt = size;//sizeof(USB_SETUP_T);
    memcpy(pData, setup, cnt);
    return cnt;
  }
  else
  {
    //if((uint32_t)pData & 0x3) BSP_DEBUG_HISTORY(BSP_DEBUG_ADDR_4BYTE|((uint32_t)pData & 0xffffff));
    
    cnt = USB_GetTransferedSize(EPNum);
    if(size < cnt)
    {
      cnt = size;
    }
    memcpy(pData, DataRecvBuf[ep_num], cnt);
    
    USB_RecvData(EPNum, DataRecvBuf[ep_num], sizeof(DataRecvBuf[ep_num]), 1<<USB_TRANSFERT_FLAG_FLEXIBLE_RECV_LEN);
    return cnt;
  }
}

/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

uint32_t USBD_WriteEP (uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
  uint8_t ep_dir, ep_num;
  uint32_t status = USB_ERROR_NONE, cnt_1=cnt;
  
  ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;
  USB_SETUP_T* setup = USB_GetEp0SetupData();

  if (ep_num >= USBD_EP_NUM_MAX) {      /* If ep_num is not available */
    return 0;
  }
  
//  if(cnt > EP_X_MPS_BYTES){
//    return 0;
//  }
  
  if(ep_dir != 1){
    return 0;
  }
  
  if(0 == ep_num)
  {
    cnt_1 = cnt > (setup->wLength) ? setup->wLength : cnt;
  }
  
  BSP_DEBUG_HISTORY(BSP_DEBUG_BSP_WRITE_EP|EPNum|(cnt_1<<8), 1<<ep_num);
  
  if((uint32_t)pData & 0x3)
  {
    //BSP_DEBUG_HISTORY(BSP_DEBUG_ADDR_4BYTE|((uint32_t)pData & 0xffffff), 1<<ep_num);
    
    if(cnt_1 > DATA_TEMP_BUF_LEN)
    {
      cnt_1 = DATA_TEMP_BUF_LEN;
    }
    memcpy(DataTempBuf, pData, cnt_1);
    status |= USB_SendData(EPNum, (void*)DataTempBuf, cnt_1, 0);
  }
  else
  {
    status |= USB_SendData(EPNum, (void*)pData, cnt_1, 0);
  }
  
  return cnt;
}

/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

uint32_t USBD_GetFrame (void) {
  return 0;
}

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

uint32_t USBD_GetError (void) {
  return 0U;
}

/*
 *  USB Device Interrupt Service Routine
 */

void IRQHandler_usb(void)
{
    NVIC_DisableIRQ(IRQn_usb);
    USBD_SignalHandler();
}

uint32_t USBD_GetEpConfigState(uint32_t EPNum)
{
  uint8_t ep_num;

  //ep_dir = (EPNum &  USB_ENDPOINT_DIRECTION_MASK) >> 7;
  ep_num =  EPNum & ~USB_ENDPOINT_DIRECTION_MASK;
  return EpState[ep_num].config;
}
