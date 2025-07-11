/**
 * @file    DAP_config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2021, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#include "IO_Config.h"
//**************************************************************************************************
/**
\defgroup DAP_Config_Debug_gr CMSIS-DAP Debug Unit Information
\ingroup DAP_ConfigIO_gr
@{
Provides definitions about the hardware and configuration of the Debug Unit.

This information includes:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit Identification strings (Vendor, Product, Serial Number).
 - Debug Unit communication packet size.
 - Debug Access Port supported modes and settings (JTAG/SWD and SWO).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

// Board configuration options

/// Processor Clock of the Cortex-M MCU used in the Debug Unit.
/// This value is used to calculate the SWD/JTAG clock speed.
#define CPU_CLOCK               (SystemCoreClock/10)        ///< Specifies the CPU Clock in Hz

/// Number of processor cycles for I/O Port write operations.
/// This value is used to calculate the SWD/JTAG clock speed that is generated with I/O
/// Port write operations in the Debug Unit by a Cortex-M MCU. Most Cortex-M processors
/// require 2 processor cycles for a I/O Port Write operation.  If the Debug Unit uses
/// a Cortex-M0+ processor with high-speed peripheral I/O only 1 processor cycle might be
/// required.
#define IO_PORT_WRITE_CYCLES    1U              ///< I/O Cycles: 2=default, 1=Cortex-M0+ fast I/0

/// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#ifndef DAP_SWD
#define DAP_SWD                 1               ///< SWD Mode:  1 = available, 0 = not available
#endif

/// Indicate that JTAG communication mode is available at the Debug Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#ifndef DAP_JTAG
#define DAP_JTAG                0               ///< JTAG Mode: 1 = available, 0 = not available.
#endif

/// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
/// This setting impacts the RAM requirements of the Debug Unit. Valid range is 1 .. 255.
#define DAP_JTAG_DEV_CNT        8               ///< Maximum number of JTAG devices on scan chain

/// Default communication mode on the Debug Access Port.
/// Used for the command \ref DAP_Connect when Port Default mode is selected.
#if (DAP_SWD == 1)
#define DAP_DEFAULT_PORT        1               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.
#elif (DAP_JTAG == 1)
#define DAP_DEFAULT_PORT        2               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.
#else
#error Must enable DAP_SWD and/or DAP_JTAG
#endif

/// Default communication speed on the Debug Access Port for SWD and JTAG mode.
/// Used to initialize the default SWD/JTAG clock frequency.
/// The command \ref DAP_SWJ_Clock can be used to overwrite this default setting.
#define DAP_DEFAULT_SWJ_CLOCK   10000000         ///< Default SWD/JTAG clock frequency in Hz.

/// Maximum Package Size for Command and Response data.
/// This configuration settings is used to optimize the communication performance with the
/// debugger and depends on the USB peripheral. Typical vales are 64 for Full-speed USB HID or WinUSB,
/// 1024 for High-speed USB HID and 512 for High-speed USB WinUSB.
#define DAP_PACKET_SIZE         64              ///< Specifies Packet Size in bytes.

/// Maximum Package Buffers for Command and Response data.
/// This configuration settings is used to optimize the communication performance with the
/// debugger and depends on the USB peripheral. For devices with limited RAM or USB buffer the
/// setting can be reduced (valid range is 1 .. 255). Change setting to 4 for High-Speed USB.
#define DAP_PACKET_COUNT        19              ///< Buffers: 64 = Full-Speed, 4 = High-Speed.

/// Indicate that UART Serial Wire Output (SWO) trace is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define SWO_UART                0               ///< SWO UART:  1 = available, 0 = not available

/// USART Driver instance number for the UART SWO.
#define SWO_UART_DRIVER         0               ///< USART Driver instance number (Driver_USART#).

/// Maximum SWO UART Baudrate
#define SWO_UART_MAX_BAUDRATE   10000000U       ///< SWO UART Maximum Baudrate in Hz

/// Indicate that Manchester Serial Wire Output (SWO) trace is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define SWO_MANCHESTER          0               ///< SWO Manchester:  1 = available, 0 = not available.

/// SWO Trace Buffer Size.
#define SWO_BUFFER_SIZE         512U           ///< SWO Trace Buffer Size in bytes (must be 2^n).

/// SWO Streaming Trace.
#define SWO_STREAM              0               ///< SWO Streaming Trace: 1 = available, 0 = not available.

/// Clock frequency of the Test Domain Timer. Timer value is returned with \ref TIMESTAMP_GET.
#define TIMESTAMP_CLOCK         0U      ///< Timestamp clock in Hz (0 = timestamps not supported).
// DAPLink: disabled because we use DWT for timestamps and M0 doesn't have a DWT.

/// Indicate that UART Communication Port is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define DAP_UART                0               ///< DAP UART:  1 = available, 0 = not available.

/// USART Driver instance number for the UART Communication Port.
#define DAP_UART_DRIVER         1               ///< USART Driver instance number (Driver_USART#).

/// UART Receive Buffer Size.
#define DAP_UART_RX_BUFFER_SIZE 4096U           ///< Uart Receive Buffer Size in bytes (must be 2^n).

/// UART Transmit Buffer Size.
#define DAP_UART_TX_BUFFER_SIZE 4096U           ///< Uart Transmit Buffer Size in bytes (must be 2^n).

/// Indicate that UART Communication via USB COM Port is available.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define DAP_UART_USB_COM_PORT   1               ///< USB COM Port:  1 = available, 0 = not available.

/// Debug Unit is connected to fixed Target Device.
/// The Debug Unit may be part of an evaluation board and always connected to a fixed
/// known device. In this case a Device Vendor, Device Name, Board Vendor and Board Name strings
/// are stored and may be used by the debugger or IDE to configure device parameters.
#define TARGET_FIXED            0               ///< Target: 1 = known, 0 = unknown;

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor


DAP Hardware I/O Pin Access Functions
-------------------------------------
The various I/O Pins are accessed by functions that implement the Read, Write, Set, or Clear to
these I/O Pins.

For the SWDIO I/O Pin there are additional functions that are called in SWD I/O mode only.
This functions are provided to achieve faster I/O that is possible with some advanced GPIO
peripherals that can independently write/read a single I/O pin without affecting any other pins
of the same I/O port. The following SWDIO I/O Pin functions are provided:
 - \ref PIN_SWDIO_OUT_ENABLE to enable the output mode from the DAP hardware.
 - \ref PIN_SWDIO_OUT_DISABLE to enable the input mode to the DAP hardware.
 - \ref PIN_SWDIO_IN to read from the SWDIO I/O pin with utmost possible speed.
 - \ref PIN_SWDIO_OUT to write to the SWDIO I/O pin with utmost possible speed.
*/


// Configure DAP I/O pins ------------------------------

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
Configures the DAP Hardware I/O pins for JTAG mode:
 - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
 - TDO to input mode.
*/
__STATIC_INLINE void PORT_JTAG_SETUP(void)
{
#if (DAP_JTAG != 0)

#endif
}
/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
 - SWCLK, SWDIO, nRESET to output mode and set to default high level.
 - TDI, TMS, nTRST to HighZ mode (pins are unused in SWD mode).
*/
__STATIC_INLINE void PORT_SWD_SETUP(void)
{
//    SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ITEM_APB_SysCtrl)
//                                    | (1 << SYSCTRL_ITEM_APB_PinCtrl)
//                                    | (1 << SYSCTRL_ITEM_APB_GPIO1)
//                                    | (1 << SYSCTRL_ITEM_APB_GPIO0));
                                
    // Set SWCLK HIGH
//    PINCTRL_SetPadMux(SWCLK_PIN, IO_SOURCE_GPIO);
    // GIO_SetDirection(SWCLK_PIN, GIO_DIR_OUTPUT);
    // GIO_WriteValue(SWCLK_PIN, 1);
    
    // Set SWDIO HIGH
//    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GPIO);
    // GIO_SetDirection(SWDIO_PIN, GIO_DIR_BOTH);
    // PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);
    // GIO_WriteValue(SWDIO_PIN, 1);

    #if (!SPI_ENABLE)
    uint32_t mask = 1 << SWCLK_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << SWCLK_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (0 << SWCLK_PIN);
    APB_GPIO0->DoutSet |= 1 << SWCLK_PIN;

    mask = 1 << SWDIO_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << SWDIO_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << SWDIO_PIN);
    APB_GPIO0->DoutSet |= 1 << SWDIO_PIN;

    int bit = 1ul << (SWDIO_PIN & 0x1f);
    APB_PINCTRL->PS_CTRL[0] |= bit;
    APB_PINCTRL->PE_CTRL[0] |= bit;
    #endif

}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
__STATIC_INLINE void PORT_OFF(void)
{
    uint32_t mask = 1 << SWCLK_PIN;
    #if (!SPI_ENABLE)
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (0 << SWCLK_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << SWCLK_PIN);

    mask = 1 << SWDIO_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (0 << SWDIO_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << SWDIO_PIN);
    #endif

    #ifdef nRESET_PIN
    mask = 1 << nRESET_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (0 << nRESET_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << nRESET_PIN);
    #endif


//    PINCTRL_SetPadMux(SWCLK_PIN, IO_SOURCE_GENERAL);
    // GIO_SetDirection(SWCLK_PIN, GIO_DIR_INPUT);

//    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GENERAL);
    // GIO_SetDirection(SWDIO_PIN, GIO_DIR_INPUT);

    // #ifdef nRESET_PIN
//    PINCTRL_SetPadMux(nRESET_PIN, IO_SOURCE_GENERAL);
    // GIO_SetDirection(nRESET_PIN, GIO_DIR_INPUT);
    // #endif
}


// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_SWCLK_TCK_IN(void)
{
    // return GIO_ReadValue(SWCLK_PIN);
    
    #if (!SPI_ENABLE)
    return (APB_GPIO0->DataIn >> SWCLK_PIN) & 1;
    #else
    return (0);   // Not available
    #endif
}

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
__STATIC_FORCEINLINE void     PIN_SWCLK_TCK_SET(void)
{
    // GIO_WriteValue(SWCLK_PIN, 1);
    
    #if (!SPI_ENABLE)
    APB_GPIO0->DoutSet |= 1 << SWCLK_PIN;
    #endif
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
__STATIC_FORCEINLINE void     PIN_SWCLK_TCK_CLR(void)
{
    // GIO_WriteValue(SWCLK_PIN, 0);
    
    #if (!SPI_ENABLE)
    APB_GPIO0->DoutClear |= 1 << SWCLK_PIN;
    #endif
}


// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Get Input.
\return Current status of the SWDIO/TMS DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_TMS_IN(void)
{
    // return GIO_ReadValue(SWDIO_PIN);
    
    #if (!SPI_ENABLE)
    return (APB_GPIO0->DataIn >> SWDIO_PIN) & 1;
    #else
    return (0);   // Not available
    #endif
}

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_TMS_SET(void)
{
    // GIO_WriteValue(SWDIO_PIN, 1);
    
    #if (!SPI_ENABLE)
    APB_GPIO0->DoutSet |= 1 << SWDIO_PIN;
    #endif
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_TMS_CLR(void)
{
    // GIO_WriteValue(SWDIO_PIN, 0);
    
    #if (!SPI_ENABLE)
    APB_GPIO0->DoutClear |= 1 << SWDIO_PIN;
    #endif
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_SWDIO_IN(void)
{
    // return GIO_ReadValue(SWDIO_PIN);
    
    #if (!SPI_ENABLE)
    return (APB_GPIO0->DataIn >> SWDIO_PIN) & 1;
    #else
    return (0);   // Not available
    #endif
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE void PIN_SWDIO_OUT(uint32_t bit)
{
    #if (!SPI_ENABLE)
    if (bit & 0x1) {
        APB_GPIO0->DoutSet |= 1 << SWDIO_PIN;
        
        
        // GIO_WriteValue(SWDIO_PIN, 1);
    } else {
        APB_GPIO0->DoutClear |= 1 << SWDIO_PIN;
        
        
        // GIO_WriteValue(SWDIO_PIN, 0);
    }
    #endif
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT_ENABLE(void)
{
//    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GENERAL);
    // GIO_SetDirection(SWDIO_PIN, GIO_DIR_BOTH);
    // GIO_WriteValue(SWDIO_PIN, 1);

    #if (!SPI_ENABLE)
    uint32_t mask = 1 << SWDIO_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << SWDIO_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << SWDIO_PIN);
    APB_GPIO0->DoutSet |= 1 << SWDIO_PIN;
    #endif
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
__STATIC_FORCEINLINE void     PIN_SWDIO_OUT_DISABLE(void)
{
//    PINCTRL_SetPadMux(SWDIO_PIN, IO_SOURCE_GENERAL);
    // GIO_SetDirection(SWDIO_PIN, GIO_DIR_INPUT);
    // PINCTRL_Pull(SWDIO_PIN, PINCTRL_PULL_UP);
    
    
    #if (!SPI_ENABLE)
    uint32_t mask = 1 << SWDIO_PIN;
    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (0 << SWDIO_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (1 << SWDIO_PIN);

    int bit = 1ul << (SWDIO_PIN & 0x1f);
    APB_PINCTRL->PS_CTRL[0] |= bit;
    APB_PINCTRL->PE_CTRL[0] |= bit;
    #endif
}


// TDI Pin I/O ---------------------------------------------

/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_TDI_IN(void)
{
    return (0);   // Not available
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE void     PIN_TDI_OUT(uint32_t bit)
{
    ;             // Not available
}


// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_TDO_IN(void)
{
    return (0);   // Not available
}


// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_nTRST_IN(void)
{
    return (0);   // Not available
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
__STATIC_FORCEINLINE void     PIN_nTRST_OUT(uint32_t bit)
{
    ;             // Not available
}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
__STATIC_FORCEINLINE uint32_t PIN_nRESET_IN(void)
{
    return (0);   // Not available
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
#include "swd_host.h"
#include "IO_Config.h"
__STATIC_FORCEINLINE void     PIN_nRESET_OUT(uint32_t bit)
{
    swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
    #ifdef nRESET_PIN
    // FET drive logic
    if (bit) {
        // GIO_WriteValue(nRESET_PIN, 1);
        
        
        APB_GPIO0->DoutSet |= 1 << nRESET_PIN;
    } else {
        // GIO_WriteValue(nRESET_PIN, 0);
        
        
        APB_GPIO0->DoutClear |= 1 << nRESET_PIN;
    }
    #else
    swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
    #endif
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_LEDs_gr CMSIS-DAP Hardware Status LEDs
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware may provide LEDs that indicate the status of the CMSIS-DAP Debug Unit.

It is recommended to provide the following LEDs for status indication:
 - Connect LED: is active when the DAP hardware is connected to a debugger.
 - Running LED: is active when the debugger has put the target device into running state.
*/

/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
__STATIC_INLINE void LED_CONNECTED_OUT(uint32_t bit)
{
    if (bit) {
        // GIO_WriteValue(LED_CONNECTED_PIN, 1);
        APB_GPIO0->DoutSet |= 1 << LED_CONNECTED_PIN;
    } else {
        // GIO_WriteValue(LED_CONNECTED_PIN, 0);
        APB_GPIO0->DoutClear |= 1 << LED_CONNECTED_PIN;
    }
}

/** Debug Unit: Set status Target Running LED.
\param bit status of the Target Running LED.
           - 1: Target Running LED ON: program execution in target started.
           - 0: Target Running LED OFF: program execution in target stopped.
*/
__STATIC_INLINE void LED_RUNNING_OUT(uint32_t bit)
{
    #if 0
    if (bit) {
        // GIO_WriteValue(LED_RUNNING_PIN, 1);
        APB_GPIO0->DoutSet |= 1 << LED_RUNNING_PIN;
    } else {
        // GIO_WriteValue(LED_RUNNING_PIN, 0);
        APB_GPIO0->DoutClear |= 1 << LED_RUNNING_PIN;
    }
    #endif
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_Timestamp_gr CMSIS-DAP Timestamp
\ingroup DAP_ConfigIO_gr
@{
Access function for Test Domain Timer.

The value of the Test Domain Timer in the Debug Unit is returned by the function \ref TIMESTAMP_GET. By
default, the DWT timer is used.  The frequency of this timer is configured with \ref TIMESTAMP_CLOCK.

*/

/** Get timestamp of Test Domain Timer.
\return Current timestamp value.
*/
__STATIC_INLINE uint32_t TIMESTAMP_GET (void) {
  return (DWT->CYCCNT);
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
__STATIC_INLINE void DAP_SETUP(void)
{
    uint32_t mask = 1 << LED_CONNECTED_PIN;
    SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ITEM_APB_SysCtrl)
                                    | (1 << SYSCTRL_ITEM_APB_PinCtrl)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO1)
                                    | (1 << SYSCTRL_ITEM_APB_GPIO0));
                                

//    PINCTRL_SetPadMux(LED_CONNECTED_PIN, IO_SOURCE_GENERAL);

    APB_GPIO0->ChDir = (APB_GPIO0->ChDir & (~mask)) | (1 << LED_CONNECTED_PIN);
    APB_GPIO0->IOIE = (APB_GPIO0->IOIE & (~mask)) | (0 << LED_CONNECTED_PIN);
    APB_GPIO0->DoutClear |= 1 << LED_CONNECTED_PIN;
    // GIO_SetDirection(LED_CONNECTED_PIN, GIO_DIR_OUTPUT);
    // GIO_WriteValue(LED_CONNECTED_PIN, 0);
    
    #if LED_RUNNING_PIN
//    PINCTRL_SetPadMux(LED_RUNNING_PIN, IO_SOURCE_GENERAL);
    GIO_SetDirection(LED_RUNNING_PIN, GIO_DIR_OUTPUT);
    GIO_WriteValue(LED_RUNNING_PIN, 0);
    #endif
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/

//daplink dont have reset comond we add it write reset table
//and swd write word include form swd_host.h
__STATIC_INLINE uint32_t RESET_TARGET(void)
{
    swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
    return (1);              // change to '1' when a device reset sequence is implemented
    //we add swd reset commod so return 1 other return 0.
}

///@}


#endif /* __DAP_CONFIG_H__ */
