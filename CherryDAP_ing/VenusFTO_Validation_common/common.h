
#ifndef COMMON__H
#define COMMON__H

#include "soc.h"

#define RF_FREQ                 (24  * MHz)
#define PLL_FREQ                (384 * MHz)
#define HCLK_FREQ               (24  * MHz)
#define PCLK_FREQ               (24  * MHz)
#define UART_FREQ               (24  * MHz)
#define TIMER_FREQ              PCLK_FREQ

////////////////////////////////////////
//  ROM Code
////////////////////////////////////////
typedef void (*rom_FlashBlock32Erase)(uint32_t addr);
typedef void (*rom_FlashChipErase)(void);
typedef void (*rom_FlashDisableContinuousMode)(void);
typedef void (*rom_FlashEnableContinuousMode)(void);
typedef void (*rom_FlashEnableQuadMode)(void);
typedef uint16_t (*rom_FlashGetStatusReg)(void);
typedef void (*rom_FlashPageErase)(uint32_t addr);
typedef void (*rom_FlashPageProgram)(uint32_t addr, const uint8_t *data, uint32_t len);
typedef void (*rom_FlashQuadPageProgram)(uint32_t addr, uint8_t data[256], uint32_t len);
typedef uint32_t (*rom_FlashRDID)(void);
typedef uint32_t (*rom_FlashREMS)(void);
typedef void (*rom_FlashRUID)(uint32_t *RUID);
typedef void (*rom_FlashSectorErase)(uint32_t addr);
typedef void (*rom_FlashSetQE)(void);
typedef void (*rom_FlashSetStatusReg)(uint16_t data);
typedef void (*rom_FlashWaitBusyDown)(void);
typedef void (*rom_FlashWaitWIPDown)(void);
typedef void (*rom_FlashWriteDisable)(void);
typedef void (*rom_FlashWriteEnable)(void);

#define ROM_FlashBlock32Erase           ((rom_FlashBlock32Erase)          (0x00000779))
#define ROM_FlashChipErase              ((rom_FlashChipErase)             (0x000007a5))
#define ROM_FlashDisableContinuousMode  ((rom_FlashDisableContinuousMode) (0x000007c9))
#define ROM_FlashEnableContinuousMode   ((rom_FlashEnableContinuousMode)  (0x0000080d))
#define ROM_FlashEnableQuadMode         ((rom_FlashEnableQuadMode)        (0x00000831))
#define ROM_FlashGetStatusReg           ((rom_FlashGetStatusReg)          (0x0000084d))
#define ROM_FlashPageErase              ((rom_FlashPageErase)             (0x000008c1))
#define ROM_FlashPageProgram            ((rom_FlashPageProgram)           (0x000008ed))
#define ROM_FlashQuadPageProgram        ((rom_FlashQuadPageProgram)       (0x0000092d))
#define ROM_FlashRDID                   ((rom_FlashRDID)                  (0x0000096d))
#define ROM_FlashREMS                   ((rom_FlashREMS)                  (0x000009d5))
#define ROM_FlashRUID                   ((rom_FlashRUID)                  (0x00000a41))
#define ROM_FlashSectorErase            ((rom_FlashSectorErase)           (0x00000ac1))
#define ROM_FlashSetQE                  ((rom_FlashSetQE)                 (0x00000aed))
#define ROM_FlashSetStatusReg           ((rom_FlashSetStatusReg)          (0x00000b01))
#define ROM_FlashWaitBusyDown           ((rom_FlashWaitBusyDown)          (0x00000b6d))
#define ROM_FlashWaitWIPDown            ((rom_FlashWaitWIPDown)           (0x00000b91))
#define ROM_FlashWriteDisable           ((rom_FlashWriteDisable)          (0x00000be1))
#define ROM_FlashWriteEnable            ((rom_FlashWriteEnable)           (0x00000c11))


////////////////////////////////////////
//  Flash
////////////////////////////////////////
void FlashWaitBusyDown(void);
void FlashWaitWIPDown(void);
void FlashWriteEnable(void);
void FlashWriteDisable(void);
void FlashSetQE(void);
void FlashEnableContinuousMode(void);
void FlashDisableContinuousMode(void);
void FlashPageProgram(uint32_t addr, uint8_t data[256], uint32_t len);
void FlashQuadPageProgram(uint32_t addr, uint8_t data[256], uint32_t len);
void FlashPageErase(uint32_t addr);
void FlashSectorErase(uint32_t addr);
void FlashBlock32Erase(uint32_t addr);
void FlashBlock64Erase(uint32_t addr);
void FlashChipErase(void);
void FlashSetStatusReg(uint16_t data);
uint16_t FlashGetStatusReg(void);

////////////////////////////////////////
//  IOMUX
////////////////////////////////////////
void ClearIOMUXFuncSel(void);
void ClearIOMUXInputSel(void);
void SetGPIOFuncSel(uint8_t GPIOIdx, E_IOMUXFuncSel FuncSel);
uint8_t GetIOMUXInputSel(uint16_t Map[3], uint8_t PINIdx);
void IOMUXPullUp(uint8_t PINIdx);
void IOMUXPullDown(uint8_t PINIdx);
void SetAnaPIN(uint8_t PINIdx);
void SetUart0PIN(uint8_t TxPIN, uint8_t RxPIN);
void SetUart1PIN(uint8_t TxPIN, uint8_t RxPIN);
void SetUart0PINFlowCtrl(uint8_t TxPIN, uint8_t RxPIN, uint8_t RTSPIN, uint8_t CTSPIN);
void SetUart1PINFlowCtrl(uint8_t TxPIN, uint8_t RxPIN, uint8_t RTSPIN, uint8_t CTSPIN);
void SetI2C0PIN(uint8_t SclPIN, uint8_t SdaPIN);
void SetI2C1PIN(uint8_t SclPIN, uint8_t SdaPIN);
void SetKeyScannerPINRow(uint32_t RowIdx, uint32_t PINIdx);
void SetKeyScannerPINCol(uint32_t ColIdx, uint32_t PINIdx);
void SetQSPIPIN(uint8_t CSPIN, uint8_t CLKPIN, uint8_t MOSIPIN, uint8_t MISOPIN, uint8_t WPPIN, uint8_t HOLDPIN);
void SetAPBSPIPIN(uint8_t CSPIN, uint8_t CLKPIN, uint8_t MOSIPIN, uint8_t MISOPIN, uint8_t WPPIN, uint8_t HOLDPIN);
void SetI2SPIN(uint8_t BClkPIN, uint8_t LrClkPIN, uint8_t DinPIN, uint8_t DoutPIN);
void SetSWPIN(uint8_t SWCLKPIN, uint8_t SWDIOPIN);
void SetIRPIN(uint8_t DOUTPIN, uint8_t DINPIN);
void SetPDMPIN(uint8_t CLKOUTPIN, uint8_t DINPIN);
void SetLLEAntPIN(uint8_t PINNum, uint32_t AntPIN[8]);
void SetSlpWkupSource(uint8_t Idx, uint8_t Pol, uint8_t Mode);
void SetDpslpWkupSource(uint8_t Idx, uint8_t Pol);
void SetGpioInput(uint8_t GPIOIdx);
void SetGpioOutput(uint8_t GPIOIdx);
void SetGpioOutputValue(uint8_t GPIOIdx, uint8_t Value);
uint8_t GetGpioInputValue(uint8_t GPIOIdx);

////////////////////////////////////////
//  Cache
////////////////////////////////////////
void ICacheIni(uint8_t CacheSizeSel);
void ICacheEnable(void);
void ICacheDisable(void);
void ICacheFlush(void);
void DCacheIni(uint8_t CacheSizeSel);
void DCacheEnable(void);
void DCacheDisable(void);
void DCacheFlush(void);

////////////////////////////////////////
//  PAD
////////////////////////////////////////
void Aon1IOLatch(void);
void Aon1IORelease(void);
void Aon2IORetentionRelease(void);
void Aon2IORetention(void);
void Aon2IOHighZRelease(void);
void Aon2IOHighZ(void);

////////////////////////////////////////
//  UART
////////////////////////////////////////
void UartInit(volatile UART_REG_H* uart_reg, uint32_t FlowCtrl, uint32_t ClkFreq, uint32_t BaudRate);
void UartTx(volatile UART_REG_H* uart_reg, uint8_t ch);
uint8_t UartRx(volatile UART_REG_H* uart_reg);
char* UartRxString(volatile UART_REG_H* uart_reg);
void WaitKeyboard(void);

////////////////////////////////////////
//  TIMER
////////////////////////////////////////
void DelayS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt);
void DelayMS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt);
void DelayUS(volatile TIMER_REG_H* timer_reg, uint32_t Freq, uint32_t Cnt);

////////////////////////////////////////
//  Clock
////////////////////////////////////////
void Sel32KSource(uint8_t Sel);

////////////////////////////////////////
//  MISC
////////////////////////////////////////
void LLECalibration(void);
void nop(uint32_t n);
void jump(int addr);

#endif
