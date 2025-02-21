// ----------------------------------------------------------------------------
// Copyright Message                                                           
// ----------------------------------------------------------------------------
//                                                                             
// INGCHIPS confidential and proprietary.                                      
// COPYRIGHT (c) 2018 by INGCHIPS                                              
//                                                                             
// All rights are reserved. Reproduction in whole or in part is                
// prohibited without the written consent of the copyright owner.              
//                                                                             
// ----------------------------------------------------------------------------
// Design Information                                                          
// ----------------------------------------------------------------------------
//                                                                             
// File         : cm4.h
//                                                                             
// Author       : Lujie JI
//                                                                             
// Organisation : INGCHIPS                                                     
//                                                                             
// Date         : 2021-01-09
//                                                                             
// Revision     : v0.1                                                         
//                                                                             
// Project      : ING91800                                                     
//                                                                             
// Description  :                                                              
//                                                                             
// ----------------------------------------------------------------------------
// Revision History                                                            
// ----------------------------------------------------------------------------
//                                                                             
// Date       Author          Revision  Change Description                     
// ========== =============== ========= =======================================
// 2021/01/09 Lujie JI        v0.1      Create
//                                                                             
// ----------------------------------------------------------------------------
#ifndef CM4__H
#define CM4__H

/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

/* Memory mapping of Cortex-M4 Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct */
#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */
#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */

#if (__MPU_PRESENT == 1U)
  #define MPU_BASE          (SCS_BASE +  0x0D90UL)                    /*!< Memory Protection Unit */
  #define MPU               ((MPU_Type       *)     MPU_BASE      )   /*!< Memory Protection Unit */
#endif

#if (__FPU_PRESENT == 1U)
  #define FPU_BASE          (SCS_BASE +  0x0F30UL)                    /*!< Floating Point Unit */
  #define FPU               ((FPU_Type       *)     FPU_BASE      )   /*!< Floating Point Unit */
#endif

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM  int  CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM int  ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM int  VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM int  AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM int  SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM int  CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM char SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM int  SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM int  CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM int  HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM int  DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM int  MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM int  BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM int  AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  int  PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  int  DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  int  ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  int  MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  int  ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        int  RESERVED0[5U];
  __IOM int  CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

/**
  \brief  St ructure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM int  ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        int  RESERVED0[24U];
  __IOM int  ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        int  RSERVED1[24U];
  __IOM int  ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        int  RESERVED2[24U];
  __IOM int  ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        int  RESERVED3[24U];
  __IOM int  IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        int  RESERVED4[56U];
  __IOM char IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        int  RESERVED5[644U];
  __OM  int  STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

typedef enum IRQn
{
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt          */
  HardFault_IRQn                = -13,      /*!<  3 HardFault Interrupt             */
  MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt     */
  BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt             */
  UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt           */
  SVCall_IRQn                   =  -5,      /*!< 11 SV Call Interrupt               */
  DebugMonitor_IRQn             =  -4,      /*!< 12 Debug Monitor Interrupt         */
  PendSV_IRQn                   =  -2,      /*!< 14 Pend SV Interrupt               */
  SysTick_IRQn                  =  -1,      /*!< 15 System Tick Interrupt           */

/******  CM4 specific Interrupt Numbers *********************************************************/
  n00_IRQn               =  0 ,
  n01_IRQn               =  1 ,
  n02_IRQn               =  2 ,
  n03_IRQn               =  3 ,
  n04_IRQn               =  4 ,
  n05_IRQn               =  5 ,
  n06_IRQn               =  6 ,
  n07_IRQn               =  7 ,
  n08_IRQn               =  8 ,
  n09_IRQn               =  9 ,
  n10_IRQn               =  10,
  n11_IRQn               =  11,
  n12_IRQn               =  12,
  n13_IRQn               =  13,
  n14_IRQn               =  14,
  n15_IRQn               =  15,
  n16_IRQn               =  16,
  n17_IRQn               =  17,
  n18_IRQn               =  18,
  n19_IRQn               =  19,
  n20_IRQn               =  20,
  n21_IRQn               =  21,
  n22_IRQn               =  22,
  n23_IRQn               =  23,
  n24_IRQn               =  24,
  n25_IRQn               =  25,
  n26_IRQn               =  26,
  n27_IRQn               =  27,
  n28_IRQn               =  28,
  n29_IRQn               =  29,
  n30_IRQn               =  30,
  n31_IRQn               =  31,
  n32_IRQn               =  32,
  n33_IRQn               =  33,
  n34_IRQn               =  34,
  n35_IRQn               =  35,
  n36_IRQn               =  36,
  n37_IRQn               =  37,
  n38_IRQn               =  38,
  n39_IRQn               =  39,
  n40_IRQn               =  40,
  n41_IRQn               =  41,
  n42_IRQn               =  42,
  n43_IRQn               =  43,
  n44_IRQn               =  44,
  n45_IRQn               =  45,
  n46_IRQn               =  46,
} IRQn_Type;

#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */

#define SCB_AIRCR_PRIGROUP_Pos              8                                             /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2                                             /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

void NVIC_SetVectorTable(int NVIC_VectTab, int Offset);
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);
void NVIC_EnableIRQ(IRQn_Type IRQn);
void NVIC_SystemReset(void);

#endif
