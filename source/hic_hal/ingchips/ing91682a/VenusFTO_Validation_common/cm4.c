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
// File         : cm4.c
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
#ifndef CM4__C
#define CM4__C

#include "cm4.h"

void NVIC_SetVectorTable(int NVIC_VectTab, int Offset)
{ 
    /* Check the parameters */ 
    SCB->VTOR = NVIC_VectTab | (Offset & (int)0xFFFFFF80);
}

void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    NVIC->ICPR[((int)(IRQn) >> 5)] = (1 << ((int)(IRQn) & 0x1F)); /* Clear pending interrupt */
}

void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    NVIC->ISER[((int)(IRQn) >> 5)] = (1 << ((int)(IRQn) & 0x1F)); /* enable interrupt */
}

void NVIC_SystemReset(void)
{
    __asm volatile ("dsb 0xF");
    
    SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
                   (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                   SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
    __asm volatile ("dsb 0xF");
    while(1);                                                    /* wait until reset */
}

#endif
