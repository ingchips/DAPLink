;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V1.08
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2011 - 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000C00

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0x40003109                         ; Reserved
                DCD     0         ; DAPLINK: Build type (BL/IF)
                DCD     0            ; DAPLINK: Compatibility
                DCD     0           ; DAPLINK: Version
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0              ; DAPLINK: Pointer to board/family/target info
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     IRQHandler_CacheI                
                DCD     IRQHandler_CacheD                
                DCD     IRQHandler_RTCCnt             
                DCD     IRQHandler_GPIO1              
                DCD     IRQHandler_GPIO0                
                DCD     IRQHandler_Timer2               
                DCD     IRQHandler_Timer1               
                DCD     IRQHandler_Timer0                
                DCD     IRQHandler_WDT                 
                DCD     IRQHandler_PDM                  
                DCD     IRQHandler_AHBAES                
                DCD     IRQHandler_LLEErr                
                DCD     IRQHandler_LLEFun                
                DCD     IRQHandler_SPIFlash             
                DCD     IRQHandler_APBSPI               
                DCD     IRQHandler_QSPI                 
                DCD     IRQHandler_SADC                 
                DCD     IRQHandler_I2S                  
                DCD     IRQHandler_Uart1                 
                DCD     IRQHandler_Uart0                 
                DCD     IRQHandler_I2C1                  
                DCD     IRQHandler_I2C0                  
                DCD     IRQHandler_DMA                  
                DCD     IRQHandler_KeyScan              
                DCD     IRQHandler_c0_int_pwm          
                DCD     IRQHandler_TRNG                 
                DCD     IRQHandler_IR_INT           
                DCD     IRQHandler_IR_WKUP         
                DCD     IRQHandler_QDEC2           
                DCD     IRQHandler_QDEC1           
                DCD     IRQHandler_QDEC0           
                DCD     IRQHandler_usb             
                DCD     IRQHandler_lpc_pos         
                DCD     IRQHandler_lpc_neg         
                DCD     IRQHandler_rcmfd_trim_done 
                DCD     IRQHandler_pmu_pvd         
                DCD     IRQHandler_pmu_pdr         
                DCD     IRQHandler_c1_int_pwm      
                DCD     IRQHandler_c2_int_pwm      
                DCD     IRQHandler_c0_ppi_int      
                DCD     IRQHandler_c1_ppi_int      
                DCD     IRQHandler_c2_ppi_int      
                DCD     IRQHandler_c3_ppi_int      
                DCD     IRQHandler_spi2ahb         
                DCD     IRQHandler_xo_ready        
                DCD     IRQHandler_lle_domain_reg  
                DCD     IRQHandler_actcnt_32k      
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  init_memory

				LDR     R0, =init_memory
                BLX     R0
                
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT IRQHandler_CacheI           [WEAK]
                EXPORT IRQHandler_CacheD           [WEAK]
                EXPORT IRQHandler_RTCCnt           [WEAK]
                EXPORT IRQHandler_GPIO1            [WEAK]
                EXPORT IRQHandler_GPIO0            [WEAK]
                EXPORT IRQHandler_Timer2           [WEAK]  
                EXPORT IRQHandler_Timer1           [WEAK]
                EXPORT IRQHandler_Timer0           [WEAK]
                EXPORT IRQHandler_WDT              [WEAK]
                EXPORT IRQHandler_PDM              [WEAK]
                EXPORT IRQHandler_AHBAES           [WEAK]
                EXPORT IRQHandler_LLEErr           [WEAK]
                EXPORT IRQHandler_LLEFun           [WEAK]
                EXPORT IRQHandler_SPIFlash         [WEAK]
                EXPORT IRQHandler_APBSPI           [WEAK]
                EXPORT IRQHandler_QSPI             [WEAK]
                EXPORT IRQHandler_SADC             [WEAK]
                EXPORT IRQHandler_I2S              [WEAK]
                EXPORT IRQHandler_Uart1            [WEAK]
                EXPORT IRQHandler_Uart0            [WEAK]
                EXPORT IRQHandler_I2C1             [WEAK]
                EXPORT IRQHandler_I2C0             [WEAK]
                EXPORT IRQHandler_DMA              [WEAK]
                EXPORT IRQHandler_KeyScan          [WEAK]
                EXPORT IRQHandler_c0_int_pwm       [WEAK]
                EXPORT IRQHandler_TRNG             [WEAK]
                EXPORT IRQHandler_IR_INT           [WEAK]
                EXPORT IRQHandler_IR_WKUP          [WEAK]
                EXPORT IRQHandler_QDEC2            [WEAK]
                EXPORT IRQHandler_QDEC1            [WEAK]
                EXPORT IRQHandler_QDEC0            [WEAK]
                EXPORT IRQHandler_usb              [WEAK]
                EXPORT IRQHandler_lpc_pos          [WEAK]
                EXPORT IRQHandler_lpc_neg          [WEAK]
                EXPORT IRQHandler_rcmfd_trim_done  [WEAK]
                EXPORT IRQHandler_pmu_pvd          [WEAK]
                EXPORT IRQHandler_pmu_pdr          [WEAK]
                EXPORT IRQHandler_c1_int_pwm       [WEAK]
                EXPORT IRQHandler_c2_int_pwm       [WEAK]
                EXPORT IRQHandler_c0_ppi_int       [WEAK]
                EXPORT IRQHandler_c1_ppi_int       [WEAK]
                EXPORT IRQHandler_c2_ppi_int       [WEAK]
                EXPORT IRQHandler_c3_ppi_int       [WEAK]
                EXPORT IRQHandler_spi2ahb          [WEAK]
                EXPORT IRQHandler_xo_ready         [WEAK]
                EXPORT IRQHandler_lle_domain_reg   [WEAK]
                EXPORT IRQHandler_actcnt_32k       [WEAK]


IRQHandler_CacheI           
IRQHandler_CacheD           
IRQHandler_RTCCnt           
IRQHandler_GPIO1            
IRQHandler_GPIO0            
IRQHandler_Timer2           
IRQHandler_Timer1           
IRQHandler_Timer0           
IRQHandler_WDT              
IRQHandler_PDM              
IRQHandler_AHBAES           
IRQHandler_LLEErr           
IRQHandler_LLEFun           
IRQHandler_SPIFlash         
IRQHandler_APBSPI           
IRQHandler_QSPI             
IRQHandler_SADC             
IRQHandler_I2S              
IRQHandler_Uart1            
IRQHandler_Uart0            
IRQHandler_I2C1             
IRQHandler_I2C0             
IRQHandler_DMA              
IRQHandler_KeyScan          
IRQHandler_c0_int_pwm       
IRQHandler_TRNG             
IRQHandler_IR_INT           
IRQHandler_IR_WKUP          
IRQHandler_QDEC2            
IRQHandler_QDEC1            
IRQHandler_QDEC0            
IRQHandler_usb              
IRQHandler_lpc_pos          
IRQHandler_lpc_neg          
IRQHandler_rcmfd_trim_done  
IRQHandler_pmu_pvd          
IRQHandler_pmu_pdr          
IRQHandler_c1_int_pwm       
IRQHandler_c2_int_pwm       
IRQHandler_c0_ppi_int       
IRQHandler_c1_ppi_int       
IRQHandler_c2_ppi_int       
IRQHandler_c3_ppi_int       
IRQHandler_spi2ahb          
IRQHandler_xo_ready         
IRQHandler_lle_domain_reg   
IRQHandler_actcnt_32k       

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
