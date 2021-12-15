;/**************************************************************************//**
; * @file     startup_NUC1263_user.s
; * @version  V3.00
; * @brief    NUC1263 Series Startup Source File
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
; ******************************************************************************/
    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000400
    ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000000
    ENDIF

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
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main


                LDR     R0, =SystemInit
                BLX     R0
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
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
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

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  EINT024_IRQHandler        [WEAK]
                EXPORT  EINT135_IRQHandler        [WEAK]
                EXPORT  GPAB_IRQHandler           [WEAK]
                EXPORT  GPCDF_IRQHandler          [WEAK]
                EXPORT  BPWM0_IRQHandler          [WEAK]
                EXPORT  BPWM1_IRQHandler          [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  BPWM2_IRQHandler          [WEAK]
                EXPORT  BPWM3_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  USBD_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  CLKDIRC_IRQHandler        [WEAK]
                EXPORT  LLSI0_IRQHandler          [WEAK]
                EXPORT  LLSI1_IRQHandler          [WEAK]
                EXPORT  LLSI2_IRQHandler          [WEAK]
                EXPORT  LLSI3_IRQHandler          [WEAK]
                EXPORT  LLSI4_IRQHandler          [WEAK]
                EXPORT  LLSI5_IRQHandler          [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  I3C0_IRQHandler           [WEAK]
                EXPORT  I3C1_IRQHandler           [WEAK]
                EXPORT  DAC_IRQHandler            [WEAK]
                EXPORT  ACMP23_IRQHandler         [WEAK]
                EXPORT  TEMP_IRQHandler           [WEAK]
                EXPORT	DEFAULT_IRQHandler		  [WEAK]                                                
                
BOD_IRQHandler
WDT_IRQHandler
EINT024_IRQHandler
EINT135_IRQHandler
GPAB_IRQHandler
GPCDF_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
BPWM2_IRQHandler
BPWM3_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
I2C2_IRQHandler
USBD_IRQHandler
ACMP01_IRQHandler
PDMA_IRQHandler
PWRWU_IRQHandler
ADC_IRQHandler
CLKDIRC_IRQHandler
LLSI0_IRQHandler
LLSI1_IRQHandler
LLSI2_IRQHandler
LLSI3_IRQHandler
LLSI4_IRQHandler
LLSI5_IRQHandler
SPI2_IRQHandler
UART2_IRQHandler
I3C0_IRQHandler
I3C1_IRQHandler
DAC_IRQHandler
ACMP23_IRQHandler
TEMP_IRQHandler
DEFAULT_IRQHandler
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
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP


                ALIGN

                ENDIF

                END
