/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2021 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: 3.30                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_NoOS.c
Purpose : Sample setup configuration of SystemView without an OS for Cortex-M0.
Revision: $Rev: 18540 $
*/
#include "SEGGER_SYSVIEW_Conf.h"
#include "SEGGER_SYSVIEW.h"

// SystemcoreClock can be used in most CMSIS compatible projects.
// In non-CMSIS projects define SYSVIEW_CPU_FREQ.
extern unsigned int SystemCoreClock;
extern unsigned int SEGGER_SYSVIEW_TickCnt;

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define SCB_ICSR  (*(volatile U32*) (0xE000ED04uL)) // Interrupt Control State Register
#define SCB_ICSR_PENDSTSET_MASK     (1UL << 26)     // SysTick pending bit
#define SYST_RVR  (*(volatile U32*) (0xE000E014uL)) // SysTick Reload Value Register
#define SYST_CVR  (*(volatile U32*) (0xE000E018uL)) // SysTick Current Value Register

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "System View Demo"

// The target device name
#define SYSVIEW_DEVICE_NAME     "STM32F051R8"
#define SYSVIEW_CORE_NAME       "Cortex-M0"
#define SYSVIEW_OS_NAME         "Non-OS_Cortex-M0"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (SystemCoreClock)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        (SystemCoreClock)

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x20000000)

// Define as 1 if the Cortex-M cycle counter is used as SystemView timestamp. Must match SEGGER_SYSVIEW_Conf.h
#ifndef   USE_CYCCNT_TIMESTAMP
  #define USE_CYCCNT_TIMESTAMP    1
#endif

// Define as 1 if the Cortex-M cycle counter is used and there might be no debugger attached while recording.
#ifndef   ENABLE_DWT_CYCCNT
  #define ENABLE_DWT_CYCCNT       (USE_CYCCNT_TIMESTAMP & SEGGER_SYSVIEW_POST_MORTEM_MODE)
#endif

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define DEMCR                     (*(volatile unsigned long*) (0xE000EDFCuL))   // Debug Exception and Monitor Control Register
#define TRACEENA_BIT              (1uL << 24)                                   // Trace enable bit
#define DWT_CTRL                  (*(volatile unsigned long*) (0xE0001000uL))   // DWT Control Register
#define NOCYCCNT_BIT              (1uL << 25)                                   // Cycle counter support bit
#define CYCCNTENA_BIT             (1uL << 0)                                    // Cycle counter enable bit

/********************************************************************* 
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME","
                             "D="SYSVIEW_DEVICE_NAME","
                             "C="SYSVIEW_CORE_NAME","
                             "O="SYSVIEW_OS_NAME);
//  SEGGER_SYSVIEW_SendSysDesc("I#11=SVC_Handler");
//  SEGGER_SYSVIEW_SendSysDesc("I#14=PendSV_Handler");
  SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");
//  SEGGER_SYSVIEW_SendSysDesc("I#16=WWDG_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#17=PVD_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#18=RTC_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#19=FLASH_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#20=RCC_CRS_IRQHandler");
  SEGGER_SYSVIEW_SendSysDesc("I#21=EXTI0_1_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#22=EXTI2_3_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#23=EXTI4_15_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#24=TSC_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#25=DMA1_Channel1_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#26=DMA1_Channel2_3_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#27=DMA1_Channel4_5_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#28=ADC1_COMP_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#29=TIM1_BRK_UP_TRG_COM_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#30=TIM1_CC_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#31=TIM2_IRQHandler");
  SEGGER_SYSVIEW_SendSysDesc("I#32=TIM3_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#33=TIM6_DAC_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#35=TIM14_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#36=TIM15_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#37=TIM16_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#38=TIM17_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#39=I2C1_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#40=I2C2_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#41=SPI1_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#42=SPI2_IRQHandler");
  SEGGER_SYSVIEW_SendSysDesc("I#43=USART1_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#44=USART2_IRQHandler");
//  SEGGER_SYSVIEW_SendSysDesc("I#46=CEC_CAN_IRQHandler");

//  SEGGER_SYSVIEW_NameMarker(APP_MARKER_UART_TX_BLOCKING, "UART_TX_Blocking");
//  SEGGER_SYSVIEW_NameMarker(APP_MARKER_UART_TX_INTERRUPT, "UART_TX_Interrupt");

  SEGGER_SYSVIEW_NameMarker(APP_MARKER_UART_TX_BLOCKING, "UART_TX_Blocking");
  SEGGER_SYSVIEW_NameMarker(APP_MARKER_UART_TX_INTERRUPT, "UART_TX_Interrupt");
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
#if USE_CYCCNT_TIMESTAMP
#if ENABLE_DWT_CYCCNT
  //
  // If no debugger is connected, the DWT must be enabled by the application
  //
  if ((DEMCR & TRACEENA_BIT) == 0) {
    DEMCR |= TRACEENA_BIT;
  }
#endif
  //
  //  The cycle counter must be activated in order
  //  to use time related functions.
  //
  if ((DWT_CTRL & NOCYCCNT_BIT) == 0) {       // Cycle counter supported?
    if ((DWT_CTRL & CYCCNTENA_BIT) == 0) {    // Cycle counter not enabled?
      DWT_CTRL |= CYCCNTENA_BIT;              // Enable Cycle counter
    }
  }
#endif
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, 
                      0, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_X_GetTimestamp()
*
* Function description
*   Returns the current timestamp in ticks using the system tick 
*   count and the SysTick counter.
*   All parameters of the SysTick have to be known and are set via
*   configuration defines on top of the file.
*
* Return value
*   The current timestamp.
*
* Additional information
*   SEGGER_SYSVIEW_X_GetTimestamp is always called when interrupts are
*   disabled. Therefore locking here is not required.
*/
U32 SEGGER_SYSVIEW_X_GetTimestamp(void) {
#if USE_CYCCNT_TIMESTAMP
  U32 TickCount;
  U32 Cycles;
  U32 CyclesPerTick;
  //
  // Get the cycles of the current system tick.
  // SysTick is down-counting, subtract the current value from the number of cycles per tick.
  //
  CyclesPerTick = SYST_RVR + 1;
  Cycles = (CyclesPerTick - SYST_CVR);
  //
  // Get the system tick count.
  //
  TickCount = SEGGER_SYSVIEW_TickCnt;
  //
  // If a SysTick interrupt is pending, re-read timer and adjust result
  //
  if ((SCB_ICSR & SCB_ICSR_PENDSTSET_MASK) != 0) {
    Cycles = (CyclesPerTick - SYST_CVR);
    TickCount++;
  }
  Cycles += TickCount * CyclesPerTick;

  return Cycles;
#endif
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_X_GetInterruptId()
*
* Function description
*   Return the currently active interrupt Id,
*   which ist the active vector taken from IPSR[5:0].
*
* Return value
*   The current currently active interrupt Id.
*
* Additional information
*   This function is not used by default, as the active vector can be 
*   read from ICSR instead on Cortex-M0.
*   For Cortex-M0+ devices, change SEGGER_SYSVIEW_GET_INTERRUPT_ID
*   in SEGGER_SYSVIEW_Conf.h to call this function instead.
*/
U32 SEGGER_SYSVIEW_X_GetInterruptId(void) {
  U32 Id;

  __asm volatile ("mrs %0, ipsr"
                  : "=r" (Id)
                  );
  Id &= 0x3F;

  return Id;
}

/*************************** End of file ****************************/
