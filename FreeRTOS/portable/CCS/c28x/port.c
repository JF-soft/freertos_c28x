//-------------------------------------------------------------------------------------------------
// Author: Ivan Zaitsev, ivan.zaitsev@gmail.com
// Modified by: Jonatan R. Fischer, jonafischer@gmail.com
//
// This file follows the FreeRTOS distribution license.
//
// FreeRTOS is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License (version 2) as published by the
// Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.
//
// ***************************************************************************
// >>!   NOTE: The modification to the GPL is included to allow you to     !<<
// >>!   distribute a combined work that includes FreeRTOS without being   !<<
// >>!   obliged to provide the source code for proprietary components     !<<
// >>!   outside of the FreeRTOS kernel.                                   !<<
// ***************************************************************************
//
// FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  Full license text is available on the following
// link: http://www.freertos.org/a00114.html
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Scheduler includes.
//-------------------------------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "task.h"

//-------------------------------------------------------------------------------------------------
// Implementation of functions defined in portable.h for the C28x port.
//-------------------------------------------------------------------------------------------------

// Constants required for hardware setup.
#define portINITIAL_CRITICAL_NESTING  ( ( uint16_t ) 10 )
#define portFLAGS_INT_ENABLED         ( ( StackType_t ) 0x08 )
#if defined(__TMS320C28XX_FPU32__)
# define AUX_REGISTERS_TO_SAVE        19 // XAR + FPU registers
# define STF_REGISTER_POSITION        10 // STF position in AUX registers array
#else
# define AUX_REGISTERS_TO_SAVE        9  // XAR registers only
#endif

#define XAR4_REGISTER_POSITION       5  // XAR4 position in AUX registers array

extern uint32_t getSTF( void );
extern void portTICK_ISR( void );

// Each task maintains a count of the critical section nesting depth and the
// value of IER before entering that critical section. Each time a critical
// section is entered the count (CSNC) is incremented.  Each time a
// critical section is exited the count is decremented - with the value of IER
// only being restored if the count is zero. Critical interrupts defined by
// portCRITICAL_INTERRUPT_MASK are enabled. If portCRITICAL_INTERRUPT_MASK == 0x0000
// global interrupts using INTM bit are enabled or disabled directly. During
// context switch (portTICK_ISR), scheduler startup and first context restore
// (portRESTORE_FIRST_CONTEXT) interrupts are globally disabled using INTM.
//
// ulCriticalVars.CSNC will get set to zero when the scheduler starts, but must
// not be initialized to zero as this will cause problems during the startup
// sequence.
//
// As ulCriticalVars is a 32 bit value, stack alignment remains unchanged.

// typedef to ease access of ulCriticalVars data in C. This typedef is not used in
// assembly, the value is copied as a 32bit value using direct addressing mode.
typedef volatile union {
    struct {
        uint16_t CSNC;  // Critical Section Nesting Counter
        uint16_t IER;   // Interrupt Enable Register before entering critical section
    };
    uint32_t all;
} CriticalVarsTask_t;

// Critical section variables for each task (this platform is little endian)
volatile uint32_t ulCriticalVars = 0xFFFF & portINITIAL_CRITICAL_NESTING;
volatile uint16_t bInit = 0;

//-------------------------------------------------------------------------------------------------
// Initialise the stack of a task to look exactly as if
// timer interrupt was executed.
//-------------------------------------------------------------------------------------------------
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
  uint16_t i;
  uint16_t base = 0;

  pxTopOfStack[base++]  = 0x0080;  // ST0. PSM = 0(No shift)
  pxTopOfStack[base++]  = 0x0000;  // T
  pxTopOfStack[base++]  = 0x0000;  // AL
  pxTopOfStack[base++]  = 0x0000;  // AH
  pxTopOfStack[base++]  = 0xFFFF;  // PL
  pxTopOfStack[base++]  = 0xFFFF;  // PH
  pxTopOfStack[base++]  = 0xFFFF;  // AR0
  pxTopOfStack[base++]  = 0xFFFF;  // AR1
  pxTopOfStack[base++]  = 0x8A08;  // ST1
  pxTopOfStack[base++]  = 0x0000;  // DP
  pxTopOfStack[base++] = 0x2000 | IER;    // IER at the moment of creating tasks, make sure INT14 is enabled
  pxTopOfStack[base++] = 0x0000;  // DBGSTAT
  pxTopOfStack[base++] = ((uint32_t)pxCode) & 0xFFFFU;       // PCL
  pxTopOfStack[base++] = ((uint32_t)pxCode >> 16) & 0x00FFU; // PCH
  pxTopOfStack[base++] = 0xAAAA;  // Alignment
  pxTopOfStack[base++] = 0xBBBB;  // Alignment

  // Fill the rest of the registers with dummy values.
  for(i = 0; i < (2 * AUX_REGISTERS_TO_SAVE); i++)
  {
    uint16_t low  = 0x0000;
    uint16_t high = 0x0000;

    if(i == (2 * XAR4_REGISTER_POSITION))
    {
      low  = ((uint32_t)pvParameters) & 0xFFFFU;
      high = ((uint32_t)pvParameters >> 16) & 0xFFFFU;
    }

#if defined(__TMS320C28XX_FPU32__)
    if(i == (2 * STF_REGISTER_POSITION))
    {
      uint32_t stf = getSTF();

      low  = stf & 0xFFFFU;
      high = (stf >> 16) & 0xFFFFU;
    }
#endif

    pxTopOfStack[base + i] = low;
    i++;
    pxTopOfStack[base + i] = high;
  }

  base += i;

  // Reserve place for ST1 which will be used in context switch
  // to set correct SPA bit ASAP.
  pxTopOfStack[base++] = 0x8A18;  // ST1 with SPA bit set
  pxTopOfStack[base++] = 0x0000;  // DP
  pxTopOfStack[base++] = 0x0000;  // placeholder for 32 bit ulCriticalVars
  pxTopOfStack[base++] = 0x0000;

  // Return a pointer to the top of the stack we have generated so this can
  // be stored in the task control block for the task.
  return pxTopOfStack + base;
}

//-------------------------------------------------------------------------------------------------
static void vApplicationSetupTimerInterrupt(void)
{
  // pointers to cpu peripherals
  volatile uint32_t *PIE_VECT = (uint32_t *)portPIE_VECT_BASE;
  volatile uint16_t *CPU_TIM2 = (uint16_t *)portCPU_TIM2_BASE;

  // Start the timer that activates timer interrupt to switch into first task.
  __asm(" EALLOW");
  PIE_VECT[portCPU_TIM2_ISR] = (uint32_t)&portTICK_ISR;
  __asm(" EDIS");

  // Initialize timer period
  uint32_t tmp = (uint32_t)(configCPU_CLOCK_HZ / configTICK_RATE_HZ);
  CPU_TIM2[2] = tmp & 0xFFFFU;
  CPU_TIM2[3] = (tmp >> 16) & 0xFFFFU;
  CPU_TIM2[4] = 0x4C30;   // Configure timer
  CPU_TIM2[4] = 0x4000;   // Enable interrupt and start timer

  // Enable CPU_TIMER2 at cpu level
  IER |= 0x2000;
}

//-------------------------------------------------------------------------------------------------
void vPortEndScheduler( void )
{
  // It is unlikely that the TMS320 port will get stopped.
  // If required simply disable the tick interrupt here.
}

//-------------------------------------------------------------------------------------------------
// See header file for description.
//-------------------------------------------------------------------------------------------------
BaseType_t xPortStartScheduler(void)
{
  CriticalVarsTask_t *tmp = (CriticalVarsTask_t *)&ulCriticalVars;

  // initialize Timer 2 for Tick operation
  vApplicationSetupTimerInterrupt();

  // flag as initialized
  bInit = 1;

  // update critical section nesting counter
  tmp->CSNC = 0;

  portENABLE_INTERRUPTS();
  portRESTORE_FIRST_CONTEXT();

  // Should not get here!
  return pdFAIL;
}

//-------------------------------------------------------------------------------------------------
void vPortEnterCritical( void )
{
  CriticalVarsTask_t *tmp = (CriticalVarsTask_t *)&ulCriticalVars;

  portDISABLE_INTERRUPTS();

#if (portCRITICAL_INTERRUPT_MASK > 0)
  // At first CSNC is a big value, to allow OS initialization before timer tick is up and running
  if(bInit == 1)
  {
    // save IER value before entering critical section if not already masked
    if(IER & ~portCRITICAL_INTERRUPT_MASK != 0)
    {  // Mask was not applied yet, save IER
      tmp->IER = IER;
      // mask IER for allowing only critical interrupts
      IER &= portCRITICAL_INTERRUPT_MASK;
    }
    portENABLE_INTERRUPTS();
  }
#endif
  // update critical nesting counter
  tmp->CSNC += 1;
}

//-------------------------------------------------------------------------------------------------
void vPortExitCritical( void )
{
  CriticalVarsTask_t *tmp = (CriticalVarsTask_t *)&ulCriticalVars;

  // update critical section nesting counter
  tmp->CSNC -= 1;

  // restore initial IER value
  if(tmp->CSNC == 0)
  {
#if (portCRITICAL_INTERRUPT_MASK > 0)
    portDISABLE_INTERRUPTS();
    IER = tmp->IER;
#endif
    portENABLE_INTERRUPTS();
  }
}
