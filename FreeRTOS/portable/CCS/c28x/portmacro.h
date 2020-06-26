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

#ifndef PORTMACRO_H
#define PORTMACRO_H

//-------------------------------------------------------------------------------------------------
// Port specific definitions.
//
// The settings in this file configure FreeRTOS correctly for the
// given hardware and compiler.
//
// These settings should not be altered.
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// OS includes
//-------------------------------------------------------------------------------------------------
// FreeRTOS config include (to make assembler work with conditional compilation)
#include "FreeRTOSConfig.h"

//-------------------------------------------------------------------------------------------------
// Hardware includes
//-------------------------------------------------------------------------------------------------
#include <stdint.h>

//-------------------------------------------------------------------------------------------------
// Type definitions.
//-------------------------------------------------------------------------------------------------
#define portCHAR        uint16_t
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        uint32_t
#define portSHORT       uint16_t
#define portBASE_TYPE   uint16_t
#define uint8_t         uint16_t
#define int8_t          int16_t
#define portSTACK_TYPE  uint16_t

typedef portSTACK_TYPE StackType_t;
typedef int16_t        BaseType_t;
typedef uint16_t       UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
  typedef uint16_t TickType_t;
  #define portMAX_DELAY ( TickType_t ) 0xffff
#else
  typedef uint32_t TickType_t;
  #define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif

//-------------------------------------------------------------------------------------------------
// Peripheral definitions.
//-------------------------------------------------------------------------------------------------
// single core DSP have these peripherals at the same location
// to run in another core, fix locations
#define portPIE_VECT_BASE  0x000D00
#define portCPU_TIM2_BASE  0x000C10
#define portCPU_TIM2_ISR   0x00001C >> 1
// IER, IFR register definitions
extern cregister volatile unsigned int IER;
extern cregister volatile unsigned int IFR;

//-------------------------------------------------------------------------------------------------
// Interrupt control macros.
//-------------------------------------------------------------------------------------------------
// example of treating INT4 only as high priority interrupt rebuild (clean+build) is required when
// changing this value (to force rebuild of porASM.asm)
#define portDISABLE_INTERRUPTS()  __asm(" SETC INTM")
#define portENABLE_INTERRUPTS()   __asm(" CLRC INTM")

// 0 to get normal behavior don't enable RTOSINT, DLOGINT
#define portCRITICAL_INTERRUPT_MASK  0x0000

#if (configUSE_PREEMPTION == 1)
# define portUSE_PREEMPTION  0x0001
#else
# define portUSE_PREEMPTION  0x0000
#endif

//-------------------------------------------------------------------------------------------------
// Critical section control macros.
//-------------------------------------------------------------------------------------------------
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );
#define portENTER_CRITICAL()  vPortEnterCritical()
#define portEXIT_CRITICAL()   vPortExitCritical()

//-------------------------------------------------------------------------------------------------
// Task utilities.
//-------------------------------------------------------------------------------------------------
#define portYIELD_OPERATION()  __asm(" OR IFR, #0x2000")

#define portYIELD() do{portYIELD_OPERATION();}while(0)
#define portYIELD_FROM_ISR( x )  do{if(x == pdTRUE){portYIELD_OPERATION();}}while(0)

extern void portRESTORE_FIRST_CONTEXT( void );
extern void vTaskSwitchContext( void );

// Functions that should called inside interrupts when using critical
// interrupts, it enables nesting inside kernel aware interrupts.
#if (portCRITICAL_INTERRUPT_MASK == 0x0000)

static inline __attribute__((always_inline))
void portPROLOG_ISR( void )
{
  // do nothing
}

static inline __attribute__((always_inline))
void portEPILOG_ISR( void )
{
  // do nothing
}

#else

static inline __attribute__((always_inline))
void portPROLOG_ISR( void )
{
  IER &= portCRITICAL_INTERRUPT_MASK;
  portENABLE_INTERRUPTS();
}

static inline __attribute__((always_inline))
void portEPILOG_ISR( void )
{
  portDISABLE_INTERRUPTS();
}

#endif

//-------------------------------------------------------------------------------------------------
// Hardware specifics.
//-------------------------------------------------------------------------------------------------
#define portBYTE_ALIGNMENT      4
#define portSTACK_GROWTH        ( 1 )
#define portTICK_PERIOD_MS      ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portNOP()               __asm(" NOP")

//-------------------------------------------------------------------------------------------------
// Task function macros as described on the FreeRTOS.org WEB site.
//-------------------------------------------------------------------------------------------------
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#endif /* PORTMACRO_H */
