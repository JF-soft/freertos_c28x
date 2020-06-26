;-------------------------------------------------------------------------------------------------
; Author: Ivan Zaitsev, ivan.zaitsev@gmail.com
; Modified by: Jonatan R. Fischer, jonafischer@gmail.com;
;
; This file follows the FreeRTOS distribution license.
;
; FreeRTOS is free software; you can redistribute it and/or modify it under
; the terms of the GNU General Public License (version 2) as published by the
; Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.
;
; ***************************************************************************
; >>!   NOTE: The modification to the GPL is included to allow you to     !<<
; >>!   distribute a combined work that includes FreeRTOS without being   !<<
; >>!   obliged to provide the source code for proprietary components     !<<
; >>!   outside of the FreeRTOS kernel.                                   !<<
; ***************************************************************************
;
; FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
; WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
; FOR A PARTICULAR PURPOSE.  Full license text is available on the following
; link: http://www.freertos.org/a00114.html
;-------------------------------------------------------------------------------------------------

;-------------------------------------------------------------------------------------------------
; C includes (remember to rebuild when changing portmacro.h)
;-------------------------------------------------------------------------------------------------
  .cdecls C,LIST,"portmacro.h"

;-------------------------------------------------------------------------------------------------
; functions defined in this module
;-------------------------------------------------------------------------------------------------
  .def _portRESTORE_FIRST_CONTEXT
  .def _portTICK_ISR
  .if .TMS320C2800_FPU32 = 1
  .def _getSTF
  .endif

;-------------------------------------------------------------------------------------------------
; external variables used
;-------------------------------------------------------------------------------------------------
  .ref _pxCurrentTCB
  .ref _ulCriticalVars
  .ref _xTaskIncrementTick
  .ref _vTaskSwitchContext

;-------------------------------------------------------------------------------------------------
; definitions
;-------------------------------------------------------------------------------------------------
; Timer 2 TCR register location
  .define portCPU_TIM2_BASE+4, portCPU_TIM2_TCR

; Stack locations used
  .define "10", SP_TOP_DELTA
  .define "12", RPC_DELTA

; Stack locations for registers XAR4 and RPC for first context restore
  .if .TMS320C2800_FPU32 = 1
  .define "30", XAR4_DELTA
  .else
  .define "10", XAR4_DELTA
  .endif

; Stack locations for register IER save and restore during context switching
  .if .TMS320C2800_FPU32 = 1
  .define "46", IER_SAVE_SPA_CLR
  .define "48", IER_SAVE_SPA_SET
  .define "42", IER_REST_SPA_CLR
  .define "44", IER_REST_SPA_SET
  .else
  .define "26", IER_SAVE_SPA_CLR
  .define "28", IER_SAVE_SPA_SET
  .define "22", IER_REST_SPA_CLR
  .define "24", IER_REST_SPA_SET
  .endif

;-------------------------------------------------------------------------------------------------
; void portRESTORE_FIRST_CONTEXT(void)
;
; This function is called to start task scheduling
; Restore first context is not critical, so I put it into .text section
;-------------------------------------------------------------------------------------------------
  .sect ".text"

  .asmfunc
_portRESTORE_FIRST_CONTEXT
  SETC    INTM, DBGM    ; disable global interrupts
; Restore stack pointer from new task control block.
  MOVL    XAR0, #_pxCurrentTCB
  MOVL    XAR0, *XAR0
  MOVL    XAR0, *XAR0
  MOV     @SP, AR0

; Restore XAR4 and RPC from saved task stack.
; and return to main task function.
; SP should be set to stack start plus 2 before LRETR.
  SUBB   SP, #XAR4_DELTA
  POP    XAR4
  SUBB   SP, #RPC_DELTA
  POP    RPC
  SUBB   SP, #SP_TOP_DELTA
  CLRC    INTM, DBGM    ; renable global interrupts
  LRETR
  .endasmfunc

;-------------------------------------------------------------------------------------------------
; void portTICK_ISR(void)
;
; Handles CPU Timer 2 ISR to perform context switch if portYIELD() is called
; and to handle timer tick increments.
;-------------------------------------------------------------------------------------------------
; select where to place context switching code (uncomment needed section)
  .sect ".text"
;  .sect   "ramfuncs"
;  .sect   ".TI.ramfunc"

  .asmfunc
_portTICK_ISR:
; Save context
  ASP
  PUSH    AR1H:AR0H
  PUSH    RPC
  PUSH    XT
  PUSH    XAR2
  PUSH    XAR3
  PUSH    XAR4
  PUSH    XAR5
  PUSH    XAR6
  PUSH    XAR7
  .if .TMS320C2800_FPU32 = 1
  PUSH    RB
  MOV32   *SP++, STF
  MOV32   *SP++, R0H
  MOV32   *SP++, R1H
  MOV32   *SP++, R2H
  MOV32   *SP++, R3H
  MOV32   *SP++, R4H
  MOV32   *SP++, R5H
  MOV32   *SP++, R6H
  MOV32   *SP++, R7H
  .endif
  PUSH    DP:ST1

AFTER_CONTEXT_SAVE:
; JF: this should work, but needs more testing, use it at your own risk
;  .if (portCRITICAL_INTERRUPT_MASK > 0)
;; Enable critical interrupt nesting
;  AND     IER, #portCRITICAL_INTERRUPT_MASK
;  CLRC    INTM
;  .endif

; Save critical section nesting counter using indirect addressing
  MOVL    XAR0, #_ulCriticalVars
  MOVL    ACC, *XAR0
  PUSH    ACC

; Save stack pointer in the task control block.
  MOVL    XAR0, #_pxCurrentTCB
  MOVL    XAR0, *XAR0
  MOV     AH, #0       ;set to 0 AH before move the new value of pxTopOfStack
  MOV     AL, @SP
  MOVL   *XAR0, ACC

; Increment tick counter if timer tick is executed.
; Don't increment if explicitly yielded.
  MOV     AL, #0x1               ; set AL = pdTRUE before tick gets executed, used by non preemptive kernel
  MOVL    XAR0, #portCPU_TIM2_TCR
  TBIT   *XAR0, #15              ; if TIF bit is set it was a timer tick
  SB      CHECK_PREEMPTION, NTC  ; do not increment tick if it was yield
  TSET   *XAR0, #15              ; clear TIF bit by writing 1 to it
  LCR     _xTaskIncrementTick

CHECK_PREEMPTION:
; AL holds the return value of _xTaskIncrementTick, which tells if we have to change context
; Do context switch if _xTaskIncrementTick==pdTRUE (AL content) or portPREEMPTION_VALUE=0x0001
  MOV     AH, #portUSE_PREEMPTION
  CMPB    AH, #0x1
  SB      CONTEXT_SWITCH, EQ
  CMPB    AL, #0x1                 ;  _xTaskIncrementTick result was pdTRUE?
  SB      SKIP_CONTEXT_SWITCH, NEQ

CONTEXT_SWITCH:
  LCR     _vTaskSwitchContext

SKIP_CONTEXT_SWITCH:
; Restore stack pointer from new task control block.
  MOVL    XAR0, #_pxCurrentTCB
  MOVL    XAR0, *XAR0
  MOVL    ACC, *XAR0
  MOV     @SP, AL

; Restore critical section nesting counter
  MOVL    XAR0, #_ulCriticalVars
  POP     ACC
  MOVL   *XAR0, ACC

; JF: this should work, but needs more testing, use it at your own risk
;  .if (portCRITICAL_INTERRUPT_MASK > 0)
;; Disable interrupt nesting
;  SETC    INTM
;  .endif

RESTORE_CONTEXT:
  POP     DP:ST1
  .if .TMS320C2800_FPU32 = 1
  MOV32   R7H, *--SP
  MOV32   R6H, *--SP
  MOV32   R5H, *--SP
  MOV32   R4H, *--SP
  MOV32   R3H, *--SP
  MOV32   R2H, *--SP
  MOV32   R1H, *--SP
  MOV32   R0H, *--SP
  MOV32   STF, *--SP
  POP     RB
  .endif
  POP     XAR7
  POP     XAR6
  POP     XAR5
  POP     XAR4
  POP     XAR3
  POP     XAR2
  POP     XT
  POP     RPC
  POP     AR1H:AR0H
  NASP
  IRET
  .endasmfunc

;-------------------------------------------------------------------------------------------------
; uint32_t getSTF(void);
;
; returns: the contents of STF register
;-------------------------------------------------------------------------------------------------
  .if .TMS320C2800_FPU32 = 1
  .sect   ".text"

  .asmfunc
_getSTF
  MOV32   *SP++, STF
  POP     ACC
  LRETR
  .endasmfunc

  .endif
