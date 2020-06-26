# freertos_c28x
FreeRTOS port for TI C2000's C28x based microcontrollers TMS320 family.

Compiled in CCS v6.1.2. FreeRTOS v9.0.0.
Developed and tested on following MCUs:
 * TMS320F28377S
 * TMS320F28034
 * TMS320F28069

Should work with any TMS320 MCU and any futher version of FreeRTOS.

This project contains FreeRTOS, port and simple examples for TMS320F28377S and TMS320F28034.
Examples needs TI's controlSUITE to be installed for both MCU and project settings
should be adjusted to let compiler find device support files.

This project is distributed under MIT open source license.

## Kernel unaware interrupts
To help the implementation of fast control loops in C2000 devices, the possibility of enabling kernel unaware interrupts during kernel critical sections has been added to this port. These time critical interrupts can also be nested inside kernel aware interrupts (interrupts where FreeRTOS `*_FROM_ISR()` API is used). This was done to allow time critical interrupts to be serviced first, delaying execution of kernel services during idle time. This feature is specially attractive when implementing control loops using the C28x core.

When setting `portCRITICAL_INTERRUPT_MASK` constant to a value greater than `0x0000`, interrupts are enabled/disabled using `IER` register instead of using `INTM` bit in `ST1` register (default behavior). The value in `portCRITICAL_INTERRUPT_MASK` is used as a bitmask for `IER`, enable/disable is performed using this mask and the value of `IER` before disabling kernel aware interrupts, which is stored in each task stack. Global interrupts are disabled for short periods of time only when updating `IER` value. As C28x core implements interrupt priorities in hardware, control related peripherals such as ADC's, PWM's, ECAP's, EQEP's are assigned to INT1 to INT5 at CPU level. In general, control applications will use mask values that enable all or some of the five lowest bits of `IER`.

As an example, consider an aplication that uses only ECAP1 to generate a periodic interrupt at 50us, setting `portCRITICAL_INTERRUPT_MASK` to `0x0008` will make FreeRTOS to treat all Group 4 as kernel unaware interrupts. For enabling nesting during kernel aware interrupts, static inline functions `portPROLOG_ISR()` and `portEPILOG_ISR()` are provided in `portmacro.h`. Kernel unaware interrupts are implemented as usually done in C28x, for instance:

```
interrupt void ecap1_ISR( void )
{
  // Simulate interrupt work
  // proceesing must allow idle time to run kernel services
  DELAY_US(40);

  // clear interrupt flags
  ECap1Regs.ECCLR.bit.CTR_EQ_PRD = 1U;
  ECap1Regs.ECCLR.bit.INT = 1U;

  // Acknowledge group 4 interrupt
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
```
and kernel aware interrupts allowing nesting of kernel unaware interrupts should be implemented as follows:
```
interrupt void timer0_ISR( void )
{
  // kernel aware interrupts should use this function
  portPROLOG_ISR();

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  // Simulate work done in ISR
  DELAY_US(750);

  // Acknowledge group 1 interrupts
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // kernel aware interrupts should use this function
  portEPILOG_ISR();
}
```
Please note that kernel unaware interrupts are not serviced during context switch, i.e. interrupt nesting is not enabled inside `portTICK_ISR()`. Service of high priority interrupts is delayed for some time during execution of context switch and/or tick function. Application designers must also provide idle time outside critical interrupts for the execution of kernel services.

Also note that interrupt stack is shared with currently running task stack. This means that the user must provide each task a stack memory buffer big enough to consider worst case stack usage to *every* task, i.e. at least one kernel aware interrupt (the one that enables nesting) and all possible kernel unaware interrupts that are nested in the application. So far, this port does not support separate task and interrupt stack memory buffers.
