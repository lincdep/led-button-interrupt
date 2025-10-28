# led-button-interrupt
Lights up and led on the STM-Discovery-F3 using an interrupt. 
This code uses no HAL, no BSP and only uses a PAC to define the interrupt handler.
This fuctionaly defines a pac for all registers that were needed to enable gpio ports, pin configuration, and interrupts.
