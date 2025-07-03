.syntax unified
.thumb


.section .vectors, "a"
.word _stack_top
.word Reset
.word NMI_Handler
.word HardFault_Handler
.word 0  /* MemManage */
.word 0  /* BusFault */
.word 0  /* UsageFault */
.word 0, 0, 0, 0  /* Reserved */
.word 0  /* SVCall */
.word 0  /* DebugMonitor */
.word 0  /* Reserved */
.word 0  /* PendSV */
.word sys_tick_handler  /* SysTick */
/* external interrupts (start at index 16) */
.word 0  /* WWDG */
.word 0  /* PVD */
.word 0  /* TAMPER */
.word 0  /* RTC */
.word 0  /* FLASH */
.word 0  /* RCC */
.word 0  /* EXTI0 */
.word 0  /* EXTI1 */
.word 0  /* EXTI2 */
.word 0  /* EXTI3 */
.word 0  /* EXTI4 */
.word 0  /* DMA1_Channel1 */
.word 0  /* DMA1_Channel2 */
.word 0  /* DMA1_Channel3 */
.word 0  /* DMA1_Channel4 */
.word 0  /* DMA1_Channel5 */
.word 0  /* DMA1_Channel6 */
.word 0  /* DMA1_Channel7 */
.word 0  /* ADC1_2 */
.word 0  /* USB_HP_CAN_TX */
.word 0  /* USB_LP_CAN_RX0 */
.word 0  /* CAN_RX1 */
.word 0  /* CAN_SCE */
.word 0  /* EXTI9_5 */
.word 0  /* TIM1_BRK */
.word 0  /* TIM1_UP */
.word 0  /* TIM1_TRG_COM */
.word 0  /* TIM1_CC */
.word tim2_isr  /* TIM2 (vector #28) */
.word 0  /* TIM3 */
.word 0  /* TIM4 */
.word 0  /* I2C1_EV */
.word 0  /* I2C1_ER */
.word 0  /* I2C2_EV */
.word 0  /* I2C2_ER */
.word 0  /* SPI1 */
.word 0  /* SPI2 */
.word 0  /* USART1 */
.word 0  /* USART2 */
.word 0  /* USART3 */
.word 0  /* EXTI15_10 */
.word 0  /* RTCAlarm */
.word 0  /* USBWakeup */


.section .text
.global Reset
.type Reset, %function
Reset:
    ldr r0, =_data_lma   // src: start of .data in flash
    ldr r1, =_data_vma   // dest: start of .data in RAM
    ldr r2, =_edata      // end of .data in RAM

copy_data:
    cmp r1, r2
    bcc 1f
    b zero_bss
1:
    ldr r3, [r0]
    adds r0, #4
    str r3, [r1]
    adds r1, #4
    b copy_data

zero_bss:
    ldr r0, =_bss_start
    ldr r1, =_bss_end

clear_loop:
    cmp r0, r1
    bcc 2f
    b call_main
2:
    movs r2, #0
    str r2, [r0]
    adds r0, #4
    b clear_loop

call_main:
    bl main
    b .


.globl NMI_Handler
NMI_Handler:
  b .

.globl HardFault_Handler
HardFault_Handler:
  b .

.globl tim2_isr
.weak tim2_isr
tim2_isr:
  bx lr

.extern _stack_top
