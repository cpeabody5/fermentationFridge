#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>  // for timer_* functions
#include <libopencm3/cm3/nvic.h>     // for nvic_enable_irq
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>

volatile uint32_t system_millis = 0;



void timer_setup(void){
    
    rcc_periph_clock_enable(RCC_TIM2);
    timer_disable_counter(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 7199);    //10kHz tick
    timer_set_period(TIM2, 999);         //
    timer_clear_flag(TIM2, TIM_SR_UIF);
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    timer_enable_counter(TIM2);

}


void tim2_isr(void) __attribute__((interrupt("IRQ")));
void tim2_isr(void){
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        gpio_toggle(GPIOC, GPIO13);
    }

    //if (timer_get_flag(TIM2, TIM_SR_UIF)){
        //timer_clear_flag(TIM2, TIM_SR_UIF);
        //gpio_toggle(GPIOC, GPIO13);
    //}
}

void adc_setup(void){
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, 0, ADC_SMPR_SMP_239DOT5CYC);
    adc_power_on(ADC1);

    adc_reset_calibration(ADC1);
    while (adc_is_calibrating(ADC1));
    adc_calibrate(ADC1);
    while (adc_is_calibrating(ADC1));
}

void sys_tick_handler(void) __attribute__((interrupt("IRQ")));
void sys_tick_handler(void) {
    system_millis++;
}

void msleep(uint32_t delay) {
    uint32_t wake = system_millis + delay;
    while (system_millis < wake);
}

void systick_setup(void) {
    systick_set_reload(72000 - 1); // 72 MHz / 1000 = 1ms tick
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

uint16_t read_adc(void){
    adc_set_regular_sequence(ADC1, 1, (uint8_t[]){0});
    adc_start_conversion_direct(ADC1);
    while( !adc_eoc(ADC1));
    return adc_read_regular(ADC1);

}

//int filterTemp(int32_t newTemp, int32_t data[], int32_t size, int32_t* i){
//
//    data[*i] = newTemp;
//    ++*i;
//    if (*i >= size){
//        *i %= 20;
//    }
//    int32_t filteredTemp = 0;
//    for (int j = 0; j < size; j++){
//        filteredTemp += data[j];
//    }
//
//    filteredTemp /= size;
//
//    return filteredTemp;
//
//}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    //rcc_clock_setup_in_hsi_out_24mhz();
    //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_24MHZ]);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);

    cm_enable_interrupts();
    timer_setup();
    adc_setup();
    systick_setup();

    int32_t filteredTemp = 0;

    while(1){
        int16_t raw = read_adc();
        int32_t temp_c = ((raw * 3300 / 4095) - 500) / 10; 

        filteredTemp = (filteredTemp + temp_c) / 2;

        gpio_clear(GPIOB, GPIO0);
        gpio_clear(GPIOB, GPIO1);
        gpio_clear(GPIOB, GPIO10);
        gpio_clear(GPIOB, GPIO11);

        if (filteredTemp <= 20){
            gpio_set(GPIOB, GPIO0);
        }
        if (filteredTemp > 20 && filteredTemp <= 30){
            gpio_set(GPIOB, GPIO1);
        }
        if (filteredTemp > 30){
            gpio_set(GPIOB, GPIO10);
            gpio_set(GPIOB, GPIO11);
        }
        msleep(50);
    }
}