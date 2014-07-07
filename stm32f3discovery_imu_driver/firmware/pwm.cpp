#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "pwm.h"

static const uint8_t prescale = 15;
static_assert(prescale > 0, "prescale divide by zero");

static const double cpu_frequency = 48e6; // MHz
static const double timer_frequency = cpu_frequency / prescale;

static const double pwm_frequency = 50;
static const double period = 1 / pwm_frequency;
static const uint32_t period_ticks = timer_frequency * period + 0.5; 
static_assert(period_ticks <= 65535, "period_ticks too large for counter!");


void pwm_setup() {
  // channel 0 - A8 - TIM1_CH1
  // channel 1 - A9 - TIM1_CH2
  
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
  gpio_set_af(GPIOA, GPIO_AF6, GPIO8 | GPIO9);
  
  rcc_periph_clock_enable(RCC_TIM1);
  timer_reset(TIM1);
  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(TIM1, period_ticks);
  timer_set_prescaler(TIM1, prescale - 1);
  
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(TIM1, TIM_OC1);
  timer_enable_oc_output(TIM1, TIM_OC1);
  pwm_set_length(0, 0);
  
  timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(TIM1, TIM_OC2);
  timer_enable_oc_output(TIM1, TIM_OC2);
  pwm_set_length(1, 0);
  
  timer_enable_break_main_output(TIM1);
  timer_enable_counter(TIM1);
}

void pwm_set_length(uint8_t channel, float length_sec) {
  //assert(channel < 2);
  
  if(length_sec < 0 || length_sec > period) return;
  
  timer_set_oc_value(TIM1, channel == 0 ? TIM_OC1 : TIM_OC2, length_sec * timer_frequency + .5);
}
