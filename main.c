// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "software_interrupt.h"
#include "gpio.h"

#include "buckler.h"
#include "virtual_timer.h"

// void SWI1_EGU1_IRQHandler(void) {
//     NRF_EGU1->EVENTS_TRIGGERED[0] = 0;
//     for (int i = 0; i < 50; i++) {
//       printf("software interrupt #%d\n", i);
//       nrf_delay_ms(100);
//     }
// }

// void GPIOTE_IRQHandler(void) {
//     NRF_GPIOTE->EVENTS_IN[0] = 0;
//     for (int i = 0; i < 100; i++){
//       //gpio_set(23);
//       //nrf_delay_ms(500);
//       //gpio_clear(23);
//       //nrf_delay_ms(500);
//       printf("BUTTON INTERRUPT! ending in %d\n", 99 - i);
//     }
// }

// void GPIOTE_HiToLoButtonEvent(void){
//   uint32_t config0 = (1UL << 0) | (1UL << 10) | (1UL << 11) | (1UL << 12) | (1UL << 17) | (1UL << 20) ;
//   uint32_t intenset = (1UL << 0);
//   NRF_GPIOTE->CONFIG[0] = config0;
//   NRF_GPIOTE->INTENSET = intenset;
//   NVIC_EnableIRQ(GPIOTE_IRQn);
//   NVIC_SetPriority(GPIOTE_IRQn, 0);
// }

static uint32_t duration() {
  gpio_config(4, OUTPUT);
  gpio_clear(4);
  nrf_delay_us(2);
  gpio_set(4);
  nrf_delay_us(15);
  gpio_clear(4);
  gpio_config(4, INPUT);
  
  uint32_t timeout = 1000000L;
  uint32_t begin = read_timer();
  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return 0; }
  while (!gpio_read(4)) if (read_timer() - begin >= timeout) { return 0; }
  uint32_t pulseBegin = read_timer();

  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return 0; }
  uint32_t pulseEnd = read_timer();
  return (pulseEnd - pulseBegin)*(10/2) / 29;  
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  virtual_timer_init();

  // gpio_config(23, OUTPUT);
  // gpio_config(28, INPUT);

  // GPIOTE_HiToLoButtonEvent();

  // software_interrupt_init();
  // NVIC_SetPriority(SWI1_EGU1_IRQn, 5);
  // printf("wtf");
  int iter = 0;
  // loop forever
  while (1) {
    uint32_t dist = duration();
    printf("%d: %d\n", iter++, dist);
    nrf_delay_ms(1000);
  }
}
