

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

void GPIOTE_Ultrasonic_ReceiveEdgeEvent(void) {
  // Set up "listeners" 
  // Grove A0 corresponds to P0.04
  uint32_t loToHiConfig = (1UL << 0) | (1UL << 10) | (1UL << 16); // LoToHi
  uint32_t hiToLoConfig = (1UL << 0) | (1UL << 10) | (1UL << 17); // HiToLo
  uint32_t intenset = (1UL << 0) | (1UL << 1); // enable interrupts for events `IN[0]` and `IN[1]`
  NRF_GPIOTE->CONFIG[0] = loToHiConfig;
  NRF_GPIOTE->CONFIG[1] = hiToLoConfig;
  NRF_GPIOTE->INTENSET = intenset;
  NVIC_EnableIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 1);
}

static void ultrasonic_holler (void) {
  gpio_config(4, OUTPUT);
  gpio_clear(4);
  nrf_delay_us(2);
  gpio_set(4);
  nrf_delay_us(15);
  gpio_clear(4);
  gpio_config(4, INPUT);
}

void GPIOTE_IRQHandler(void) {
    // handle echo returning
    if (NRF_GPIOTE->EVENTS_IN[0]) {
      // rising edge detected
      NRF_GPIOTE->EVENTS_IN[0] = 0;
      printf("rising edge detected\n");
    }
    else if (NRF_GPIOTE->EVENTS_IN[1]) {
      NRF_GPIOTE->EVENTS_IN[1] = 0;
      printf("falling edge detected\n");
    }
    else {
      printf("something weird here");
    }
}

void duration() {
  printf("entered duration\n");
  ultrasonic_holler();
  
  uint32_t timeout = 1000000L;
  uint32_t begin = read_timer();
  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  while (!gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  uint32_t pulseBegin = read_timer();

  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  uint32_t pulseEnd = read_timer();
  uint32_t dist = (pulseEnd - pulseBegin)*(10/2) / 29;
  printf("%d\n", dist);
  // return dist;  
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  virtual_timer_init();
  nrf_delay_ms(3000);
  // virtual_timer_start_repeated(1000000, ultrasonic_holler);
  virtual_timer_start_repeated(1000000, duration);
  // GPIOTE_Ultrasonic_ReceiveEdgeEvent();

  int iter = 0;
  // loop forever
  while (1) {
  //   uint32_t dist = duration();
  //   printf("%d, %d\n", iter++, dist);
  //   // ultrasonic_holler();
    // printf("%d\n", read_timer());
    nrf_delay_ms(2000);
  }
  
}
