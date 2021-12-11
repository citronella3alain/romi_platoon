

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
#include "nrfx_gpiote.h"

#include "buckler.h"
#include "virtual_timer.h"

uint32_t rising_time = 0; 
// begin of time interval at which echo starts

nrfx_gpiote_pin_t sensor1_pin = 4;

void GPIOTE_Ultrasonic_ReceiveEdgeEvent(void) {
  // Set up "listeners" 
  // Grove A0 corresponds to P0.04
  // uint32_t loToHiConfig = (1UL << 0) | (1UL << 10) | (1UL << 16); // LoToHi
  // uint32_t hiToLoConfig = (1UL << 0) | (1UL << 10) | (1UL << 17); // HiToLo
  // uint32_t intenset_both = (1UL << 0) | (1UL << 1); // enable interrupts for events `IN[0]` and `IN[1]`

  uint32_t toggleConfig = (1UL << 0) | (1UL << 10) | (1UL << 16) | (1UL << 17); // toggle
  uint32_t intenset_toggle = (1UL << 0); // enable interrupt for event `IN[0]`

  // NRF_GPIOTE->CONFIG[0] = loToHiConfig;
  // NRF_GPIOTE->CONFIG[1] = hiToLoConfig;
  // NRF_GPIOTE->INTENSET = intenset_both;
  NRF_GPIOTE->CONFIG[0] = toggleConfig;
  NRF_GPIOTE->INTENSET = intenset_toggle;
  NVIC_EnableIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 1);
}


static void grove_holler (void) {
  // printf("hollering\n");
  NRF_GPIOTE->CONFIG[0] = 0;
  // NRF_GPIOTE->CONFIG[1] = 0;

  nrf_gpio_cfg_output(4);
  nrf_gpio_pin_clear(4);
  nrf_delay_us(2);
  nrf_gpio_pin_set(4);
  nrf_delay_us(15);
  nrf_gpio_pin_clear(4);
  nrf_delay_us(2);
  nrf_gpio_cfg_input(4, NRF_GPIO_PIN_NOPULL);
  GPIOTE_Ultrasonic_ReceiveEdgeEvent(); // this works but not 100% sure why 
  printf("hollered\n");
}


void handle_received_echo_toggle (nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  uint32_t curr_time = read_timer();
  uint32_t curr_pin_state = nrf_gpio_pin_read(pin);
  if (curr_pin_state == 1) {
    rising_time = curr_time;
    printf("rise on pin %d, t = %d\n", pin, curr_time);
  }
  else if (curr_pin_state == 0) {
    uint32_t dist = (curr_time - rising_time)*(10/2) / 29;
    printf("fall on pin %d, t = %d, dist ~ %d mm\n", pin, curr_time, dist);
  }
  // printf("toggle: pin: %d, state: %d\n", pin, curr_pin_state);
  // uint32_t time = read_timer();
  // printf("time: %d\n", time);
  return;
}

// void duration() {
//   grove_holler();
  
//   uint32_t timeout = 1000000L;
//   uint32_t begin = read_timer();
//   while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
//   while (!gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
//   uint32_t pulseBegin = read_timer();

//   while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
//   uint32_t pulseEnd = read_timer();
//   uint32_t dist = (pulseEnd - pulseBegin)*(10/2) / 29;
//   printf("%d\n", dist);
//   // return dist;  
// }

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize GPIOTE library
  if (!(nrfx_gpiote_is_init())) {
    nrfx_err_t err = nrfx_gpiote_init();
    if (err == NRFX_ERROR_INVALID_STATE) {
      printf("Could not initialize GPIOTE module. Exiting...\n");
      return 1;
    }
  }
  
  // init GPIOTE input pin
  nrfx_gpiote_in_config_t* toggle_config = (nrfx_gpiote_in_config_t*) malloc(sizeof(nrfx_gpiote_in_config_t));
  toggle_config->sense = NRF_GPIOTE_POLARITY_TOGGLE;
  toggle_config->pull = GPIO_PIN_CNF_PULL_Disabled;
  toggle_config->is_watcher = false; // we are not tracking an output
  toggle_config->hi_accuracy = true; // high accuracy (IN_EVENT) used
  toggle_config->skip_gpio_setup = true; // do not change gpio configuration

  nrfx_err_t init_err = nrfx_gpiote_in_init(sensor1_pin, toggle_config, handle_received_echo_toggle);

  // printf("nrfx gpiote input initialized!\n");
  virtual_timer_init();
  nrf_delay_ms(3000);

  // measurement rate of ~10 hz 
  virtual_timer_start_repeated(100000, grove_holler); 
  // virtual_timer_start_repeated(1000000, duration);
  

  int iter = 0;
  // loop forever
  while (1) {
  //   uint32_t dist = duration();
  //   printf("%d, %d\n", iter++, dist);
    // printf("%d\n", read_timer());
    nrf_delay_ms(2000);
  }
  
}
