

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_twi_mngr.h"
#include "nrfx_gpiote.h"

#include "buckler.h"
#include "virtual_timer.h"

// from driving

#include "nrf_drv_spi.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "controller.h"
//#include "threading.h"

#include "simple_ble.h"

//BLUETOOTH
#define ID 0x0045

static int32_t offset = 0;
static uint8_t latency = 0;
static uint32_t ble_times[2] = {0, 0};
static uint32_t beacon_times[2] = {0, 0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = ID, // TODO: replace with your lab bench number
        .adv_name          = "EE149 BUCKLER", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = BLE_GAP_CP_MIN_CONN_INTVL_MIN,
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t buckler_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t data_char = {.uuid16 = 0x108a};
static uint32_t data[4] = {0, 0, 0, 0}; // clock, encoder, ultrasonic, checkpoint
static simple_ble_char_t instruction_char = {.uuid16 = 0x108b};
static uint8_t instruction[7] = {0, 0, 0, 0, 0, 0, 0}; // lead toggle, speed, follow_distance, clock

simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &instruction_char)) {
      uint32_t time = read_timer();
      data[0] = time;
      printf("Got write to characteristic!\n");
      printf("Data: %d, %d, %d \n", data[0], data[1], data[2]);
      printf("Instruction: %d, %d, %d \n", instruction[0], instruction[1], instruction[2]);


      // CRUDE PTP: ASSUME COMPUTATION TO BE INSTANTANEOUS

      beacon_times[0] = beacon_times[1];
      beacon_times[1] = *((uint32_t *)(instruction[2]));

      if (beacon_times[0] != 0 && beacon_times[1] > beacon_times[0]) {
        latency = (beacon_times[1] - beacon_times[0])/2;
        offset = time - beacon_times[1] - latency;
      }

      printf("Current Time: %d, Offset: %d, Latency: %d \n", time, offset, latency);

      // char buf[16];
      // snprintf(buf, 16, "OFF: %d", offset);
      // display_write(buf, DISPLAY_LINE_1);
    }
}

void ble_setup() {

  // char buf[16];
  // snprintf(buf, 16, "ID: %x", ID);
  // display_write(buf, DISPLAY_LINE_0);


  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&buckler_service);

  simple_ble_add_characteristic(1, 0, 0, 0,
      sizeof(data), (uint8_t*)&data,
      &buckler_service, &data_char);

  simple_ble_add_characteristic(0, 1, 0, 0,
      sizeof(instruction), (uint8_t*)&instruction,
      &buckler_service, &instruction_char);
  // Start Advertising
  simple_ble_adv_only_name();
}

// END BLUETOOTH CONSTANT define

#define ULTRASONIC_MEASUREMENT_WINDOW 5
#define PLATOON_DIST 50

static nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

uint32_t rising_time = 0;
uint32_t ultrasonic_measurement_counter = 0;

// begin of time interval at which echo starts
robot_state_t state = OFF;
nrfx_gpiote_pin_t sensor1_pin = 4;

uint32_t* dist_mm_ptr;

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
    printf("rise on pin %ld, t = %d\n", pin, curr_time);
  }
  else if (curr_pin_state == 0) {
    uint32_t dist = (curr_time - rising_time)*(10/2) / 29;

    dist_mm_ptr[ultrasonic_measurement_counter] = dist;
    ultrasonic_measurement_counter = (ultrasonic_measurement_counter + 1) % ULTRASONIC_MEASUREMENT_WINDOW;
    printf("fall on pin %ld, t = %d, dist ~ %d mm\n", pin, curr_time, dist);
  }

  return;
}

void initializer() {
  ret_code_t error_code = NRF_SUCCESS;
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");
  // initialize display

  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");
}

int init_gpiote() {
  dist_mm_ptr = malloc(ULTRASONIC_MEASUREMENT_WINDOW * sizeof(uint32_t));
  for (uint32_t i = 0; i < ULTRASONIC_MEASUREMENT_WINDOW; i++) {
    dist_mm_ptr[i] = PLATOON_DIST;
  }
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
  return 0;
}

void LED_init() {
  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);
}



int main(void) {
  initializer();
  init_gpiote();
  virtual_timer_init();
  nrf_delay_ms(1000);
  virtual_timer_start_repeated(100000, grove_holler);
  kobukiInit();
  ble_setup();
  printf("Kobuki initialized!\n");

  // loop forever

  robot_state_t state = OFF;
  while (1) {
  //   uint32_t dist = duration();
  //   printf("%d, %d\n", iter++, dist);
    // printf("%d\n", read_timer());
    nrf_delay_ms(1);
    // power_manage();
    state = controller(state, dist_mm_ptr, ULTRASONIC_MEASUREMENT_WINDOW, data, instruction);
  }

}
