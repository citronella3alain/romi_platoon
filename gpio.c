#include "gpio.h"

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)

gpio* gpio_struct = 0x50000000 + 0x504;
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
  if (dir == INPUT) {
    gpio_struct->pin_cnfs[gpio_num] &= ~(1UL << 0);
    gpio_struct->pin_cnfs[gpio_num] &= ~(1UL << 1);
  }
  else {
    gpio_struct->pin_cnfs[gpio_num] |= (1UL << 1);
    gpio_struct->pin_cnfs[gpio_num] |= (1UL << 0);
  }
  // gpio_struct->pin_cnfs[28] &= ~(1UL << 0);
}

// Set gpio_num high
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
  gpio_struct->out |= (1UL << gpio_num);
}

// Set gpio_num low
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
    gpio_struct->out &= ~(1UL << gpio_num);
}

// Inputs:
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    return ((gpio_struct->in) & (1 << gpio_num)) >> gpio_num;

}
