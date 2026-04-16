/* GPIO Header */

#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "register_map.h"

// GPIO initialization
int gpio_init(void);
void gpio_cleanup(void);

// Read single DI channel
int gpio_read_di(uint8_t channel);

// Write single DO channel
int gpio_write_do(uint8_t channel, bool value);

// Read all DI channels as bitmask
uint32_t gpio_read_all_di(void);

// Write DO channels by mask
int gpio_write_do_by_mask(uint16_t mask);

// Print GPIO configuration
void gpio_print_config(void);

#endif /* GPIO_H */
