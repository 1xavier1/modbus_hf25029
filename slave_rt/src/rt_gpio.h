/* Real-time GPIO Header */

#ifndef RT_GPIO_H
#define RT_GPIO_H

#include <stdint.h>
#include <stdbool.h>

// GPIO base address for TLT153 (example - verify with hardware manual)
#define GPIO_BASE_ADDR    0x0300B000
#define GPIO_PORT_SIZE    0x1000

// Initialize GPIO with memory-mapped access
int rt_gpio_init(void);

// Cleanup GPIO
void rt_gpio_cleanup(void);

// Read GPIO port data register (fast mmap access)
uint32_t rt_gpio_read_port(int port);

// Write GPIO port data register
void rt_gpio_write_port(int port, uint32_t value);

// Set GPIO pin high
void rt_gpio_set(int port, int pin);

// Set GPIO pin low
void rt_gpio_clear(int port, int pin);

// Toggle GPIO pin
void rt_gpio_toggle(int port, int pin);

// Read GPIO pin
int rt_gpio_get(int port, int pin);

// Read all 22 DI channels as bitmask
uint32_t rt_gpio_read_all_di(void);

// Write all 12 DO channels from bitmask
void rt_gpio_write_all_do(uint32_t mask);

#endif /* RT_GPIO_H */
