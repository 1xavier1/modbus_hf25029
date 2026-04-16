/* Copyright 2024. All Rights Reserved. */
/* GPIO Configuration Loader */

#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Max channels - must match register_map.h
#ifndef GPIO_DI_MAX_CHANNELS
#define GPIO_DI_MAX_CHANNELS  22
#endif

#ifndef GPIO_DO_MAX_CHANNELS
#define GPIO_DO_MAX_CHANNELS  12
#endif

// GPIO channel configuration
typedef struct {
    int gpio_num;         // Linux GPIO number
    const char *name;     // Pin name (e.g., "PB2")
    const char *label;    // Channel label (e.g., "DI_CH01")
} gpio_channel_config_t;

// Full GPIO configuration
typedef struct {
    int di_count;
    gpio_channel_config_t di_channels[GPIO_DI_MAX_CHANNELS];
    int do_count;
    gpio_channel_config_t do_channels[GPIO_DO_MAX_CHANNELS];
} gpio_config_t;

// Load configuration from JSON file
// Returns 0 on success, -1 on failure
int gpio_config_load(const char *filename, gpio_config_t *config);

// Load default configuration
void gpio_config_load_default(gpio_config_t *config);

// Print configuration
void gpio_config_print(const gpio_config_t *config);

#endif /* GPIO_CONFIG_H */
