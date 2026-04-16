/* Copyright 2024. All Rights Reserved. */
/* GPIO Implementation using sysfs with config file support */

#include "gpio.h"
#include "register_map.h"
#include "gpio_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

// ========================================================================
// Runtime GPIO Configuration
// ========================================================================

static gpio_config_t g_gpio_config;
static int g_gpio_config_loaded = 0;

// ========================================================================
// Internal state
// ========================================================================

static int g_gpio_initialized = 0;
static int g_di_fd[GPIO_DI_MAX_CHANNELS];
static int g_do_fd[GPIO_DO_MAX_CHANNELS];

// ========================================================================
// Helper functions
// ========================================================================

static int write_file(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -1;
    }
    if (write(fd, value, strlen(value)) < 0) {
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

static int gpio_export(int gpio_num) {
    char buf[128];
    char path[64];

    // Check if already exported
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", gpio_num);
    struct stat st;
    if (stat(path, &st) == 0) {
        return 0;  // Already exported
    }

    // Export GPIO
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        return -1;
    }
    snprintf(buf, sizeof(buf), "%d", gpio_num);
    if (write(fd, buf, strlen(buf)) < 0) {
        close(fd);
        return -1;
    }
    close(fd);

    // Wait for GPIO to be created
    usleep(100000);  // 100ms

    return 0;
}

static int gpio_unexport(int gpio_num) {
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        return -1;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", gpio_num);
    write(fd, buf, strlen(buf));
    close(fd);
    return 0;
}

static int gpio_set_direction(int gpio_num, const char *direction) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    return write_file(path, direction);
}

static int gpio_set_value(int gpio_num, int value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);
    return write_file(path, value ? "1" : "0");
}

static int gpio_get_value(int gpio_num) {
    char path[64];
    char buf[8];

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return -1;
    }

    memset(buf, 0, sizeof(buf));
    if (read(fd, buf, sizeof(buf) - 1) < 0) {
        close(fd);
        return -1;
    }
    close(fd);

    return (buf[0] == '1') ? 1 : 0;
}

// ========================================================================
// Public API
// ========================================================================

int gpio_init(void) {
    return gpio_init_with_config(NULL);
}

int gpio_init_with_config(const char *config_file) {
    if (g_gpio_initialized) {
        return 0;
    }

    // Load GPIO configuration
    if (!g_gpio_config_loaded) {
        if (config_file) {
            gpio_config_load(config_file, &g_gpio_config);
        } else {
            gpio_config_load_default(&g_gpio_config);
        }
        g_gpio_config_loaded = 1;
    }

    printf("[GPIO] Initializing GPIOs from %s...\n",
           config_file ? config_file : "default config");

    // Initialize fd arrays
    for (int i = 0; i < GPIO_DI_MAX_CHANNELS; i++) {
        g_di_fd[i] = -1;
    }
    for (int i = 0; i < GPIO_DO_MAX_CHANNELS; i++) {
        g_do_fd[i] = -1;
    }

    // Export and configure DI pins
    int di_count = g_gpio_config.di_count;
    for (int i = 0; i < di_count; i++) {
        int gpio = g_gpio_config.di_channels[i].gpio_num;

        if (gpio_export(gpio) < 0) {
            printf("[GPIO] Failed to export DI pin %d (%s)\n",
                   gpio, g_gpio_config.di_channels[i].name);
            continue;
        }

        if (gpio_set_direction(gpio, "in") < 0) {
            printf("[GPIO] Failed to set direction for DI pin %d (%s)\n",
                   gpio, g_gpio_config.di_channels[i].name);
            continue;
        }

        // Open value file for reading
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
        g_di_fd[i] = open(path, O_RDONLY);
        if (g_di_fd[i] < 0) {
            printf("[GPIO] Failed to open DI pin %d value\n", gpio);
        }
    }
    printf("[GPIO] DI channels: %d\n", di_count);

    // Export and configure DO pins
    int do_count = g_gpio_config.do_count;
    for (int i = 0; i < do_count; i++) {
        int gpio = g_gpio_config.do_channels[i].gpio_num;

        if (gpio_export(gpio) < 0) {
            printf("[GPIO] Failed to export DO pin %d (%s)\n",
                   gpio, g_gpio_config.do_channels[i].name);
            continue;
        }

        if (gpio_set_direction(gpio, "out") < 0) {
            printf("[GPIO] Failed to set direction for DO pin %d (%s)\n",
                   gpio, g_gpio_config.do_channels[i].name);
            continue;
        }

        // Set default value to 0
        gpio_set_value(gpio, 0);
    }
    printf("[GPIO] DO channels: %d\n", do_count);

    g_gpio_initialized = 1;
    printf("[GPIO] Initialization complete\n");
    return 0;
}

void gpio_cleanup(void) {
    int di_count = g_gpio_config.di_count;
    int do_count = g_gpio_config.do_count;

    for (int i = 0; i < di_count; i++) {
        if (g_di_fd[i] >= 0) {
            close(g_di_fd[i]);
            g_di_fd[i] = -1;
        }
        gpio_unexport(g_gpio_config.di_channels[i].gpio_num);
    }

    for (int i = 0; i < do_count; i++) {
        gpio_unexport(g_gpio_config.do_channels[i].gpio_num);
    }

    g_gpio_initialized = 0;
}

int gpio_read_di(uint8_t channel) {
    if (channel >= g_gpio_config.di_count) {
        return -1;
    }

    int gpio = g_gpio_config.di_channels[channel].gpio_num;
    return gpio_get_value(gpio);
}

int gpio_write_do(uint8_t channel, bool value) {
    if (channel >= g_gpio_config.do_count) {
        return -1;
    }

    int gpio = g_gpio_config.do_channels[channel].gpio_num;
    return gpio_set_value(gpio, value ? 1 : 0);
}

uint32_t gpio_read_all_di(void) {
    uint32_t status = 0;
    int di_count = g_gpio_config.di_count;

    for (int i = 0; i < di_count; i++) {
        int val = gpio_read_di(i);
        if (val > 0) {
            status |= (1 << i);
        }
    }

    return status;
}

int gpio_write_do_by_mask(uint16_t mask) {
    int do_count = g_gpio_config.do_count;

    for (int i = 0; i < do_count; i++) {
        bool val = (mask >> i) & 1;
        gpio_write_do(i, val);
    }
    return 0;
}

void gpio_print_config(void) {
    gpio_config_print(&g_gpio_config);
}

const gpio_config_t* gpio_get_config(void) {
    return &g_gpio_config;
}
