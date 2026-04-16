/* Copyright 2024. All Rights Reserved. */
/* GPIO Implementation using sysfs */

#include "gpio.h"
#include "register_map.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

// ========================================================================
// GPIO Configuration - Flexible mapping
// ========================================================================

// DI GPIO pins (22 channels)
// These can be modified to match actual hardware wiring
const gpio_di_config_t g_di_gpio_config[DI_GPIO_COUNT] = {
    {34,  "PB2",  "DI_CH01"},   // J11-3
    {129, "PE1",  "DI_CH02"},   // J11-4
    {130, "PE2",  "DI_CH03"},   // J11-5
    {131, "PE3",  "DI_CH04"},   // J11-6
    {132, "PE4",  "DI_CH05"},   // J11-7
    {297, "PJ9",  "DI_CH06"},   // J31-1
    {326, "PK6",  "DI_CH07"},   // J31-2
    // Continue for remaining DI channels if needed
    {35,  "PB3",  "DI_CH08"},   // Placeholder
    {136, "PE8",  "DI_CH09"},   // Placeholder
    {137, "PE9",  "DI_CH10"},   // Placeholder
    {138, "PE10", "DI_CH11"},   // Placeholder
    {139, "PE11", "DI_CH12"},   // Placeholder
    {140, "PE12", "DI_CH13"},   // Placeholder
    {141, "PE13", "DI_CH14"},   // Placeholder
    {142, "PE14", "DI_CH15"},   // Placeholder
    {143, "PE15", "DI_CH16"},   // Placeholder
    {144, "PF0",  "DI_CH17"},   // Placeholder
    {145, "PF1",  "DI_CH18"},   // Placeholder
    {146, "PF2",  "DI_CH19"},   // Placeholder
    {147, "PF3",  "DI_CH20"},   // Placeholder
    {148, "PF4",  "DI_CH21"},   // Placeholder
    {149, "PF5",  "DI_CH22"},   // Placeholder
};

// DO GPIO pins (12 channels)
const gpio_do_config_t g_do_gpio_config[DO_GPIO_COUNT] = {
    {35,  "PB3",  "DO_CH01"},   // Placeholder
    {36,  "PB4",  "DO_CH02"},   // Placeholder
    {37,  "PB5",  "DO_CH03"},   // Placeholder
    {38,  "PB6",  "DO_CH04"},   // Placeholder
    {39,  "PB7",  "DO_CH05"},   // Placeholder
    {40,  "PB8",  "DO_CH06"},   // Placeholder
    {41,  "PB9",  "DO_CH07"},   // Placeholder
    {42,  "PB10", "DO_CH08"},   // Placeholder
    {43,  "PB11", "DO_CH09"},   // Placeholder
    {44,  "PB12", "DO_CH10"},   // Placeholder
    {45,  "PB13", "DO_CH11"},   // Placeholder
    {46,  "PB14", "DO_CH12"},   // Placeholder
};

// ========================================================================
// Internal state
// ========================================================================

static int g_gpio_initialized = 0;
static int g_di_fd[DI_GPIO_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      -1, -1};
static int g_do_fd[DO_GPIO_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      -1, -1};

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
    if (g_gpio_initialized) {
        return 0;
    }

    printf("[GPIO] Initializing GPIOs...\n");

    // Export and configure DI pins
    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        int gpio = g_di_gpio_config[i].gpio_num;

        if (gpio_export(gpio) < 0) {
            printf("[GPIO] Failed to export DI pin %d (%s)\n",
                   gpio, g_di_gpio_config[i].name);
            continue;
        }

        if (gpio_set_direction(gpio, "in") < 0) {
            printf("[GPIO] Failed to set direction for DI pin %d (%s)\n",
                   gpio, g_di_gpio_config[i].name);
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

    // Export and configure DO pins
    for (int i = 0; i < DO_GPIO_COUNT; i++) {
        int gpio = g_do_gpio_config[i].gpio_num;

        if (gpio_export(gpio) < 0) {
            printf("[GPIO] Failed to export DO pin %d (%s)\n",
                   gpio, g_do_gpio_config[i].name);
            continue;
        }

        if (gpio_set_direction(gpio, "out") < 0) {
            printf("[GPIO] Failed to set direction for DO pin %d (%s)\n",
                   gpio, g_do_gpio_config[i].name);
            continue;
        }

        // Set default value to 0
        gpio_set_value(gpio, 0);
    }

    g_gpio_initialized = 1;
    printf("[GPIO] Initialization complete\n");
    return 0;
}

void gpio_cleanup(void) {
    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        if (g_di_fd[i] >= 0) {
            close(g_di_fd[i]);
            g_di_fd[i] = -1;
        }
        gpio_unexport(g_di_gpio_config[i].gpio_num);
    }

    for (int i = 0; i < DO_GPIO_COUNT; i++) {
        gpio_unexport(g_do_gpio_config[i].gpio_num);
    }

    g_gpio_initialized = 0;
}

int gpio_read_di(uint8_t channel) {
    if (channel >= DI_GPIO_COUNT) {
        return -1;
    }

    int gpio = g_di_gpio_config[channel].gpio_num;
    return gpio_get_value(gpio);
}

int gpio_write_do(uint8_t channel, bool value) {
    if (channel >= DO_GPIO_COUNT) {
        return -1;
    }

    int gpio = g_do_gpio_config[channel].gpio_num;
    return gpio_set_value(gpio, value ? 1 : 0);
}

uint32_t gpio_read_all_di(void) {
    uint32_t status = 0;

    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        int val = gpio_read_di(i);
        if (val > 0) {
            status |= (1 << i);
        }
    }

    return status;
}

int gpio_write_do_by_mask(uint16_t mask) {
    for (int i = 0; i < DO_GPIO_COUNT; i++) {
        bool val = (mask >> i) & 1;
        gpio_write_do(i, val);
    }
    return 0;
}

void gpio_print_config(void) {
    printf("\n========== GPIO Configuration ==========\n");
    printf("DI Channels (%d):\n", DI_GPIO_COUNT);
    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        printf("  CH%02d: GPIO %3d (%s)\n",
               i + 1, g_di_gpio_config[i].gpio_num, g_di_gpio_config[i].name);
    }

    printf("DO Channels (%d):\n", DO_GPIO_COUNT);
    for (int i = 0; i < DO_GPIO_COUNT; i++) {
        printf("  CH%02d: GPIO %3d (%s)\n",
               i + 1, g_do_gpio_config[i].gpio_num, g_do_gpio_config[i].name);
    }
    printf("==========================================\n\n");
}
