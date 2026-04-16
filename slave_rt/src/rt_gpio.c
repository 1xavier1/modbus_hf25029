/* Copyright 2024. All Rights Reserved. */
/* Real-time GPIO Implementation using mmap */

#include "rt_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

// ========================================================================
// Memory-mapped GPIO
// ========================================================================

static volatile uint32_t *gpio_map = NULL;
static int gpio_mem_fd = -1;
static int g_gpio_initialized = 0;

// GPIO register offsets (typical for SoC GPIO controllers)
#define GPIO_DATA_REG     0x00    // Data register
#define GPIO_DIR_REG      0x04    // Direction register
#define GPIO_SET_REG      0x08    // Set register (write 1 to set)
#define GPIO_CLR_REG      0x0C    // Clear register (write 1 to clear)

int rt_gpio_init(void) {
    if (g_gpio_initialized) {
        return 0;
    }

    printf("[RT-GPIO] Initializing with mmap...\n");

    // Open /dev/mem for direct memory access
    gpio_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (gpio_mem_fd < 0) {
        perror("[RT-GPIO] open /dev/mem failed");
        printf("[RT-GPIO] Falling back to sysfs mode\n");
        return -1;
    }

    // Memory map GPIO registers
    gpio_map = mmap(NULL,
                   GPIO_PORT_SIZE,
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   gpio_mem_fd,
                   GPIO_BASE_ADDR);

    if (gpio_map == MAP_FAILED) {
        perror("[RT-GPIO] mmap failed");
        close(gpio_mem_fd);
        gpio_mem_fd = -1;
        return -1;
    }

    // Set all GPIO pins to input mode initially
    // In real implementation, configure direction registers

    g_gpio_initialized = 1;
    printf("[RT-GPIO] Memory mapped at %p\n", (void*)gpio_map);

    return 0;
}

void rt_gpio_cleanup(void) {
    if (gpio_map != NULL && gpio_map != MAP_FAILED) {
        munmap((void*)gpio_map, GPIO_PORT_SIZE);
        gpio_map = NULL;
    }

    if (gpio_mem_fd >= 0) {
        close(gpio_mem_fd);
        gpio_mem_fd = -1;
    }

    g_gpio_initialized = 0;
}

// ========================================================================
// GPIO Operations
// ========================================================================

uint32_t rt_gpio_read_port(int port) {
    if (!g_gpio_initialized || gpio_map == NULL) {
        return 0;
    }

    // Memory barrier for ARM
    uint32_t val = gpio_map[port * 4 + (GPIO_DATA_REG / 4)];
    __sync_synchronize();
    return val;
}

void rt_gpio_write_port(int port, uint32_t value) {
    if (!g_gpio_initialized || gpio_map == NULL) {
        return;
    }

    __sync_synchronize();
    gpio_map[port * 4 + (GPIO_DATA_REG / 4)] = value;
}

void rt_gpio_set(int port, int pin) {
    if (!g_gpio_initialized || gpio_map == NULL) {
        return;
    }

    __sync_synchronize();
    gpio_map[port * 4 + (GPIO_SET_REG / 4)] = (1 << pin);
}

void rt_gpio_clear(int port, int pin) {
    if (!g_gpio_initialized || gpio_map == NULL) {
        return;
    }

    __sync_synchronize();
    gpio_map[port * 4 + (GPIO_CLR_REG / 4)] = (1 << pin);
}

void rt_gpio_toggle(int port, int pin) {
    uint32_t val = rt_gpio_read_port(port);
    if (val & (1 << pin)) {
        rt_gpio_clear(port, pin);
    } else {
        rt_gpio_set(port, pin);
    }
}

int rt_gpio_get(int port, int pin) {
    uint32_t val = rt_gpio_read_port(port);
    return (val >> pin) & 1;
}

uint32_t rt_gpio_read_all_di(void) {
    // In real implementation, read from actual GPIO pins
    // For now, this would be integrated with the data simulator
    return 0;
}

void rt_gpio_write_all_do(uint32_t mask) {
    // In real implementation, write to actual GPIO pins
    (void)mask;
}
