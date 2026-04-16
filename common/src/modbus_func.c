/* Copyright 2024. All Rights Reserved. */
/* Common Modbus Functions Implementation */

#include "modbus_func.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

// ========================================================================
// Version
// ========================================================================

// g_log_level is defined in main.c, extern here for use in common code
extern int g_log_level;

// ========================================================================
// Time Functions
// ========================================================================

void msleep(unsigned int ms) {
    usleep(ms * 1000);
}

uint64_t get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// ========================================================================
// Modbus CRC16 (RTU)
// ========================================================================

uint16_t modbus_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
