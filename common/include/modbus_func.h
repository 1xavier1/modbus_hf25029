/* Copyright 2024. All Rights Reserved. */
/* Common definitions for Modbus Slave */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

// ========================================================================
// Version
// ========================================================================
#define VERSION                 "1.0"
#define PROGRAM_NAME            "HF25029-CDP Modbus Slave"

// ========================================================================
// Thread Control
// ========================================================================

// Global quit flag
extern volatile bool g_quit;

// Mutex for register data protection
extern pthread_mutex_t g_reg_mutex;

// Thread handles
extern pthread_t g_rtu_thread;
extern pthread_t g_tcp_thread;
extern pthread_t g_sync_thread;

// ========================================================================
// Modbus Function Codes
// ========================================================================
#define MODBUS_FC_READ_COILS            0x01
#define MODBUS_FC_READ_DISCRETE_INPUT   0x02
#define MODBUS_FC_READ_HOLDING_REG      0x03
#define MODBUS_FC_READ_INPUT_REG        0x04
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05
#define MODBUS_FC_WRITE_SINGLE_REG      0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS  0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10

// Exception codes
#define MODBUS_EX_ILLEGAL_FUNCTION      0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR    0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE    0x03
#define MODBUS_EX_SERVER_FAILURE        0x04

// ========================================================================
// Modbus TCP
// ========================================================================
#define MODBUS_TCP_PORT                 502
#define MODBUS_TCP_MAX_CONNECTIONS     5
#define MODBUS_TCP_BUFFER_SIZE         1024

// MBAP Header offsets
#define MBAP_TRANSACTION_ID   0
#define MBAP_PROTOCOL_ID     2
#define MBAP_LENGTH           4
#define MBAP_UNIT_ID          6

// ========================================================================
// Modbus RTU
// ========================================================================
#define MODBUS_RTU_DEFAULT_DEV         "/dev/ttyAS2"
#define MODBUS_RTU_DEFAULT_BAUD       115200
#define MODBUS_RTU_DEFAULT_BITS       8
#define MODBUS_RTU_DEFAULT_PARITY     'N'
#define MODBUS_RTU_DEFAULT_STOP       1
#define MODBUS_RTU_TIMEOUT_MS          100

// ========================================================================
// Device Configuration
// ========================================================================
#define DEFAULT_SLAVE_ADDR     1
#define DEFAULT_BAUDRATE      115200

// Baudrate codes
#define BAUD_1200     0
#define BAUD_2400     1
#define BAUD_4800     2
#define BAUD_9600     3
#define BAUD_19200    4
#define BAUD_38400    5
#define BAUD_115200   6

// Convert baudrate code to actual value
#define BAUD_CODE_TO_VALUE(code) \
    ((code) == BAUD_1200 ? 1200 : \
     (code) == BAUD_2400 ? 2400 : \
     (code) == BAUD_4800 ? 4800 : \
     (code) == BAUD_9600 ? 9600 : \
     (code) == BAUD_19200 ? 19200 : \
     (code) == BAUD_38400 ? 38400 : 115200)

// ========================================================================
// Utility Functions
// ========================================================================

// Calculate Modbus CRC16 (RTU)
uint16_t modbus_crc16(const uint8_t *data, int len);

// Delay function
void msleep(unsigned int ms);

// Get current time in milliseconds
uint64_t get_time_ms(void);

// ========================================================================
// Logging
// ========================================================================
#define LOG_LEVEL_DEBUG   0
#define LOG_LEVEL_INFO    1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_ERROR   3

extern int g_log_level;

#define LOG_DEBUG(fmt, ...) \
    do { if (g_log_level <= LOG_LEVEL_DEBUG) \
        printf("[DEBUG] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_INFO(fmt, ...) \
    do { if (g_log_level <= LOG_LEVEL_INFO) \
        printf("[INFO ] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_WARN(fmt, ...) \
    do { if (g_log_level <= LOG_LEVEL_WARN) \
        printf("[WARN ] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_ERROR(fmt, ...) \
    do { if (g_log_level <= LOG_LEVEL_ERROR) \
        printf("[ERROR] " fmt "\n", ##__VA_ARGS__); } while(0)

#endif /* COMMON_H */
