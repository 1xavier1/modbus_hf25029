/* Copyright 2024. All Rights Reserved. */
/* Hardware Abstraction Layer - Real Hardware Access */

#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

/*
 * Hardware Abstraction Layer for HF25029-CDP
 *
 * Provides real hardware access instead of simulation:
 * - AD7606 ADC (5 channels via GPIO模拟SPI)
 * - GPIO for DI/DO
 * - UART for RS485/RS232
 */

// ========================================================================
// Configuration
// ========================================================================

// Hardware device paths
#define DEV_AD7606       "/dev/spi3_ad7606"
#define DEV_GPIO         "/dev/gpio"

// RS485 Channels (实际设备节点对应关系)
#define DEV_RS485_CH1    "/dev/ttyAS9"   // UART9  - PJ15
#define DEV_RS485_CH2    "/dev/ttyAS7"   // UART7  - PB12
#define DEV_RS485_CH3    "/dev/ttyAS2"   // UART2  - PB2 (与RTU共享)
#define DEV_RS485_CH4    "/dev/ttyAS8"   // UART8  - PB3
#define DEV_RS485_CH5    "/dev/ttyAS4"   // UART4  - PG13
#define DEV_RS485_CH6    "/dev/ttyAS3"   // UART3  - PD12

// RS232 Channels (预留，目前未使用)
#define DEV_RS232_CH1    "/dev/ttyAS5"   // UART5  - 未配置
#define DEV_RS232_CH2    "/dev/ttyAS6"   // UART6  - 未配置
#define DEV_RS232_CH3    "/dev/ttyAS1"   // UART1  - 未配置

// GPIO pin definitions for DI (22 channels)
#define DI_GPIO_BASE  34   // First DI GPIO
#define DI_GPIO_COUNT 22

// GPIO pin definitions for DO (12 channels)
#define DO_GPIO_BASE  35   // First DO GPIO
#define DO_GPIO_COUNT 12

// ========================================================================
// Initialization
// ========================================================================

// Initialize all hardware
// Returns 0 on success, -1 on failure
int hardware_init(void);

// Load network configuration from file
// If config_file is NULL, uses default path /etc/modbus_slave/network.conf
// Returns 0 on success, -1 if file not found
int hardware_load_config(const char *config_file);

// Release all hardware resources
void hardware_release(void);

// ========================================================================
// AD7606 ADC (Potentiometer Gain)
// ========================================================================

// Get potentiometer raw ADC value (0-4095) for specified channel
// channel: 0-4
// Returns: ADC raw value 0-4095
uint16_t hardware_get_pot_gain(uint8_t channel);

// Get potentiometer percentage (0-100)
// channel: 0-4
uint16_t hardware_get_pot_percent(uint8_t channel);

// Trigger ADC conversion and get all 5 channels
// samples: array of 5 uint16_t to store results
void hardware_get_all_adc(uint16_t *samples);

// ========================================================================
// Digital Input (DI)
// ========================================================================

// Get all 22 DI status as a bitmask
// Bit[0] = DI_CH01, Bit[1] = DI_CH02, etc.
uint32_t hardware_get_di_status(void);

// Get single DI channel status
// channel: 0-21
bool hardware_get_di(uint8_t channel);

// Get DI latch status (change detection since last clear)
// Returns bitmask of channels that have changed
uint32_t hardware_get_di_latch(void);

// Clear DI latch for specific channel
void hardware_clear_di_latch(uint8_t channel);

// Clear all DI latches
void hardware_clear_all_di_latch(void);

// ========================================================================
// Digital Output (DO)
// ========================================================================

// Get all 12 DO status as a bitmask
uint16_t hardware_get_do_status(void);

// Get single DO channel status
bool hardware_get_do(uint8_t channel);

// Set single DO channel status
void hardware_set_do(uint8_t channel, bool value);

// ========================================================================
// RS485 Data (5 channels)
// ========================================================================

// Get RS485 channel data length
// channel: 0-4, returns 0-3 (V1.2 max 3 bytes)
uint8_t hardware_get_rs485_len(uint8_t channel);

// Get RS485 channel received data
// channel: 0-4
// buf: buffer to store data (max 3 bytes)
// returns: actual data length
int hardware_get_rs485_data(uint8_t channel, uint8_t *buf, int max_len);

// Get RS485 channel status
// Returns status bits:
//   Bit 0: Data Ready
//   Bit 1: CRC Error
//   Bit 2: Frame Error
//   Bit 3: Overflow Error
//   Bit 4: Timeout Error
uint8_t hardware_get_rs485_stat(uint8_t channel);

// Clear RS485 status bits
void hardware_clear_rs485_stat(uint8_t channel, uint8_t bits);

// ========================================================================
// RS232 Data (3 channels)
// ========================================================================

// Get RS232 channel receive length
// channel: 0-2, returns 0-256
uint16_t hardware_get_rs232_len(uint8_t channel);

// Get RS232 channel receive data
// channel: 0-2
// buf: buffer to store data
// max_len: buffer size
// returns: actual data length
int hardware_get_rs232_data(uint8_t channel, uint8_t *buf, int max_len);

// Get RS232 channel status
uint16_t hardware_get_rs232_stat(uint8_t channel);

// Get RS232 channel error status
uint8_t hardware_get_rs232_err(uint8_t channel);

// Write data to RS232 TX buffer
// channel: 0-2
// buf: data to write
// len: data length (0-256)
void hardware_put_rs232_data(uint8_t channel, const uint8_t *buf, int len);

// Clear RS232 receive buffer
void hardware_clear_rs232_rx(uint8_t channel);

// ========================================================================
// Device Configuration
// ========================================================================

// Get device address
uint8_t hardware_get_slave_addr(void);

// Set device address
void hardware_set_slave_addr(uint8_t addr);

// Get baudrate code
uint8_t hardware_get_baudrate(void);

// Set baudrate code
void hardware_set_baudrate(uint8_t baudrate);

// Get parity
uint8_t hardware_get_parity(void);

// Set parity
void hardware_set_parity(uint8_t parity);

// ========================================================================
// Network Configuration
// ========================================================================

// Get IP address (4 bytes: octet1, octet2, octet3, octet4)
void hardware_get_ip_addr(uint8_t *ip);

// Set IP address
void hardware_set_ip_addr(uint8_t *ip);

// Get subnet mask
void hardware_get_netmask(uint8_t *netmask);

// Set subnet mask
void hardware_set_netmask(uint8_t *netmask);

// ========================================================================
// RS232/RS485 Parameter Configuration
// ========================================================================

void hardware_get_rs232_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity);

void hardware_set_rs232_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity);

void hardware_get_rs485_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity);

void hardware_set_rs485_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity);

// ========================================================================
// RS485 Protocol Configuration (V1.2)
// ========================================================================

uint8_t hardware_get_rs485_proto(uint8_t channel);
void hardware_set_rs485_proto(uint8_t channel, uint8_t protocol);

// RS485 Forwarding Control (V1.2)
// ========================================================================

uint8_t hardware_get_rs485_ctrl(uint8_t channel);
void hardware_set_rs485_ctrl(uint8_t channel, uint8_t ctrl);

// RS485 Polling - call periodically (e.g., every 20ms) to receive data
void hardware_rs485_poll_all(void);

// ========================================================================
// RS232 Control
// ========================================================================

uint16_t hardware_get_rs232_tx_len(uint8_t channel);
void hardware_set_rs232_tx_len(uint8_t channel, uint16_t len);
int hardware_get_rs232_tx_data(uint8_t channel, uint8_t *buf, int max_len);
void hardware_rs232_start_tx(uint8_t channel);
void hardware_clear_rs232_tx(uint8_t channel);

// ========================================================================
// Debug
// ========================================================================

void hardware_print_state(void);

// ========================================================================
// Persistent Storage
// ========================================================================

// Flush all dirty registers to persistent storage
void hardware_persist_flush(void);

// Initialize persistent storage subsystem
void hardware_persist_init(void);

#endif /* HARDWARE_H */
