/* Copyright 2024. All Rights Reserved. */
/* Data Simulator - Fake Hardware Data Generator */

#ifndef DATA_SIM_H
#define DATA_SIM_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Data Simulator for HF25029-CDP
 *
 * Simulates all hardware data without real hardware:
 * - 22 Digital Inputs (DI)
 * - 12 Digital Outputs (DO)
 * - 5 Potentiometer Gains
 * - 5 RS485 Input Channels
 * - 3 RS232 Channels
 */

// ========================================================================
// Configuration
// ========================================================================

// Simulation tick interval (ms)
#define DATA_SIM_TICK_MS       10

// DI change interval (ms)
#define DATA_SIM_DI_CHANGE_MS   100

// Potentiometer sine wave period (seconds)
#define DATA_SIM_POT_PERIOD_S   10.0

// RS485 frame update interval (ms)
#define DATA_SIM_RS485_UPDATE_MS  20

// RS232 frame max size
#define DATA_SIM_RS232_MAX_LEN   256

// ========================================================================
// Initialization
// ========================================================================

// Initialize simulator with default values
void data_sim_init(void);

// Reset simulator to default state
void data_sim_reset(void);

// ========================================================================
// Data Update (call periodically)
// ========================================================================

// Update all simulated data based on elapsed time
// tick_ms: milliseconds since last update
void data_sim_update(uint32_t tick_ms);

// ========================================================================
// DI Data (22 channels)
// ========================================================================

// Get all 22 DI status as a bitmask
// Bit[0] = DI_CH01, Bit[1] = DI_CH02, etc.
uint32_t data_sim_get_di_status(void);

// Get single DI channel status
// channel: 0-21
bool data_sim_get_di(uint8_t channel);

// Get DI latch status (change detection)
// Returns bitmask of channels that have changed
uint32_t data_sim_get_di_latch(void);

// Clear DI latch for specific channel
// channel: 0-21
void data_sim_clear_di_latch(uint8_t channel);

// Clear all DI latches
void data_sim_clear_all_di_latch(void);

// ========================================================================
// DO Data (12 channels)
// ========================================================================

// Get all 12 DO status as a bitmask
uint16_t data_sim_get_do_status(void);

// Get single DO channel status
bool data_sim_get_do(uint8_t channel);

// Set single DO channel status (from master write)
void data_sim_set_do(uint8_t channel, bool value);

// ========================================================================
// Potentiometer Data (5 channels)
// ========================================================================

// Get potentiometer raw value (0-4095)
// channel: 0-4
uint16_t data_sim_get_pot_gain(uint8_t channel);

// Get potentiometer percentage (0-100)
// channel: 0-4
uint16_t data_sim_get_pot_percent(uint8_t channel);

// ========================================================================
// RS485 Data (5 channels)
// ========================================================================

// Get RS485 channel data length
// channel: 0-4, returns 0-10
uint8_t data_sim_get_rs485_len(uint8_t channel);

// Get RS485 channel data
// channel: 0-4
// buf: buffer to store data (max 10 bytes)
// returns: actual data length
int data_sim_get_rs485_data(uint8_t channel, uint8_t *buf, int max_len);

// Get RS485 channel status
// channel: 0-4
// Returns status bits:
//   Bit 0: Data Ready
//   Bit 1: CRC Error
//   Bit 2: Frame Error
//   Bit 3: Overflow Error
//   Bit 4: Timeout Error
uint8_t data_sim_get_rs485_stat(uint8_t channel);

// Clear RS485 status bits
void data_sim_clear_rs485_stat(uint8_t channel, uint8_t bits);

// ========================================================================
// RS232 Data (3 channels)
// ========================================================================

// Get RS232 channel receive length
// channel: 0-2, returns 0-256
uint16_t data_sim_get_rs232_len(uint8_t channel);

// Get RS232 channel receive data
// channel: 0-2
// buf: buffer to store data
// max_len: buffer size
// returns: actual data length
int data_sim_get_rs232_data(uint8_t channel, uint8_t *buf, int max_len);

// Get RS232 channel status
// channel: 0-2
uint16_t data_sim_get_rs232_stat(uint8_t channel);

// Get RS232 channel error status
// channel: 0-2
uint8_t data_sim_get_rs232_err(uint8_t channel);

// Write data to RS232 TX buffer (simulating received data from external device)
// channel: 0-2
// buf: data to write
// len: data length (0-256)
void data_sim_put_rs232_data(uint8_t channel, const uint8_t *buf, int len);

// Clear RS232 receive buffer
void data_sim_clear_rs232_rx(uint8_t channel);

// ========================================================================
// Device Configuration
// ========================================================================

// Get simulated slave address
uint8_t data_sim_get_slave_addr(void);

// Set simulated slave address
void data_sim_set_slave_addr(uint8_t addr);

// Get simulated baudrate code
uint8_t data_sim_get_baudrate(void);

// Get simulated parity
uint8_t data_sim_get_parity(void);

// ========================================================================
// RS232 Parameter Configuration
// ========================================================================

// Get RS232 port configuration
// channel: 0-2
// baudrate_code: returned baudrate code
// databits: returned data bits (7 or 8)
// stopbits: returned stop bits (1 or 2)
// parity: returned parity (0=none, 1=odd, 2=even)
void data_sim_get_rs232_param(uint8_t channel,
                               uint8_t *baudrate_code,
                               uint8_t *databits,
                               uint8_t *stopbits,
                               uint8_t *parity);

// Set RS232 port configuration
void data_sim_set_rs232_param(uint8_t channel,
                               uint8_t baudrate_code,
                               uint8_t databits,
                               uint8_t stopbits,
                               uint8_t parity);

// ========================================================================
// RS485 Parameter Configuration
// ========================================================================

// Get RS485 port configuration
void data_sim_get_rs485_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity);

// Set RS485 port configuration
void data_sim_set_rs485_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity);

// ========================================================================
// RS232 Control
// ========================================================================

// Get RS232 TX buffer length
uint16_t data_sim_get_rs232_tx_len(uint8_t channel);

// Set RS232 TX buffer length (for master to send)
void data_sim_set_rs232_tx_len(uint8_t channel, uint16_t len);

// Get RS232 TX data
int data_sim_get_rs232_tx_data(uint8_t channel, uint8_t *buf, int max_len);

// Start RS232 transmission
void data_sim_rs232_start_tx(uint8_t channel);

// Clear RS232 TX buffer
void data_sim_clear_rs232_tx(uint8_t channel);

// ========================================================================
// Debug
// ========================================================================

// Print current simulation state
void data_sim_print_state(void);

#endif /* DATA_SIM_H */
