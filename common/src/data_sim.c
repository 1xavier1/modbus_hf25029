/* Copyright 2024. All Rights Reserved. */
/* Data Simulator Implementation */

#include "data_sim.h"
#include "register_map.h"
#include "modbus_func.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ========================================================================
// Internal State
// ========================================================================

// DI state (22 channels)
static uint32_t g_di_status = 0;
static uint32_t g_di_latch = 0;

// DO state (12 channels)
static uint16_t g_do_status = 0;

// Potentiometer values (5 channels, 0-4095)
static uint16_t g_pot_gain[5] = {0, 1024, 2048, 3072, 4095};

// RS485 data (5 channels, max 3 bytes each, V1.2)
static uint8_t g_rs485_data[5][3];
static uint8_t g_rs485_len[5] = {0, 0, 0, 0, 0};
static uint8_t g_rs485_stat[5] = {0, 0, 0, 0, 0};

// RS232 data (3 channels, 256 bytes each)
static uint8_t g_rs232_rx[3][256];
static uint16_t g_rs232_rx_len[3] = {0, 0, 0};
static uint16_t g_rs232_stat[3] = {0, 0, 0};
static uint8_t g_rs232_err[3] = {0, 0, 0};

static uint8_t g_rs232_tx[3][256];
static uint16_t g_rs232_tx_len[3] = {0, 0, 0};

// Device configuration
static uint8_t g_slave_addr = DEFAULT_SLAVE_ADDR;
static uint8_t g_baudrate = BAUD_115200;
static uint8_t g_parity = 0;

// RS232 parameters
static uint8_t g_rs232_param[3][4] = {
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0}
};

// RS485 parameters
static uint8_t g_rs485_param[5][4] = {
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0},
    {BAUD_115200, 8, 1, 0}
};

// RS485 protocol selection (V1.2新增, 1-6)
static uint8_t g_rs485_proto[5] = {1, 1, 1, 1, 1};

// Simulation timing
static uint32_t g_tick_ms = 0;
static uint32_t g_last_di_change = 0;
static uint32_t g_last_rs485_update = 0;
static uint32_t g_last_pot_update = 0;

// Random seed
static unsigned int g_random_seed = 12345;

// Sequence counters for simulated data
static uint8_t g_rs485_seq[5] = {0, 0, 0, 0, 0};

// ========================================================================
// Simple pseudo-random number generator
// ========================================================================
static unsigned int simple_rand(void) {
    g_random_seed = g_random_seed * 1103515245 + 12345;
    return (g_random_seed / 65536) % 32768;
}

static void simple_srand(unsigned int seed) {
    g_random_seed = seed;
}

// ========================================================================
// Initialization
// ========================================================================

void data_sim_init(void) {
    simple_srand(12345);  // Fixed seed for reproducibility

    // Initialize with some random DI states
    g_di_status = simple_rand() & 0x3FFFFF;  // 22 bits

    // Initialize RS485 with some data
    for (int i = 0; i < 5; i++) {
        g_rs485_len[i] = 3;
        g_rs485_data[i][0] = 0xAA;  // Header
        g_rs485_data[i][1] = g_rs485_seq[i];
        g_rs485_data[i][2] = simple_rand() & 0xFF;
        g_rs485_stat[i] = 0x01;  // Data ready
    }

    // Initialize RS232 with some data
    for (int i = 0; i < 3; i++) {
        g_rs232_rx_len[i] = 32;
        for (int j = 0; j < 32; j++) {
            g_rs232_rx[i][j] = simple_rand() & 0xFF;
        }
        g_rs232_stat[i] = 0x01;  // Buffer not empty
    }

    g_tick_ms = 0;
    g_last_di_change = 0;
    g_last_rs485_update = 0;

    printf("[DATA-SIM] Initialized\n");
}

void data_sim_reset(void) {
    memset(g_rs485_data, 0, sizeof(g_rs485_data));
    memset(g_rs485_stat, 0, sizeof(g_rs485_stat));
    memset(g_rs232_rx, 0, sizeof(g_rs232_rx));
    memset(g_rs232_rx_len, 0, sizeof(g_rs232_rx_len));
    memset(g_rs232_stat, 0, sizeof(g_rs232_stat));
    memset(g_rs232_err, 0, sizeof(g_rs232_err));
    memset(g_rs232_tx, 0, sizeof(g_rs232_tx));
    memset(g_rs232_tx_len, 0, sizeof(g_rs232_tx_len));

    g_di_latch = 0;
    g_do_status = 0;

    data_sim_init();
}

// ========================================================================
// Data Update
// ========================================================================

void data_sim_update(uint32_t tick_ms) {
    g_tick_ms += tick_ms;

    // Update DI (random changes)
    if (g_tick_ms - g_last_di_change >= DATA_SIM_DI_CHANGE_MS) {
        uint32_t old_di = g_di_status;

        // Randomly change 1-2 DI channels
        int changes = (simple_rand() % 2) + 1;
        for (int i = 0; i < changes; i++) {
            int ch = simple_rand() % 22;
            uint32_t mask = 1 << ch;
            g_di_status ^= mask;  // Toggle

            // Set latch bit if changed
            if ((old_di & mask) != (g_di_status & mask)) {
                g_di_latch |= mask;
            }
        }

        g_last_di_change = g_tick_ms;
    }

    // Update potentiometer (sine wave)
    if (g_tick_ms - g_last_pot_update >= 100) {  // Update every 100ms
        double t = (double)g_tick_ms / 1000.0;
        for (int i = 0; i < 5; i++) {
            // Different phase for each channel
            double phase = i * (3.14159265 / 2.5);
            double sine = sin(2 * 3.14159265 * t / DATA_SIM_POT_PERIOD_S + phase);
            // Map sine wave (-1 to 1) to 0-4095
            g_pot_gain[i] = (uint16_t)((sine + 1.0) * 2047.5);
        }
        g_last_pot_update = g_tick_ms;
    }

    // Update RS485 data (every 20ms, V1.2: max 3 bytes)
    if (g_tick_ms - g_last_rs485_update >= DATA_SIM_RS485_UPDATE_MS) {
        for (int i = 0; i < 5; i++) {
            // Update sequence number
            g_rs485_seq[i]++;

            g_rs485_data[i][0] = 0xAA;  // Header
            g_rs485_data[i][1] = g_rs485_seq[i];
            g_rs485_data[i][2] = simple_rand() & 0xFF;

            g_rs485_len[i] = 3;  // Fixed 3 bytes (V1.2)

            // Set data ready flag
            g_rs485_stat[i] = 0x01;  // Data ready
        }
        g_last_rs485_update = g_tick_ms;
    }
}

// ========================================================================
// DI Data
// ========================================================================

uint32_t data_sim_get_di_status(void) {
    return g_di_status;
}

bool data_sim_get_di(uint8_t channel) {
    if (channel >= 22) return false;
    return (g_di_status >> channel) & 1;
}

uint32_t data_sim_get_di_latch(void) {
    return g_di_latch;
}

void data_sim_clear_di_latch(uint8_t channel) {
    if (channel >= 22) return;
    g_di_latch &= ~(1 << channel);
}

void data_sim_clear_all_di_latch(void) {
    g_di_latch = 0;
}

// ========================================================================
// DO Data
// ========================================================================

uint16_t data_sim_get_do_status(void) {
    return g_do_status;
}

bool data_sim_get_do(uint8_t channel) {
    if (channel >= 12) return false;
    return (g_do_status >> channel) & 1;
}

void data_sim_set_do(uint8_t channel, bool value) {
    if (channel >= 12) return;
    if (value) {
        g_do_status |= (1 << channel);
    } else {
        g_do_status &= ~(1 << channel);
    }
}

// ========================================================================
// Potentiometer Data
// ========================================================================

uint16_t data_sim_get_pot_gain(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_pot_gain[channel];
}

uint16_t data_sim_get_pot_percent(uint8_t channel) {
    return POT_VALUE_TO_PERCENT(data_sim_get_pot_gain(channel));
}

// ========================================================================
// RS485 Data
// ========================================================================

uint8_t data_sim_get_rs485_len(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_len[channel];
}

int data_sim_get_rs485_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 5 || buf == NULL) return -1;

    int copy_len = g_rs485_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs485_data[channel], copy_len);

    // Clear data ready flag after reading
    g_rs485_stat[channel] &= ~0x01;

    return copy_len;
}

uint8_t data_sim_get_rs485_stat(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_stat[channel];
}

void data_sim_clear_rs485_stat(uint8_t channel, uint8_t bits) {
    if (channel >= 5) return;
    g_rs485_stat[channel] &= ~bits;
}

// ========================================================================
// RS232 Data
// ========================================================================

uint16_t data_sim_get_rs232_len(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_rx_len[channel];
}

int data_sim_get_rs232_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 3 || buf == NULL) return -1;

    int copy_len = g_rs232_rx_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs232_rx[channel], copy_len);

    // Clear buffer not empty flag
    g_rs232_stat[channel] &= ~0x01;

    return copy_len;
}

uint16_t data_sim_get_rs232_stat(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_stat[channel];
}

uint8_t data_sim_get_rs232_err(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_err[channel];
}

void data_sim_put_rs232_data(uint8_t channel, const uint8_t *buf, int len) {
    if (channel >= 3 || buf == NULL || len <= 0) return;
    if (len > 256) len = 256;

    memcpy(g_rs232_rx[channel], buf, len);
    g_rs232_rx_len[channel] = len;
    g_rs232_stat[channel] |= 0x01;  // Set buffer not empty
}

void data_sim_clear_rs232_rx(uint8_t channel) {
    if (channel >= 3) return;
    memset(g_rs232_rx[channel], 0, 256);
    g_rs232_rx_len[channel] = 0;
    g_rs232_stat[channel] &= ~0x01;
}

// ========================================================================
// Device Configuration
// ========================================================================

uint8_t data_sim_get_slave_addr(void) {
    return g_slave_addr;
}

void data_sim_set_slave_addr(uint8_t addr) {
    g_slave_addr = addr;
}

uint8_t data_sim_get_baudrate(void) {
    return g_baudrate;
}

uint8_t data_sim_get_parity(void) {
    return g_parity;
}

// ========================================================================
// RS232 Parameter Configuration
// ========================================================================

void data_sim_get_rs232_param(uint8_t channel,
                               uint8_t *baudrate_code,
                               uint8_t *databits,
                               uint8_t *stopbits,
                               uint8_t *parity) {
    if (channel >= 3) return;
    if (baudrate_code) *baudrate_code = g_rs232_param[channel][0];
    if (databits) *databits = g_rs232_param[channel][1];
    if (stopbits) *stopbits = g_rs232_param[channel][2];
    if (parity) *parity = g_rs232_param[channel][3];
}

void data_sim_set_rs232_param(uint8_t channel,
                               uint8_t baudrate_code,
                               uint8_t databits,
                               uint8_t stopbits,
                               uint8_t parity) {
    if (channel >= 3) return;
    g_rs232_param[channel][0] = baudrate_code;
    g_rs232_param[channel][1] = databits;
    g_rs232_param[channel][2] = stopbits;
    g_rs232_param[channel][3] = parity;
}

// ========================================================================
// RS485 Parameter Configuration
// ========================================================================

void data_sim_get_rs485_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity) {
    if (channel >= 5) return;
    if (baudrate_code) *baudrate_code = g_rs485_param[channel][0];
    if (databits) *databits = g_rs485_param[channel][1];
    if (stopbits) *stopbits = g_rs485_param[channel][2];
    if (parity) *parity = g_rs485_param[channel][3];
}

void data_sim_set_rs485_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity) {
    if (channel >= 5) return;
    g_rs485_param[channel][0] = baudrate_code;
    g_rs485_param[channel][1] = databits;
    g_rs485_param[channel][2] = stopbits;
    g_rs485_param[channel][3] = parity;
}

// ========================================================================
// RS485 Protocol Configuration (V1.2 新增)
// ========================================================================

uint8_t data_sim_get_rs485_proto(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_proto[channel];
}

void data_sim_set_rs485_proto(uint8_t channel, uint8_t protocol) {
    if (channel >= 5) return;
    if (protocol < 1) protocol = 1;
    if (protocol > 6) protocol = 6;
    g_rs485_proto[channel] = protocol;
}

// ========================================================================
// RS232 Control
// ========================================================================

uint16_t data_sim_get_rs232_tx_len(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_tx_len[channel];
}

void data_sim_set_rs232_tx_len(uint8_t channel, uint16_t len) {
    if (channel >= 3) return;
    if (len > 256) len = 256;
    g_rs232_tx_len[channel] = len;
}

int data_sim_get_rs232_tx_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 3 || buf == NULL) return -1;

    int copy_len = g_rs232_tx_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs232_tx[channel], copy_len);
    return copy_len;
}

void data_sim_rs232_start_tx(uint8_t channel) {
    if (channel >= 3) return;
    // Simulate transmission complete immediately
    // In real implementation, this would trigger async transmission
    g_rs232_stat[channel] |= 0x04;  // TX buffer empty
}

void data_sim_clear_rs232_tx(uint8_t channel) {
    if (channel >= 3) return;
    memset(g_rs232_tx[channel], 0, 256);
    g_rs232_tx_len[channel] = 0;
}

// ========================================================================
// Debug
// ========================================================================

void data_sim_print_state(void) {
    printf("\n========== Data Simulator State ==========\n");

    printf("DI Status:  0x%06X\n", g_di_status);
    printf("DI Latch:   0x%06X\n", g_di_latch);
    printf("DO Status:   0x%03X\n", g_do_status);

    printf("Pot Gain:   ");
    for (int i = 0; i < 5; i++) {
        printf("%d=%d(%d%%) ", i+1, g_pot_gain[i], POT_VALUE_TO_PERCENT(g_pot_gain[i]));
    }
    printf("\n");

    printf("RS485:      ");
    for (int i = 0; i < 5; i++) {
        printf("CH%d=%dbytes,", i+1, g_rs485_len[i]);
    }
    printf("\n");

    printf("RS232 RX:   ");
    for (int i = 0; i < 3; i++) {
        printf("CH%d=%dbytes,", i+1, g_rs232_rx_len[i]);
    }
    printf("\n");

    printf("Slave Addr: %d, Baudrate: %d\n", g_slave_addr, g_baudrate);
    printf("==========================================\n\n");
}
