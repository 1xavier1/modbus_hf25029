/* Copyright 2024. All Rights Reserved. */
/* HF25029-CDP Modbus Register Map Definition */

#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#include <stdint.h>

/*
 * Modbus Register Map for HF25029-CDP Signal Acquisition Module
 *
 * Data Types:
 * - Input Registers (30001-30864): Read-only, Function Code 0x04
 * - Holding Registers (40001-40836): Read/Write, Function Code 0x03/0x06/0x10
 * - Coils (00001-00012): Read/Write, Function Code 0x01/0x05/0x0F
 * - Discrete Inputs (10001-10022): Read-only, Function Code 0x02
 */

// ========================================================================
// Input Registers (30001-30864) - Read Only
// ========================================================================

// 5-channel Potentiometer Gain (30001-30005)
// Range: 0x0000-0x0FFF (0-100%), 16-bit resolution
#define REG_POT_GAIN_BASE         30001
#define REG_POT_GAIN_COUNT        5

// 22-channel DI Latch Status (30006-30027)
// Each bit indicates if corresponding DI has changed since last clear
#define REG_DI_LATCH_BASE         30006
#define REG_DI_LATCH_COUNT        22

// 5-channel RS485 Receive Length (30028-30032)
// Range: 0-10 bytes per channel
#define REG_RS485_LEN_BASE        30028
#define REG_RS485_LEN_COUNT       5

// 5-channel RS485 Receive Data (30033-30082)
// 10 registers per channel (50 total), 1 byte per register (low 8 bits)
#define REG_RS485_DATA_BASE       30033
#define REG_RS485_DATA_COUNT      50
#define REG_RS485_DATA_PER_CH     10

// 5-channel RS485 Status (30083-30087)
#define REG_RS485_STAT_BASE       30083
#define REG_RS485_STAT_COUNT      5
// Status bits per channel:
//   Bit 0: Data Ready
//   Bit 1: CRC Error
//   Bit 2: Frame Error
//   Bit 3: Overflow Error
//   Bit 4: Timeout Error

// 3-channel RS232 Receive Length (30088-30090)
// Range: 0-256 bytes per channel
#define REG_RS232_LEN_BASE        30088
#define REG_RS232_LEN_COUNT       3

// 3-channel RS232 Receive Data (30091-30858)
// 256 registers per channel (768 total), 1 byte per register (low 8 bits)
#define REG_RS232_DATA_BASE       30091
#define REG_RS232_DATA_COUNT      768
#define REG_RS232_DATA_PER_CH     256

// 3-channel RS232 Status (30859-30864)
#define REG_RS232_STAT_BASE       30859
#define REG_RS232_STAT_COUNT      6
// Status bits per channel:
//   Bit 0: Rx Buffer Not Empty
//   Bit 1: Rx Buffer Overflow
//   Bit 2: Tx Buffer Empty
//   Bit 3: Tx Buffer Full
//   Bit 4: Tx Busy

// Error registers for RS232 (same base + 3)
#define REG_RS232_ERR_BASE        30862
// Error bits:
//   Bit 0: Parity Error
//   Bit 1: Frame Error
//   Bit 2: Overflow Error

// ========================================================================
// Holding Registers (40001-40836) - Read/Write
// ========================================================================

// Device Parameters (40001-40020)
#define REG_DEVICE_ADDR           40001   // Modbus slave address (1-247, default 1)
#define REG_BAUDRATE             40002   // Baudrate code: 0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=115200
#define REG_PARITY               40003   // Parity: 0=None, 1=Odd, 2=Even

// IP Address (40004-40007) - 4 octets
#define REG_IP_BASE              40004
#define REG_IP_COUNT              4
#define DEFAULT_IP               {192, 168, 1, 100}

// Subnet Mask (40008-40011) - 4 octets
#define REG_NETMASK_BASE          40008
#define REG_NETMASK_COUNT         4
#define DEFAULT_NETMASK           {255, 255, 255, 0}

// Save Parameter Flag (40020)
// Write 0x55AA to save all parameters
#define REG_SAVE_PARAM            40020
#define SAVE_PARAM_VALUE          0x55AA

// DI Latch Clear Flags (40021-40042)
// Write 0x0001 to clear corresponding DI latch
#define REG_DI_CLEAR_BASE         40021
#define REG_DI_CLEAR_COUNT        22

// RS232 Parameter Configuration (40043-40054)
// 4 registers per channel: Baudrate, DataBits, StopBits, Parity
#define REG_RS232_PARAM_BASE      40043
#define REG_RS232_PARAM_COUNT     12
#define REG_RS232_PARAM_PER_CH    4

// RS232 Transmit Length (40055-40057)
// Range: 0-256 bytes
#define REG_RS232_TX_LEN_BASE     40055

// RS232 Transmit Data (40058-40825)
// 256 registers per channel
#define REG_RS232_TX_DATA_BASE    40058
#define REG_RS232_TX_DATA_COUNT   768
#define REG_RS232_TX_DATA_PER_CH  256

// RS232 Control Commands (40826-40831)
// Write 0x0001: Start transmit
// Write 0x0002: Clear receive buffer
// Write 0x0004: Clear transmit buffer
#define REG_RS232_CTRL_BASE       40826
#define REG_RS232_CTRL_COUNT      3

// RS485 Forwarding Control (40832-40836)
// Bit 0: Enable forwarding (1=enable)
// Write 0x0001 to clear status bits
#define REG_RS485_CTRL_BASE       40832
#define REG_RS485_CTRL_COUNT      5

// RS485 Parameter Configuration (40837-40856)
// 4 registers per channel: Baudrate, DataBits, StopBits, Parity
#define REG_RS485_PARAM_BASE      40837
#define REG_RS485_PARAM_COUNT     20
#define REG_RS485_PARAM_PER_CH    4

// ========================================================================
// Coils (00001-00012) - Digital Output
// ========================================================================
#define COIL_DO_BASE              1
#define COIL_DO_COUNT             12

// ========================================================================
// Discrete Inputs (10001-10022) - Digital Input
// ========================================================================
#define DI_INPUT_BASE             10001
#define DI_INPUT_COUNT            22

// ========================================================================
// Helper Macros
// ========================================================================

// Calculate percentage from potentiometer value
#define POT_VALUE_TO_PERCENT(val) (((val) * 100) / 4095)

// Check if register address is input register
#define IS_INPUT_REG(addr)        ((addr) >= 30001 && (addr) <= 30864)

// Check if register address is holding register
#define IS_HOLDING_REG(addr)      ((addr) >= 40001 && (addr) <= 40836)

// Check if address is coil
#define IS_COIL(addr)             ((addr) >= 1 && (addr) <= 12)

// Check if address is discrete input
#define IS_DISCRETE_INPUT(addr)   ((addr) >= 10001 && (addr) <= 10022)

// Get protocol address from register number
#define REG_TO_PROTO(reg)         ((reg) % 10000)

// ========================================================================
// GPIO Pin Configuration (Flexible)
// ========================================================================

// Digital Input GPIO pins (22 channels)
// These can be configured flexibly
typedef struct {
    int gpio_num;         // Linux GPIO number
    const char *name;     // Pin name (e.g., "PB2")
    const char *channel;  // Channel name (e.g., "DI_CH01")
} gpio_di_config_t;

// Digital Output GPIO pins (12 channels)
typedef struct {
    int gpio_num;
    const char *name;
    const char *channel;
} gpio_do_config_t;

// Default DI configuration
#define DI_GPIO_COUNT     22
#define DO_GPIO_COUNT     12

extern const gpio_di_config_t g_di_gpio_config[DI_GPIO_COUNT];
extern const gpio_do_config_t g_do_gpio_config[DO_GPIO_COUNT];

#endif /* REGISTER_MAP_H */
