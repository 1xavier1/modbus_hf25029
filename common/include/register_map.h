/* Copyright 2024. All Rights Reserved. */
/* HF25029-CDP Modbus Register Map Definition */

#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#include <stdint.h>

/*
 * Modbus Register Map for HF25029-CDP Signal Acquisition Module
 *
 * All addresses are PROTOCOL addresses (0-based offset from base):
 * - Input Registers: PDU address 0-863 corresponds to 30001-30864
 * - Holding Registers: PDU address 0-860 corresponds to 40001-40861
 * - Coils: PDU address 0-11 corresponds to 00001-00012
 * - Discrete Inputs: PDU address 0-21 corresponds to 10001-10022
 *
 * Version: V1.2 (RS485 协议配置新增 40857-40861)
 */

// ========================================================================
// Input Registers (PDU address 0-863) - Read Only
// ========================================================================

// 5-channel Potentiometer Gain (PDU address 0-4)
// Range: 0x0000-0x0FFF (0-100%), 16-bit resolution
#define REG_POT_GAIN_BASE         0
#define REG_POT_GAIN_COUNT        5

// 22-channel DI Latch Status (PDU address 5-26)
// Each bit indicates if corresponding DI has changed since last clear
#define REG_DI_LATCH_BASE         5
#define REG_DI_LATCH_COUNT        22

// 5-channel RS485 Receive Length (PDU address 27-31)
// Range: 0-3 bytes per channel (V1.2)
#define REG_RS485_LEN_BASE        27
#define REG_RS485_LEN_COUNT       5
#define REG_RS485_LEN_MAX        3

// 5-channel RS485 Receive Data (PDU address 32-46)
// 3 registers per channel (15 total), 1 byte per register (low 8 bits) (V1.2)
#define REG_RS485_DATA_BASE       32
#define REG_RS485_DATA_COUNT      15
#define REG_RS485_DATA_PER_CH     3

// 5-channel RS485 Status (PDU address 47-51)
#define REG_RS485_STAT_BASE       47
#define REG_RS485_STAT_COUNT      5
// Status bits per channel:
//   Bit 0: Data Ready
//   Bit 1: CRC Error
//   Bit 2: Frame Error
//   Bit 3: Overflow Error
//   Bit 4: Timeout Error

// 3-channel RS232 Receive Length (PDU address 52-54)
// Range: 0-256 bytes per channel
#define REG_RS232_LEN_BASE        52
#define REG_RS232_LEN_COUNT       3

// 3-channel RS232 Receive Data (PDU address 55-822)
// 256 registers per channel (768 total), 1 byte per register (low 8 bits)
#define REG_RS232_DATA_BASE       55
#define REG_RS232_DATA_COUNT      768
#define REG_RS232_DATA_PER_CH     256

// 3-channel RS232 Status (PDU address 823-828)
#define REG_RS232_STAT_BASE       823
#define REG_RS232_STAT_COUNT      6
// Status bits per channel:
//   Bit 0: Rx Buffer Not Empty
//   Bit 1: Rx Buffer Overflow
//   Bit 2: Tx Buffer Empty
//   Bit 3: Tx Buffer Full
//   Bit 4: Tx Busy

// Error registers for RS232 (PDU address 826-828)
#define REG_RS232_ERR_BASE        826
// Error bits:
//   Bit 0: Parity Error
//   Bit 1: Frame Error
//   Bit 2: Overflow Error

// ========================================================================
// Holding Registers (PDU address 0-860) - Read/Write
// ========================================================================

// Device Parameters (PDU address 0-19)
#define REG_DEVICE_ADDR           0    // Modbus slave address (1-247, default 1)
#define REG_BAUDRATE             1    // Baudrate code: 0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=115200
#define REG_PARITY               2    // Parity: 0=None, 1=Odd, 2=Even

// IP Address (PDU address 3-6) - 4 octets
#define REG_IP_BASE              3
#define REG_IP_COUNT              4
#define DEFAULT_IP               {192, 168, 1, 100}

// Subnet Mask (PDU address 7-10) - 4 octets
#define REG_NETMASK_BASE          7
#define REG_NETMASK_COUNT         4
#define DEFAULT_NETMASK           {255, 255, 255, 0}

// Save Parameter Flag (PDU address 19)
// Write 0x55AA to save all parameters
#define REG_SAVE_PARAM            19
#define SAVE_PARAM_VALUE          0x55AA

// DI Latch Clear Flags (PDU address 20-41)
// Write 0x0001 to clear corresponding DI latch
#define REG_DI_CLEAR_BASE         20
#define REG_DI_CLEAR_COUNT        22

// RS232 Parameter Configuration (PDU address 42-53)
// 4 registers per channel: Baudrate, DataBits, StopBits, Parity
#define REG_RS232_PARAM_BASE      42
#define REG_RS232_PARAM_COUNT     12
#define REG_RS232_PARAM_PER_CH    4

// RS232 Transmit Length (PDU address 54-56)
// Range: 0-256 bytes
#define REG_RS232_TX_LEN_BASE     54

// RS232 Transmit Data (PDU address 57-824)
// 256 registers per channel
#define REG_RS232_TX_DATA_BASE    57
#define REG_RS232_TX_DATA_COUNT   768
#define REG_RS232_TX_DATA_PER_CH  256

// RS232 Control Commands (PDU address 825-827)
// Write 0x0001: Start transmit
// Write 0x0002: Clear receive buffer
// Write 0x0004: Clear transmit buffer
#define REG_RS232_CTRL_BASE       825
#define REG_RS232_CTRL_COUNT      3

// RS485 Forwarding Control (PDU address 828-832)
// Bit 0: Enable forwarding (1=enable)
// Write 0x0001 to clear status bits
#define REG_RS485_CTRL_BASE       828
#define REG_RS485_CTRL_COUNT      5

// RS485 Parameter Configuration (PDU address 833-852)
// 4 registers per channel: Baudrate, DataBits, StopBits, Parity
#define REG_RS485_PARAM_BASE      833
#define REG_RS485_PARAM_COUNT     20
#define REG_RS485_PARAM_PER_CH    4

// RS485 Protocol Configuration (PDU address 853-857) - V1.2 新增
// 1 register per channel: Protocol selection (1-6, default 1)
#define REG_RS485_PROTO_BASE      853
#define REG_RS485_PROTO_COUNT     5
#define REG_RS485_PROTO_MAX       6

// ========================================================================
// Coils (PDU address 0-11) - Digital Output
// ========================================================================
#define COIL_DO_BASE              0
#define COIL_DO_COUNT             12

// ========================================================================
// Discrete Inputs (PDU address 0-21) - Digital Input
// ========================================================================
#define DI_INPUT_BASE             0
#define DI_INPUT_COUNT            22

// ========================================================================
// Helper Macros
// ========================================================================

// Calculate percentage from potentiometer value
#define POT_VALUE_TO_PERCENT(val) (((val) * 100) / 4095)

// Check if register address is input register (PDU address 0-863)
#define IS_INPUT_REG(addr)        ((addr) >= 0 && (addr) <= 863)

// Check if register address is holding register (PDU address 0-860)
#define IS_HOLDING_REG(addr)      ((addr) >= 0 && (addr) <= 860)

// Check if address is coil (PDU address 0-11)
#define IS_COIL(addr)             ((addr) >= 0 && (addr) <= 11)

// Check if address is discrete input (PDU address 0-21)
#define IS_DISCRETE_INPUT(addr)   ((addr) >= 0 && (addr) <= 21)

// Get protocol address from register number (for reference)
// Holding Register: protocol_addr = reg - 40001
// Input Register: protocol_addr = reg - 30001
// Coil: protocol_addr = reg - 1
// Discrete Input: protocol_addr = reg - 10001
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
