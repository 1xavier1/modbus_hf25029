/* Copyright 2024. All Rights Reserved. */
/* Modbus RTU Slave Implementation */

#include "modbus_rtu.h"
#include "serial.h"
#include "modbus_func.h"
#include "register_map.h"
#include "data_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

// ========================================================================
// CRC16 Calculation
// ========================================================================

static uint16_t calc_crc16(const uint8_t *data, int len) {
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

// ========================================================================
// Modbus Request Processing
// ========================================================================

typedef struct {
    uint8_t slave_addr;
    uint8_t func_code;
    uint16_t start_addr;
    uint16_t quantity;
    uint8_t byte_count;  // for write operations
    uint8_t data[256];
} modbus_request_t;

typedef struct {
    uint8_t func_code;
    uint8_t byte_count;
    uint8_t data[256];
} modbus_response_t;

static int parse_request(const uint8_t *buf, int len, modbus_request_t *req) {
    if (len < 4) return -1;

    req->slave_addr = buf[0];
    req->func_code = buf[1];

    switch (req->func_code) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUT:
        case MODBUS_FC_READ_HOLDING_REG:
        case MODBUS_FC_READ_INPUT_REG:
            if (len < 6) return -1;
            req->start_addr = (buf[2] << 8) | buf[3];
            req->quantity = (buf[4] << 8) | buf[5];
            break;

        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REG:
            if (len < 6) return -1;
            req->start_addr = (buf[2] << 8) | buf[3];
            req->quantity = (buf[4] << 8) | buf[5];
            break;

        case MODBUS_FC_WRITE_MULTIPLE_COILS:
        case MODBUS_FC_WRITE_MULTIPLE_REGS:
            if (len < 7) return -1;
            req->start_addr = (buf[2] << 8) | buf[3];
            req->quantity = (buf[4] << 8) | buf[5];
            req->byte_count = buf[6];
            if (len < 7 + req->byte_count) return -1;
            memcpy(req->data, &buf[7], req->byte_count);
            break;

        default:
            return -1;
    }

    return 0;
}

// ========================================================================
// Modbus Function Handlers
// ========================================================================

static int handle_read_coils(uint16_t start_addr, uint16_t quantity, uint8_t *data, int *data_len) {
    if (start_addr < 1 || start_addr > 12) return MODBUS_EX_ILLEGAL_DATA_ADDR;
    if (quantity < 1 || quantity > 2000) return MODBUS_EX_ILLEGAL_DATA_ADDR;

    uint16_t coil_status = data_sim_get_do_status();

    *data_len = (quantity + 7) / 8;
    memset(data, 0, *data_len);

    for (int i = 0; i < quantity; i++) {
        int coil_addr = start_addr + i - 1;  // 0-based
        if (coil_addr >= 0 && coil_addr < 12) {
            if ((coil_status >> coil_addr) & 1) {
                data[i / 8] |= (1 << (i % 8));
            }
        }
    }

    return 0;
}

static int handle_read_discrete_input(uint16_t start_addr, uint16_t quantity, uint8_t *data, int *data_len) {
    if (start_addr < 1 || start_addr > 22) return MODBUS_EX_ILLEGAL_DATA_ADDR;
    if (quantity < 1 || quantity > 2000) return MODBUS_EX_ILLEGAL_DATA_ADDR;

    uint32_t di_status = data_sim_get_di_status();

    *data_len = (quantity + 7) / 8;
    memset(data, 0, *data_len);

    for (int i = 0; i < quantity; i++) {
        int di_addr = start_addr + i - 1;  // 0-based
        if (di_addr >= 0 && di_addr < 22) {
            if ((di_status >> di_addr) & 1) {
                data[i / 8] |= (1 << (i % 8));
            }
        }
    }

    return 0;
}

static int handle_read_holding_reg(uint16_t start_addr, uint16_t quantity, uint8_t *data, int *data_len) {
    if (quantity < 1 || quantity > 125) return MODBUS_EX_ILLEGAL_DATA_ADDR;

    *data_len = 0;

    for (int i = 0; i < quantity; i++) {
        uint16_t addr = start_addr + i;
        uint16_t value = 0;

        if (addr >= REG_DEVICE_ADDR && addr <= REG_DEVICE_ADDR) {
            value = data_sim_get_slave_addr();
        } else if (addr >= REG_BAUDRATE && addr <= REG_BAUDRATE) {
            value = data_sim_get_baudrate();
        } else if (addr >= REG_DI_CLEAR_BASE && addr < REG_DI_CLEAR_BASE + REG_DI_CLEAR_COUNT) {
            // Read clear register - return 0
            value = 0;
        } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
            int ch = (addr - REG_RS232_PARAM_BASE) / 4;
            int idx = (addr - REG_RS232_PARAM_BASE) % 4;
            uint8_t baud, db, sb, par;
            data_sim_get_rs232_param(ch, &baud, &db, &sb, &par);
            if (idx == 0) value = baud;
            else if (idx == 1) value = db;
            else if (idx == 2) value = sb;
            else if (idx == 3) value = par;
        } else if (addr >= REG_RS232_TX_LEN_BASE && addr < REG_RS232_TX_LEN_BASE + 3) {
            int ch = addr - REG_RS232_TX_LEN_BASE;
            value = data_sim_get_rs232_tx_len(ch);
        } else {
            // For other addresses, return 0
            value = 0;
        }

        data[(*data_len)++] = (value >> 8) & 0xFF;
        data[(*data_len)++] = value & 0xFF;
    }

    return 0;
}

static int handle_read_input_reg(uint16_t start_addr, uint16_t quantity, uint8_t *data, int *data_len) {
    if (quantity < 1 || quantity > 125) return MODBUS_EX_ILLEGAL_DATA_ADDR;

    *data_len = 0;

    for (int i = 0; i < quantity; i++) {
        uint16_t addr = start_addr + i;

        uint16_t value = 0;

        // Potentiometer gains (30001-30005)
        if (addr >= REG_POT_GAIN_BASE && addr < REG_POT_GAIN_BASE + REG_POT_GAIN_COUNT) {
            int ch = addr - REG_POT_GAIN_BASE;
            value = data_sim_get_pot_gain(ch);
        }
        // DI Latch (30006-30027)
        else if (addr >= REG_DI_LATCH_BASE && addr < REG_DI_LATCH_BASE + REG_DI_LATCH_COUNT) {
            int ch = addr - REG_DI_LATCH_BASE;
            value = (data_sim_get_di_latch() >> ch) & 1;
        }
        // RS485 Length (30028-30032)
        else if (addr >= REG_RS485_LEN_BASE && addr < REG_RS485_LEN_BASE + REG_RS485_LEN_COUNT) {
            int ch = addr - REG_RS485_LEN_BASE;
            value = data_sim_get_rs485_len(ch);
        }
        // RS485 Data (30033-30082)
        else if (addr >= REG_RS485_DATA_BASE && addr < REG_RS485_DATA_BASE + REG_RS485_DATA_COUNT) {
            int ch = (addr - REG_RS485_DATA_BASE) / REG_RS485_DATA_PER_CH;
            int idx = (addr - REG_RS485_DATA_BASE) % REG_RS485_DATA_PER_CH;
            uint8_t buf[10];
            int len = data_sim_get_rs485_data(ch, buf, sizeof(buf));
            if (idx < len) {
                value = buf[idx];
            }
        }
        // RS485 Status (30083-30087)
        else if (addr >= REG_RS485_STAT_BASE && addr < REG_RS485_STAT_BASE + REG_RS485_STAT_COUNT) {
            int ch = addr - REG_RS485_STAT_BASE;
            value = data_sim_get_rs485_stat(ch);
        }
        // RS232 Length (30088-30090)
        else if (addr >= REG_RS232_LEN_BASE && addr < REG_RS232_LEN_BASE + REG_RS232_LEN_COUNT) {
            int ch = addr - REG_RS232_LEN_BASE;
            value = data_sim_get_rs232_len(ch);
        }
        // RS232 Data (30091-30858)
        else if (addr >= REG_RS232_DATA_BASE && addr < REG_RS232_DATA_BASE + REG_RS232_DATA_COUNT) {
            int ch = (addr - REG_RS232_DATA_BASE) / REG_RS232_DATA_PER_CH;
            int idx = (addr - REG_RS232_DATA_BASE) % REG_RS232_DATA_PER_CH;
            uint8_t buf[256];
            int len = data_sim_get_rs232_data(ch, buf, sizeof(buf));
            if (idx < len) {
                value = buf[idx];
            }
        }
        // RS232 Status (30859-30864)
        else if (addr >= REG_RS232_STAT_BASE && addr < REG_RS232_STAT_BASE + REG_RS232_STAT_COUNT) {
            int ch = addr - REG_RS232_STAT_BASE;
            value = data_sim_get_rs232_stat(ch);
        }

        data[(*data_len)++] = (value >> 8) & 0xFF;
        data[(*data_len)++] = value & 0xFF;
    }

    return 0;
}

static int handle_write_single_coil(uint16_t addr, uint16_t value) {
    if (addr < 1 || addr > 12) return MODBUS_EX_ILLEGAL_DATA_ADDR;
    if (value != 0x0000 && value != 0xFF00) return MODBUS_EX_ILLEGAL_DATA_VALUE;

    int ch = addr - 1;
    data_sim_set_do(ch, value != 0);

    return 0;
}

static int handle_write_single_reg(uint16_t addr, uint16_t value) {
    // Handle single register writes
    if (addr >= REG_DEVICE_ADDR && addr <= REG_DEVICE_ADDR) {
        if (value >= 1 && value <= 247) {
            data_sim_set_slave_addr(value);
        }
    } else if (addr >= REG_BAUDRATE && addr <= REG_BAUDRATE) {
        // Baudrate change would require serial port reconfiguration
    } else if (addr >= REG_SAVE_PARAM && addr <= REG_SAVE_PARAM) {
        if (value == SAVE_PARAM_VALUE) {
            // Save parameters
        }
    } else if (addr >= REG_DI_CLEAR_BASE && addr < REG_DI_CLEAR_BASE + REG_DI_CLEAR_COUNT) {
        int ch = addr - REG_DI_CLEAR_BASE;
        if (value == 0x0001) {
            data_sim_clear_di_latch(ch);
        }
    } else if (addr >= REG_RS232_CTRL_BASE && addr < REG_RS232_CTRL_BASE + 3) {
        int ch = addr - REG_RS232_CTRL_BASE;
        if (value & 0x0001) {
            // Start transmit
        }
        if (value & 0x0002) {
            data_sim_clear_rs232_rx(ch);
        }
        if (value & 0x0004) {
            data_sim_clear_rs232_tx(ch);
        }
    }

    return 0;
}

static int process_request(const uint8_t *req_data, int req_len,
                           uint8_t *resp_data, int *resp_len) {
    modbus_request_t req;
    modbus_response_t resp;

    if (parse_request(req_data, req_len, &req) < 0) {
        return -1;
    }

    // Check slave address
    uint8_t slave_addr = data_sim_get_slave_addr();
    if (req.slave_addr != slave_addr && req.slave_addr != 0) {
        return -1;  // Not for us
    }

    resp.func_code = req.func_code;
    resp.byte_count = 0;

    int exception = 0;

    switch (req.func_code) {
        case MODBUS_FC_READ_COILS:
            exception = handle_read_coils(req.start_addr, req.quantity,
                                         resp.data, &resp.byte_count);
            break;

        case MODBUS_FC_READ_DISCRETE_INPUT:
            exception = handle_read_discrete_input(req.start_addr, req.quantity,
                                                 resp.data, &resp.byte_count);
            break;

        case MODBUS_FC_READ_HOLDING_REG:
            exception = handle_read_holding_reg(req.start_addr, req.quantity,
                                               resp.data, &resp.byte_count);
            break;

        case MODBUS_FC_READ_INPUT_REG:
            exception = handle_read_input_reg(req.start_addr, req.quantity,
                                             resp.data, &resp.byte_count);
            break;

        case MODBUS_FC_WRITE_SINGLE_COIL:
            exception = handle_write_single_coil(req.start_addr, req.quantity);
            if (exception == 0) {
                // Echo back the request for coil write
                resp.byte_count = 4;
                resp.data[0] = req_data[2];
                resp.data[1] = req_data[3];
                resp.data[2] = req_data[4];
                resp.data[3] = req_data[5];
            }
            break;

        case MODBUS_FC_WRITE_SINGLE_REG:
            exception = handle_write_single_reg(req.start_addr, req.quantity);
            if (exception == 0) {
                // Echo back the request for register write
                resp.byte_count = 4;
                resp.data[0] = req_data[2];
                resp.data[1] = req_data[3];
                resp.data[2] = req_data[4];
                resp.data[3] = req_data[5];
            }
            break;

        default:
            exception = MODBUS_EX_ILLEGAL_FUNCTION;
            break;
    }

    // Handle exception
    if (exception != 0) {
        resp.func_code |= 0x80;
        resp.data[0] = exception;
        *resp_len = 3;
    } else {
        *resp_len = 2 + resp.byte_count;
    }

    memcpy(resp_data, &resp.func_code, *resp_len);
    return 0;
}

// ========================================================================
// RTU Slave Thread
// ========================================================================

void* rtu_slave_thread(void *arg) {
    const char *dev = (const char*)arg;
    if (dev == NULL) {
        dev = MODBUS_RTU_DEFAULT_DEV;
    }

    printf("[RTU] Starting Modbus RTU slave on %s\n", dev);

    int fd = serial_open(dev, MODBUS_RTU_DEFAULT_BAUD,
                         MODBUS_RTU_DEFAULT_PARITY,
                         MODBUS_RTU_DEFAULT_BITS,
                         MODBUS_RTU_DEFAULT_STOP);

    if (fd < 0) {
        printf("[RTU] Failed to open serial port\n");
        return NULL;
    }

    printf("[RTU] Serial port opened, listening...\n");

    uint8_t rx_buf[256];
    int rx_pos = 0;

    while (!g_quit) {
        uint8_t byte;
        int n = serial_read(fd, &byte, 1, 10);  // 10ms timeout

        if (n > 0) {
            rx_buf[rx_pos++] = byte;

            // Simple frame detection - wait for minimum bytes
            if (rx_pos >= 8) {
                // Try to process complete frame
                uint16_t crc_received = (rx_buf[rx_pos-1] << 8) | rx_buf[rx_pos-2];
                uint16_t crc_calc = calc_crc16(rx_buf, rx_pos - 2);

                if (crc_received == crc_calc) {
                    // Valid frame
                    uint8_t resp[256];
                    int resp_len;

                    int ret = process_request(rx_buf, rx_pos - 2,  // Exclude CRC
                                            resp, &resp_len);

                    if (ret == 0) {
                        // Add CRC to response
                        uint16_t crc = calc_crc16(resp, resp_len);
                        resp[resp_len++] = crc & 0xFF;
                        resp[resp_len++] = (crc >> 8) & 0xFF;

                        serial_write(fd, resp, resp_len);
                    }
                }

                rx_pos = 0;  // Reset buffer
            }

            // Prevent overflow
            if (rx_pos >= sizeof(rx_buf)) {
                rx_pos = 0;
            }
        }
    }

    serial_close(fd);
    printf("[RTU] Thread exiting\n");
    return NULL;
}
