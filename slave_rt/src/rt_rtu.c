/* Copyright 2024. All Rights Reserved. */
/* Real-time Modbus RTU Slave */

#include "rt_rtu.h"
#include "rt_utils.h"
#include "serial.h"
#include "modbus_func.h"
#include "register_map.h"
#include "data_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

// RTU configuration
#define RTU_DEV              "/dev/ttyAS2"
#define RTU_BAUD             115200
#define RTU_PERIOD_US        1000    // 1ms period

// CRC16 calculation
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

// Process Modbus request (simplified version)
static int process_rtu_request(const uint8_t *req, int req_len,
                               uint8_t *resp, int *resp_len) {
    if (req_len < 4) return -1;

    uint8_t slave_addr = req[0];
    uint8_t func_code = req[1];

    // Check slave address
    if (slave_addr != data_sim_get_slave_addr() && slave_addr != 0) {
        return -1;
    }

    int payload_len = 0;

    switch (func_code) {
        case MODBUS_FC_READ_COILS: {
            if (req_len < 6) return -1;
            uint16_t start = (req[2] << 8) | req[3];
            uint16_t quantity = (req[4] << 8) | req[5];

            uint16_t coil_status = data_sim_get_do_status();
            int byte_count = (quantity + 7) / 8;
            resp[2] = byte_count;

            for (int i = 0; i < quantity && i < 16; i++) {
                int bit = start - 1 + i;
                if (bit >= 0 && bit < 12) {
                    if ((coil_status >> bit) & 1) {
                        resp[3 + i / 8] |= (1 << (i % 8));
                    }
                }
            }
            payload_len = 1 + byte_count;
            break;
        }

        case MODBUS_FC_READ_INPUT_REG: {
            if (req_len < 6) return -1;
            uint16_t start = (req[2] << 8) | req[3];
            uint16_t quantity = (req[4] << 8) | req[5];

            resp[2] = quantity * 2;
            int idx = 3;

            for (int i = 0; i < quantity; i++) {
                uint16_t addr = start + i;
                uint16_t value = 0;

                if (addr >= REG_POT_GAIN_BASE && addr < REG_POT_GAIN_BASE + 5) {
                    value = data_sim_get_pot_gain(addr - REG_POT_GAIN_BASE);
                } else if (addr >= REG_DI_LATCH_BASE && addr < REG_DI_LATCH_BASE + 22) {
                    value = (data_sim_get_di_latch() >> (addr - REG_DI_LATCH_BASE)) & 1;
                } else if (addr >= REG_RS485_LEN_BASE && addr < REG_RS485_LEN_BASE + 5) {
                    value = data_sim_get_rs485_len(addr - REG_RS485_LEN_BASE);
                } else if (addr >= REG_RS232_LEN_BASE && addr < REG_RS232_LEN_BASE + 3) {
                    value = data_sim_get_rs232_len(addr - REG_RS232_LEN_BASE);
                }

                resp[idx++] = (value >> 8) & 0xFF;
                resp[idx++] = value & 0xFF;
            }
            payload_len = 1 + quantity * 2;
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL: {
            if (req_len < 6) return -1;
            uint16_t addr = (req[2] << 8) | req[3];
            uint16_t value = (req[4] << 8) | req[5];

            if (addr >= 1 && addr <= 12) {
                data_sim_set_do(addr - 1, value != 0);
            }

            // Echo back
            memcpy(&resp[2], &req[2], 4);
            payload_len = 4;
            break;
        }

        default:
            // Exception
            resp[2] = MODBUS_EX_ILLEGAL_FUNCTION;
            payload_len = 1;
            func_code |= 0x80;
            break;
    }

    resp[0] = slave_addr;
    resp[1] = func_code;
    *resp_len = 2 + payload_len;

    return 0;
}

void* rt_rtu_thread(void *arg) {
    const char *dev = (const char*)arg;
    if (dev == NULL) {
        dev = RTU_DEV;
    }

    RT_LOG_INFO("RTU thread started on %s", dev);

    int fd = serial_open(dev, RTU_BAUD, 'N', 8, 1);
    if (fd < 0) {
        RT_LOG_ERROR("Failed to open serial port");
        return NULL;
    }

    RT_LOG_INFO("Serial port opened, RTU listening");

    uint8_t rx_buf[256];
    int rx_pos = 0;

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    uint64_t loop_count = 0;
    int64_t max_latency = 0;

    while (!g_quit) {
        uint64_t t1 = rt_get_time_us();

        // Update next period
        next_time.tv_sec += RTU_PERIOD_US / 1000000;
        next_time.tv_nsec += (RTU_PERIOD_US % 1000000) * 1000;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }

        // Read available bytes
        uint8_t byte;
        while (read(fd, &byte, 1) == 1) {
            rx_buf[rx_pos++] = byte;
            if (rx_pos >= sizeof(rx_buf)) {
                rx_pos = 0;
            }
        }

        // Process complete frame
        if (rx_pos >= 8) {
            uint16_t crc_received = (rx_buf[rx_pos-1] << 8) | rx_buf[rx_pos-2];
            uint16_t crc_calc = calc_crc16(rx_buf, rx_pos - 2);

            if (crc_received == crc_calc) {
                uint8_t resp[256];
                int resp_len;

                if (process_rtu_request(rx_buf, rx_pos - 2, resp, &resp_len) == 0) {
                    uint16_t crc = calc_crc16(resp, resp_len);
                    resp[resp_len++] = crc & 0xFF;
                    resp[resp_len++] = (crc >> 8) & 0xFF;
                    write(fd, resp, resp_len);
                }
            }
            rx_pos = 0;
        }

        // Sleep until next period
        uint64_t t2 = rt_get_time_us();
        int64_t latency = t2 - t1;

        if (latency > max_latency) {
            max_latency = latency;
        }

        loop_count++;
        if (loop_count % 1000 == 0) {
            RT_LOG_INFO("RTU loop: count=%llu, max_latency=%lldus",
                       loop_count, max_latency);
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    close(fd);
    RT_LOG_INFO("RTU thread exiting");
    return NULL;
}
