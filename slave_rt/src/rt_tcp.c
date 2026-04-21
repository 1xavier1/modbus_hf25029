/* Copyright 2024. All Rights Reserved. */
/* Real-time Modbus TCP Slave */

#include "rt_tcp.h"
#include "rt_utils.h"
#include "modbus_func.h"
#include "register_map.h"
#include "hardware.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#define TCP_PORT  502

// Process TCP request
static int process_tcp_request(const uint8_t *req, int req_len,
                              uint8_t *resp, int *resp_len) {
    if (req_len < 9) return -1;

    uint16_t txn_id = (req[0] << 8) | req[1];
    uint8_t unit_id = req[6];
    uint8_t func_code = req[7];

    if (unit_id != hardware_get_slave_addr() && unit_id != 0) {
        return -1;
    }

    int payload_len = 0;

    switch (func_code) {
        case MODBUS_FC_READ_HOLDING_REG: {
            uint16_t start = (req[8] << 8) | req[9];
            uint16_t quantity = (req[10] << 8) | req[11];

            resp[8] = quantity * 2;
            int idx = 9;

            for (int i = 0; i < quantity; i++) {
                uint16_t addr = start + i;
                uint16_t value = 0;

                if (addr >= REG_DEVICE_ADDR && addr <= REG_DEVICE_ADDR) {
                    value = hardware_get_slave_addr();
                } else if (addr >= REG_BAUDRATE && addr <= REG_BAUDRATE) {
                    value = hardware_get_baudrate();
                } else if (addr >= REG_PARITY && addr <= REG_PARITY) {
                    value = hardware_get_parity();
                } else if (addr >= REG_IP_BASE && addr < REG_IP_BASE + REG_IP_COUNT) {
                    int idx_ip = addr - REG_IP_BASE;
                    uint8_t ip[4];
                    hardware_get_ip_addr(ip);
                    value = ip[idx_ip];
                } else if (addr >= REG_NETMASK_BASE && addr < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
                    int idx_ip = addr - REG_NETMASK_BASE;
                    uint8_t netmask[4];
                    hardware_get_netmask(netmask);
                    value = netmask[idx_ip];
                } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
                    int ch = (addr - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
                    int idx_param = (addr - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
                    uint8_t baud, db, sb, par;
                    hardware_get_rs232_param(ch, &baud, &db, &sb, &par);
                    if (idx_param == 0) value = baud;
                    else if (idx_param == 1) value = db;
                    else if (idx_param == 2) value = sb;
                    else if (idx_param == 3) value = par;
                } else if (addr >= REG_RS485_CTRL_BASE && addr < REG_RS485_CTRL_BASE + REG_RS485_CTRL_COUNT) {
                    int ch = addr - REG_RS485_CTRL_BASE;
                    value = hardware_get_rs485_ctrl(ch);
                } else if (addr >= REG_RS485_PARAM_BASE && addr < REG_RS485_PARAM_BASE + REG_RS485_PARAM_COUNT) {
                    int ch = (addr - REG_RS485_PARAM_BASE) / REG_RS485_PARAM_PER_CH;
                    int idx_param = (addr - REG_RS485_PARAM_BASE) % REG_RS485_PARAM_PER_CH;
                    uint8_t baud, db, sb, par;
                    hardware_get_rs485_param(ch, &baud, &db, &sb, &par);
                    if (idx_param == 0) value = baud;
                    else if (idx_param == 1) value = db;
                    else if (idx_param == 2) value = sb;
                    else if (idx_param == 3) value = par;
                } else if (addr >= REG_RS485_PROTO_BASE && addr < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
                    int ch = addr - REG_RS485_PROTO_BASE;
                    value = hardware_get_rs485_proto(ch);
                }

                resp[idx++] = (value >> 8) & 0xFF;
                resp[idx++] = value & 0xFF;
            }
            payload_len = 1 + quantity * 2;
            break;
        }

        case MODBUS_FC_READ_INPUT_REG: {
            uint16_t start = (req[8] << 8) | req[9];
            uint16_t quantity = (req[10] << 8) | req[11];

            resp[8] = quantity * 2;
            int idx = 9;

            for (int i = 0; i < quantity; i++) {
                uint16_t addr = start + i;
                uint16_t value = 0;

                if (addr >= REG_POT_GAIN_BASE && addr < REG_POT_GAIN_BASE + REG_POT_GAIN_COUNT) {
                    value = hardware_get_pot_gain(addr - REG_POT_GAIN_BASE);
                } else if (addr >= REG_DI_LATCH_BASE && addr < REG_DI_LATCH_BASE + REG_DI_LATCH_COUNT) {
                    value = (hardware_get_di_latch() >> (addr - REG_DI_LATCH_BASE)) & 1;
                }

                resp[idx++] = (value >> 8) & 0xFF;
                resp[idx++] = value & 0xFF;
            }
            payload_len = 1 + quantity * 2;
            break;
        }

        case MODBUS_FC_READ_COILS: {
            uint16_t start = (req[8] << 8) | req[9];
            uint16_t quantity = (req[10] << 8) | req[11];

            uint16_t coil = hardware_get_do_status();
            int byte_count = (quantity + 7) / 8;
            resp[8] = byte_count;

            for (int i = 0; i < quantity && i < 16; i++) {
                int bit = start + i;
                if (bit >= 0 && bit < 12) {
                    if ((coil >> bit) & 1) {
                        resp[9 + i / 8] |= (1 << (i % 8));
                    }
                }
            }
            payload_len = 1 + byte_count;
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL: {
            uint16_t addr = (req[8] << 8) | req[9];
            uint16_t value = (req[10] << 8) | req[11];

            if (addr >= 0 && addr <= 11) {
                hardware_set_do(addr, value != 0);
            }

            memcpy(&resp[8], &req[8], 4);
            payload_len = 4;
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_REG: {
            uint16_t addr = (req[8] << 8) | req[9];
            uint16_t value = (req[10] << 8) | req[11];

            if (addr == REG_DEVICE_ADDR) {
                if (value >= 1 && value <= 247) {
                    hardware_set_slave_addr((uint8_t)value);
                }
            } else if (addr == REG_BAUDRATE) {
                hardware_set_baudrate((uint8_t)value);
            } else if (addr == REG_PARITY) {
                hardware_set_parity((uint8_t)value);
            } else if (addr >= REG_IP_BASE && addr < REG_IP_BASE + REG_IP_COUNT) {
                int idx_ip = addr - REG_IP_BASE;
                uint8_t ip[4];
                hardware_get_ip_addr(ip);
                ip[idx_ip] = (uint8_t)value;
                hardware_set_ip_addr(ip);
            } else if (addr >= REG_NETMASK_BASE && addr < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
                int idx_ip = addr - REG_NETMASK_BASE;
                uint8_t netmask[4];
                hardware_get_netmask(netmask);
                netmask[idx_ip] = (uint8_t)value;
                hardware_set_netmask(netmask);
            } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
                int ch = (addr - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
                int idx_param = (addr - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
                uint8_t baud, db, sb, par;
                hardware_get_rs232_param(ch, &baud, &db, &sb, &par);
                if (idx_param == 0) baud = (uint8_t)value;
                else if (idx_param == 1) db = (uint8_t)value;
                else if (idx_param == 2) sb = (uint8_t)value;
                else if (idx_param == 3) par = (uint8_t)value;
                hardware_set_rs232_param(ch, baud, db, sb, par);
            } else if (addr >= REG_RS485_CTRL_BASE && addr < REG_RS485_CTRL_BASE + REG_RS485_CTRL_COUNT) {
                int ch = addr - REG_RS485_CTRL_BASE;
                hardware_set_rs485_ctrl(ch, (uint8_t)value);
            } else if (addr >= REG_RS485_PARAM_BASE && addr < REG_RS485_PARAM_BASE + REG_RS485_PARAM_COUNT) {
                int ch = (addr - REG_RS485_PARAM_BASE) / REG_RS485_PARAM_PER_CH;
                int idx_param = (addr - REG_RS485_PARAM_BASE) % REG_RS485_PARAM_PER_CH;
                uint8_t baud, db, sb, par;
                hardware_get_rs485_param(ch, &baud, &db, &sb, &par);
                if (idx_param == 0) baud = (uint8_t)value;
                else if (idx_param == 1) db = (uint8_t)value;
                else if (idx_param == 2) sb = (uint8_t)value;
                else if (idx_param == 3) par = (uint8_t)value;
                hardware_set_rs485_param(ch, baud, db, sb, par);
            } else if (addr >= REG_RS485_PROTO_BASE && addr < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
                int ch = addr - REG_RS485_PROTO_BASE;
                hardware_set_rs485_proto(ch, (uint8_t)value);
            } else if (addr == REG_SAVE_PARAM) {
                if (value == SAVE_PARAM_VALUE) {
                    hardware_persist_flush();
                }
            }

            memcpy(&resp[8], &req[8], 4);
            payload_len = 4;
            break;
        }

        default:
            resp[8] = MODBUS_EX_ILLEGAL_FUNCTION;
            payload_len = 1;
            func_code |= 0x80;
            break;
    }

    // Build MBAP header
    resp[0] = (txn_id >> 8) & 0xFF;
    resp[1] = txn_id & 0xFF;
    resp[2] = 0;
    resp[3] = 0;
    resp[4] = ((payload_len + 1) >> 8) & 0xFF;
    resp[5] = (payload_len + 1) & 0xFF;
    resp[6] = unit_id;
    resp[7] = func_code;

    *resp_len = 8 + payload_len;

    return 0;
}

void* rt_tcp_thread(void *arg) {
    (void)arg;

    RT_LOG_INFO("TCP thread started on port %d", TCP_PORT);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        return NULL;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        close(server_fd);
        return NULL;
    }

    listen(server_fd, 5);

    // Make socket non-blocking for accept
    fd_set read_fds;
    struct timeval tv;

    while (!g_quit) {
        FD_ZERO(&read_fds);
        FD_SET(server_fd, &read_fds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms

        int ret = select(server_fd + 1, &read_fds, NULL, NULL, &tv);
        if (ret <= 0) continue;

        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);

        if (client_fd < 0) continue;

        // Set TCP_NODELAY for lower latency
        int nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        uint8_t rx_buf[1024];
        uint8_t tx_buf[1024];

        while (!g_quit) {
            ret = read(client_fd, rx_buf, sizeof(rx_buf));
            if (ret <= 0) break;

            int tx_len = 0;
            if (process_tcp_request(rx_buf, ret, tx_buf, &tx_len) == 0) {
                write(client_fd, tx_buf, tx_len);
            }
        }

        close(client_fd);
    }

    close(server_fd);
    RT_LOG_INFO("TCP thread exiting");
    return NULL;
}
