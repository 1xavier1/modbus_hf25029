/* Copyright 2024. All Rights Reserved. */
/* Modbus TCP Slave Implementation */

#include "modbus_tcp.h"
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
#include <netdb.h>
#include <errno.h>

#define TCP_PORT         502
#define TCP_BACKLOG      5
#define TCP_BUFFER_SIZE  1024

// ========================================================================
// Modbus TCP Request Processing
// ========================================================================

static int process_tcp_request(const uint8_t *req_data, int req_len,
                              uint8_t *resp_data, int *resp_len) {
    if (req_len < 9) {  // MBAP header is 7 bytes + at least function code
        return -1;
    }

    // Parse MBAP header
    uint16_t transaction_id = (req_data[0] << 8) | req_data[1];
    uint16_t protocol_id = (req_data[2] << 8) | req_data[3];
    uint16_t length = (req_data[4] << 8) | req_data[5];
    uint8_t unit_id = req_data[6];

    // Check slave address
    uint8_t slave_addr = hardware_get_slave_addr();
    if (unit_id != slave_addr && unit_id != 0) {
        return -1;  // Not for us
    }

    uint8_t func_code = req_data[7];
    uint8_t *req_payload = (uint8_t*)&req_data[7];
    int payload_len = req_len - 7;

    uint8_t resp_payload[256];
    int payload_resp_len = 0;

    int exception = 0;

    switch (func_code) {
        case MODBUS_FC_READ_COILS: {
            if (payload_len < 4) return -1;
            uint16_t start_addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t quantity = (req_payload[3] << 8) | req_payload[4];

            uint16_t coil_status = hardware_get_do_status();
            int byte_count = (quantity + 7) / 8;
            memset(resp_payload, 0, byte_count);

            for (int i = 0; i < quantity; i++) {
                int coil_addr = start_addr + i;  // PDU address already 0-based
                if (coil_addr >= 0 && coil_addr < 12) {
                    if ((coil_status >> coil_addr) & 1) {
                        resp_payload[i / 8] |= (1 << (i % 8));
                    }
                }
            }

            resp_payload[0] = byte_count;
            payload_resp_len = 1 + byte_count;
            break;
        }

        case MODBUS_FC_READ_DISCRETE_INPUT: {
            if (payload_len < 4) return -1;
            uint16_t start_addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t quantity = (req_payload[3] << 8) | req_payload[4];

            uint32_t di_status = hardware_get_di_status();
            int byte_count = (quantity + 7) / 8;
            memset(resp_payload, 0, byte_count);

            for (int i = 0; i < quantity; i++) {
                int di_addr = start_addr + i;  // PDU address already 0-based
                if (di_addr >= 0 && di_addr < 22) {
                    if ((di_status >> di_addr) & 1) {
                        resp_payload[i / 8] |= (1 << (i % 8));
                    }
                }
            }

            resp_payload[0] = byte_count;
            payload_resp_len = 1 + byte_count;
            break;
        }

        case MODBUS_FC_READ_HOLDING_REG: {
            if (payload_len < 4) return -1;
            uint16_t start_addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t quantity = (req_payload[3] << 8) | req_payload[4];

            resp_payload[0] = quantity * 2;
            payload_resp_len = 1;

            for (int i = 0; i < quantity; i++) {
                uint16_t addr = start_addr + i;
                uint16_t value = 0;

                if (addr >= REG_DEVICE_ADDR && addr <= REG_DEVICE_ADDR) {
                    value = hardware_get_slave_addr();
                } else if (addr >= REG_BAUDRATE && addr <= REG_BAUDRATE) {
                    value = hardware_get_baudrate();
                } else if (addr >= REG_IP_BASE && addr < REG_IP_BASE + REG_IP_COUNT) {
                    // IP address: 4 registers (PDU address 3-6)
                    int idx = addr - REG_IP_BASE;
                    uint8_t ip[4];
                    hardware_get_ip_addr(ip);
                    value = ip[idx];
                } else if (addr >= REG_NETMASK_BASE && addr < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
                    // Subnet mask: 4 registers (PDU address 7-10)
                    int idx = addr - REG_NETMASK_BASE;
                    uint8_t netmask[4];
                    hardware_get_netmask(netmask);
                    value = netmask[idx];
                } else if (addr >= REG_RS232_TX_LEN_BASE && addr < REG_RS232_TX_LEN_BASE + 3) {
                    int ch = addr - REG_RS232_TX_LEN_BASE;
                    value = hardware_get_rs232_tx_len(ch);
                } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
                    int ch = (addr - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
                    int idx = (addr - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
                    uint8_t baud, db, sb, par;
                    hardware_get_rs232_param(ch, &baud, &db, &sb, &par);
                    if (idx == 0) value = baud;
                    else if (idx == 1) value = db;
                    else if (idx == 2) value = sb;
                    else if (idx == 3) value = par;
                } else if (addr >= REG_RS232_CTRL_BASE && addr < REG_RS232_CTRL_BASE + REG_RS232_CTRL_COUNT) {
                    // RS232 control registers are write-only, return 0 on read
                    value = 0;
                } else if (addr >= REG_RS485_PARAM_BASE && addr < REG_RS485_PARAM_BASE + REG_RS485_PARAM_COUNT) {
                    int ch = (addr - REG_RS485_PARAM_BASE) / REG_RS485_PARAM_PER_CH;
                    int idx = (addr - REG_RS485_PARAM_BASE) % REG_RS485_PARAM_PER_CH;
                    uint8_t baud, db, sb, par;
                    hardware_get_rs485_param(ch, &baud, &db, &sb, &par);
                    if (idx == 0) value = baud;
                    else if (idx == 1) value = db;
                    else if (idx == 2) value = sb;
                    else if (idx == 3) value = par;
                } else if (addr >= REG_RS485_PROTO_BASE && addr < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
                    int ch = addr - REG_RS485_PROTO_BASE;
                    value = hardware_get_rs485_proto(ch);
                }

                resp_payload[payload_resp_len++] = (value >> 8) & 0xFF;
                resp_payload[payload_resp_len++] = value & 0xFF;
            }
            break;
        }

        case MODBUS_FC_READ_INPUT_REG: {
            if (payload_len < 4) return -1;
            uint16_t start_addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t quantity = (req_payload[3] << 8) | req_payload[4];

            resp_payload[0] = quantity * 2;
            payload_resp_len = 1;

            for (int i = 0; i < quantity; i++) {
                uint16_t addr = start_addr + i;
                uint16_t value = 0;

                if (addr >= REG_POT_GAIN_BASE && addr < REG_POT_GAIN_BASE + REG_POT_GAIN_COUNT) {
                    int ch = addr - REG_POT_GAIN_BASE;
                    value = hardware_get_pot_gain(ch);
                } else if (addr >= REG_DI_LATCH_BASE && addr < REG_DI_LATCH_BASE + REG_DI_LATCH_COUNT) {
                    int ch = addr - REG_DI_LATCH_BASE;
                    value = (hardware_get_di_latch() >> ch) & 1;
                } else if (addr >= REG_RS485_LEN_BASE && addr < REG_RS485_LEN_BASE + REG_RS485_LEN_COUNT) {
                    int ch = addr - REG_RS485_LEN_BASE;
                    value = hardware_get_rs485_len(ch);
                } else if (addr >= REG_RS485_DATA_BASE && addr < REG_RS485_DATA_BASE + REG_RS485_DATA_COUNT) {
                    int ch = (addr - REG_RS485_DATA_BASE) / REG_RS485_DATA_PER_CH;
                    int idx = (addr - REG_RS485_DATA_BASE) % REG_RS485_DATA_PER_CH;
                    uint8_t buf[10];
                    int len = hardware_get_rs485_data(ch, buf, sizeof(buf));
                    if (idx < len) {
                        value = buf[idx];
                    }
                } else if (addr >= REG_RS485_STAT_BASE && addr < REG_RS485_STAT_BASE + REG_RS485_STAT_COUNT) {
                    int ch = addr - REG_RS485_STAT_BASE;
                    value = hardware_get_rs485_stat(ch);
                } else if (addr >= REG_RS232_LEN_BASE && addr < REG_RS232_LEN_BASE + REG_RS232_LEN_COUNT) {
                    int ch = addr - REG_RS232_LEN_BASE;
                    value = hardware_get_rs232_len(ch);
                } else if (addr >= REG_RS232_STAT_BASE && addr < REG_RS232_STAT_BASE + REG_RS232_STAT_COUNT) {
                    int ch = addr - REG_RS232_STAT_BASE;
                    value = hardware_get_rs232_stat(ch);
                }

                resp_payload[payload_resp_len++] = (value >> 8) & 0xFF;
                resp_payload[payload_resp_len++] = value & 0xFF;
            }
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL: {
            if (payload_len < 4) return -1;
            uint16_t addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t value = (req_payload[3] << 8) | req_payload[4];

            if (addr >= 0 && addr <= 11) {  // PDU address 0-based
                int ch = addr;
                hardware_set_do(ch, value != 0);
            }

            // Echo back
            memcpy(resp_payload, req_payload, 4);
            payload_resp_len = 4;
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_REG: {
            if (payload_len < 4) return -1;
            uint16_t addr = (req_payload[1] << 8) | req_payload[2];
            uint16_t value = (req_payload[3] << 8) | req_payload[4];

            if (addr >= REG_DEVICE_ADDR && addr <= REG_DEVICE_ADDR) {
                if (value >= 1 && value <= 247) {
                    hardware_set_slave_addr(value);
                }
            } else if (addr >= REG_IP_BASE && addr < REG_IP_BASE + REG_IP_COUNT) {
                // IP address: 4 registers (PDU address 3-6)
                int idx = addr - REG_IP_BASE;
                uint8_t ip[4];
                hardware_get_ip_addr(ip);
                ip[idx] = (uint8_t)value;
                hardware_set_ip_addr(ip);
            } else if (addr >= REG_NETMASK_BASE && addr < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
                // Subnet mask: 4 registers (PDU address 7-10)
                int idx = addr - REG_NETMASK_BASE;
                uint8_t netmask[4];
                hardware_get_netmask(netmask);
                netmask[idx] = (uint8_t)value;
                hardware_set_netmask(netmask);
            } else if (addr >= REG_DI_CLEAR_BASE && addr < REG_DI_CLEAR_BASE + REG_DI_CLEAR_COUNT) {
                int ch = addr - REG_DI_CLEAR_BASE;
                if (value == 0x0001) {
                    hardware_clear_di_latch(ch);
                }
            } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
                int ch = (addr - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
                int idx = (addr - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
                uint8_t baud, db, sb, par;
                hardware_get_rs232_param(ch, &baud, &db, &sb, &par);
                if (idx == 0) baud = (uint8_t)value;
                else if (idx == 1) db = (uint8_t)value;
                else if (idx == 2) sb = (uint8_t)value;
                else if (idx == 3) par = (uint8_t)value;
                hardware_set_rs232_param(ch, baud, db, sb, par);
            } else if (addr >= REG_RS485_PROTO_BASE && addr < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
                int ch = addr - REG_RS485_PROTO_BASE;
                hardware_set_rs485_proto(ch, (uint8_t)value);
            } else if (addr >= REG_RS232_CTRL_BASE && addr < REG_RS232_CTRL_BASE + REG_RS232_CTRL_COUNT) {
                int ch = addr - REG_RS232_CTRL_BASE;
                if (value & 0x0001) {
                    // Start transmit
                }
                if (value & 0x0002) {
                    hardware_clear_rs232_rx(ch);
                }
                if (value & 0x0004) {
                    hardware_clear_rs232_tx(ch);
                }
            }

            // Echo back
            memcpy(resp_payload, req_payload, 4);
            payload_resp_len = 4;
            break;
        }

        default:
            exception = MODBUS_EX_ILLEGAL_FUNCTION;
            break;
    }

    // Build response
    resp_data[0] = (transaction_id >> 8) & 0xFF;
    resp_data[1] = transaction_id & 0xFF;
    resp_data[2] = (protocol_id >> 8) & 0xFF;
    resp_data[3] = protocol_id & 0xFF;

    if (exception != 0) {
        resp_data[4] = 0x00;
        resp_data[5] = 0x03;  // Length
        resp_data[6] = unit_id;
        resp_data[7] = func_code | 0x80;
        resp_data[8] = exception;
        *resp_len = 9;
    } else {
        resp_data[4] = ((payload_resp_len + 1) >> 8) & 0xFF;
        resp_data[5] = (payload_resp_len + 1) & 0xFF;
        resp_data[6] = unit_id;
        resp_data[7] = func_code;
        memcpy(&resp_data[8], resp_payload, payload_resp_len);
        *resp_len = 8 + payload_resp_len;
    }

    return 0;
}

// ========================================================================
// TCP Slave Thread
// ========================================================================

static void* tcp_connection_handler(void *arg) {
    int client_fd = *(int*)arg;
    free(arg);

    uint8_t rx_buf[TCP_BUFFER_SIZE];
    uint8_t tx_buf[TCP_BUFFER_SIZE];

    printf("[TCP] Client handler started\n");

    while (!g_quit) {
        fd_set rfds;
        struct timeval tv;

        FD_ZERO(&rfds);
        FD_SET(client_fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms timeout

        int ret = select(client_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        } else if (ret == 0) {
            continue;
        }

        int n = read(client_fd, rx_buf, sizeof(rx_buf));
        if (n <= 0) {
            break;
        }

        int tx_len = 0;
        if (process_tcp_request(rx_buf, n, tx_buf, &tx_len) == 0) {
            write(client_fd, tx_buf, tx_len);
        }
    }

    close(client_fd);
    printf("[TCP] Client handler exiting\n");
    return NULL;
}

void* tcp_slave_thread(void *arg) {
    (void)arg;

    printf("[TCP] Starting Modbus TCP slave on port %d\n", TCP_PORT);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("[TCP] socket");
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
        perror("[TCP] bind");
        close(server_fd);
        return NULL;
    }

    if (listen(server_fd, TCP_BACKLOG) < 0) {
        perror("[TCP] listen");
        close(server_fd);
        return NULL;
    }

    printf("[TCP] Listening on port %d\n", TCP_PORT);

    while (!g_quit) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            perror("[TCP] accept");
            continue;
        }

        // Disable Nagle for lower latency
        int nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        printf("[TCP] Client connected: %s:%d\n",
               inet_ntoa(client_addr.sin_addr),
               ntohs(client_addr.sin_port));

        // Spawn handler thread
        pthread_t handler_thread;
        int *client_fd_ptr = malloc(sizeof(int));
        *client_fd_ptr = client_fd;
        pthread_create(&handler_thread, NULL, tcp_connection_handler, client_fd_ptr);
        pthread_detach(handler_thread);
    }

    close(server_fd);
    printf("[TCP] Thread exiting\n");
    return NULL;
}
