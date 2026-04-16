/* Copyright 2024. All Rights Reserved. */
/* Modbus TCP Master Simulator */

#include "master_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

// Modbus function codes
#define FC_READ_COILS           0x01
#define FC_READ_DISCRETE_INPUT  0x02
#define FC_READ_HOLDING_REG     0x03
#define FC_READ_INPUT_REG       0x04
#define FC_WRITE_SINGLE_COIL    0x05
#define FC_WRITE_SINGLE_REG     0x06

static uint16_t g_txn_id = 0;

// Transaction ID counter
static uint16_t next_txn_id(void) {
    return ++g_txn_id;
}

// Get current time in milliseconds
static uint64_t get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// Calculate latency in ms
static double latency_ms(uint64_t start, uint64_t end) {
    return (end - start) / 1000.0;
}

static void print_timestamp(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm *tm = localtime(&ts.tv_sec);
    printf("%02d:%02d:%02d.%03ld",
           tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
}

// TCP send and receive
static int tcp_transaction(int sockfd, uint8_t *tx_buf, int tx_len,
                          uint8_t *rx_buf, int rx_max) {
    uint64_t t1 = get_time_ms();

    if (send(sockfd, tx_buf, tx_len, 0) != tx_len) {
        return -1;
    }

    // Wait for response
    int rx_len = 0;
    int timeout = 1000;  // 1 second timeout

    while (rx_len < rx_max) {
        int n = recv(sockfd, rx_buf + rx_len, rx_max - rx_len, 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return -1;
        } else if (n == 0) {
            return -1;  // Connection closed
        }
        rx_len += n;

        // Check if we have complete response
        if (rx_len >= 9) {
            uint16_t resp_len = (rx_buf[4] << 8) | rx_buf[5];
            if (rx_len >= 6 + resp_len) {
                break;
            }
        }

        // Check timeout
        uint64_t t2 = get_time_ms();
        if (t2 - t1 > timeout) {
            return -1;
        }
    }

    return rx_len;
}

// Test: Read Input Registers (30001-30027)
static int test_read_input_regs(int sockfd, int slave_addr) {
    uint8_t tx[12];
    uint8_t rx[256];

    uint16_t txn = next_txn_id();

    // MBAP header
    tx[0] = (txn >> 8) & 0xFF;
    tx[1] = txn & 0xFF;
    tx[2] = 0;
    tx[3] = 0;
    tx[4] = 0;
    tx[5] = 6;  // Length
    tx[6] = slave_addr;
    tx[7] = FC_READ_INPUT_REG;
    tx[8] = (30001 >> 8) & 0xFF;
    tx[9] = 30001 & 0xFF;
    tx[10] = 0;
    tx[11] = 27;  // 27 registers

    int rx_len = tcp_transaction(sockfd, tx, 12, rx, sizeof(rx));
    if (rx_len < 0) {
        printf("[ERROR] Read input regs failed\n");
        return -1;
    }

    if (rx[7] & 0x80) {
        printf("[ERROR] Exception: %d\n", rx[8]);
        return -1;
    }

    int byte_count = rx[8];
    printf("[%s] Read Input Reg (30001-30027): %d bytes\n",
           "", byte_count);

    // Parse data
    printf("  POT: ");
    for (int i = 0; i < 5; i++) {
        uint16_t val = (rx[9 + i*2] << 8) | rx[10 + i*2];
        printf("%d=%d ", i+1, val);
    }
    printf("\n");

    return 0;
}

// Test: Read Coils (00001-00012)
static int test_read_coils(int sockfd, int slave_addr) {
    uint8_t tx[12];
    uint8_t rx[256];

    uint16_t txn = next_txn_id();

    tx[0] = (txn >> 8) & 0xFF;
    tx[1] = txn & 0xFF;
    tx[2] = 0;
    tx[3] = 0;
    tx[4] = 0;
    tx[5] = 6;
    tx[6] = slave_addr;
    tx[7] = FC_READ_COILS;
    tx[8] = 0;
    tx[9] = 1;   // Start addr 1
    tx[10] = 0;
    tx[11] = 12;  // 12 coils

    int rx_len = tcp_transaction(sockfd, tx, 12, rx, sizeof(rx));
    if (rx_len < 0) {
        printf("[ERROR] Read coils failed\n");
        return -1;
    }

    int byte_count = rx[8];
    uint16_t coil_status = (rx[9] << 8) | rx[10];

    printf("[%s] Read Coils (00001-00012): byte_count=%d, status=0x%03X\n",
           "", byte_count, coil_status);

    printf("  DO: ");
    for (int i = 0; i < 12; i++) {
        printf("%d=%d ", i+1, (coil_status >> i) & 1);
    }
    printf("\n");

    return 0;
}

// Test: Write Single Coil
static int test_write_coil(int sockfd, int slave_addr, int coil_addr, int value) {
    uint8_t tx[12];
    uint8_t rx[256];

    uint16_t txn = next_txn_id();

    tx[0] = (txn >> 8) & 0xFF;
    tx[1] = txn & 0xFF;
    tx[2] = 0;
    tx[3] = 0;
    tx[4] = 0;
    tx[5] = 6;
    tx[6] = slave_addr;
    tx[7] = FC_WRITE_SINGLE_COIL;
    tx[8] = 0;
    tx[9] = coil_addr;
    tx[10] = value ? 0xFF : 0x00;
    tx[11] = 0x00;

    int rx_len = tcp_transaction(sockfd, tx, 12, rx, sizeof(rx));
    if (rx_len < 0) {
        printf("[ERROR] Write coil failed\n");
        return -1;
    }

    printf("[%s] Write Coil %d=%d: OK\n", "", coil_addr, value);

    return 0;
}

// Test: Read Discrete Inputs (10001-10022)
static int test_read_discrete_input(int sockfd, int slave_addr) {
    uint8_t tx[12];
    uint8_t rx[256];

    uint16_t txn = next_txn_id();

    tx[0] = (txn >> 8) & 0xFF;
    tx[1] = txn & 0xFF;
    tx[2] = 0;
    tx[3] = 0;
    tx[4] = 0;
    tx[5] = 6;
    tx[6] = slave_addr;
    tx[7] = FC_READ_DISCRETE_INPUT;
    tx[8] = (10001 >> 8) & 0xFF;
    tx[9] = 10001 & 0xFF;
    tx[10] = 0;
    tx[11] = 22;  // 22 inputs

    int rx_len = tcp_transaction(sockfd, tx, 12, rx, sizeof(rx));
    if (rx_len < 0) {
        printf("[ERROR] Read discrete input failed\n");
        return -1;
    }

    uint16_t di_status = (rx[9] << 8) | rx[10];
    printf("[%s] Read Discrete Input (10001-10022): status=0x%06X\n",
           "", di_status);

    printf("  DI: ");
    for (int i = 0; i < 22; i++) {
        printf("%02d=%d ", i+1, (di_status >> i) & 1);
    }
    printf("\n");

    return 0;
}

void tcp_master_run(const char *host, int port, int slave_addr,
                   int interval_ms, int count, int verbose) {
    (void)verbose;

    printf("[TCP] Connecting to %s:%d...\n", host, port);

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("[TCP] socket");
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, host, &server_addr.sin_addr) <= 0) {
        printf("[TCP] Invalid address: %s\n", host);
        close(sockfd);
        return;
    }

    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("[TCP] connect");
        close(sockfd);
        return;
    }

    // Disable Nagle
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    printf("[TCP] Connected!\n");

    int test_num = 0;
    uint64_t start_time = get_time_ms();
    double total_latency = 0;
    double max_latency = 0;
    double min_latency = 999999;

    while (!g_quit) {
        uint64_t t1 = get_time_ms();

        print_timestamp();
        printf("\n");

        // Run tests
        test_read_input_regs(sockfd, slave_addr);
        test_read_coils(sockfd, slave_addr);
        test_read_discrete_input(sockfd, slave_addr);
        test_write_coil(sockfd, slave_addr, (test_num % 12) + 1, test_num % 2);

        uint64_t t2 = get_time_ms();
        double lat = latency_ms(t1, t2);
        total_latency += lat;
        if (lat > max_latency) max_latency = lat;
        if (lat < min_latency) min_latency = lat;

        test_num++;

        if (count > 0 && test_num >= count) {
            break;
        }

        // Wait for next interval
        usleep(interval_ms * 1000);
    }

    close(sockfd);

    printf("\n===========================================\n");
    printf("  Test Summary\n");
    printf("===========================================\n");
    printf("Tests:    %d\n", test_num);
    printf("Duration: %.2fs\n", (get_time_ms() - start_time) / 1000.0);
    printf("Latency: avg=%.2fms, min=%.2fms, max=%.2fms\n",
           total_latency / test_num, min_latency, max_latency);
    printf("===========================================\n");
}
