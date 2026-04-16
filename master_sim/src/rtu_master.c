/* Copyright 2024. All Rights Reserved. */
/* Modbus RTU Master Simulator */

#include "master_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
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

// Calculate CRC16
static uint16_t calc_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static int open_serial(const char *dev, int baudrate) {
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty;
    tcgetattr(fd, &tty);

    speed_t speed;
    switch (baudrate) {
        case 1200:   speed = B1200; break;
        case 2400:   speed = B2400; break;
        case 4800:   speed = B4800; break;
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = speed | CS8 | CLOCAL | CREAD |PARENB | PARODD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

static uint64_t get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void print_timestamp(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm *tm = localtime(&ts.tv_sec);
    printf("%02d:%02d:%02d.%03ld",
           tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
}

static int serial_transaction(int fd, uint8_t *tx, int tx_len,
                              uint8_t *rx, int rx_max, int timeout_ms) {
    // Add CRC to TX
    uint16_t crc = calc_crc16(tx, tx_len);
    tx[tx_len++] = crc & 0xFF;
    tx[tx_len++] = (crc >> 8) & 0xFF;

    uint64_t t1 = get_time_ms();

    // Send
    if (write(fd, tx, tx_len) != tx_len) {
        return -1;
    }

    // Receive
    int rx_len = 0;
    while (rx_len < rx_max) {
        uint8_t byte;
        int n = read(fd, &byte, 1);
        if (n > 0) {
            rx[rx_len++] = byte;

            // Check for complete frame (minimum 5 bytes + CRC)
            if (rx_len >= 5) {
                uint16_t len = rx[2] << 8 | rx[3];
                if (rx_len >= 6 + len) {
                    break;
                }
            }
        } else {
            usleep(1000);
        }

        uint64_t t2 = get_time_ms();
        if (t2 - t1 > timeout_ms) {
            return -1;
        }
    }

    // Verify CRC
    if (rx_len >= 2) {
        uint16_t crc_received = (rx[rx_len-1] << 8) | rx[rx_len-2];
        uint16_t crc_calc = calc_crc16(rx, rx_len - 2);
        if (crc_received != crc_calc) {
            printf("[RTU-WARN] CRC mismatch: got 0x%04X, expected 0x%04X\n",
                   crc_received, crc_calc);
        }
    }

    return rx_len;
}

static int test_read_input_regs(int fd, int slave_addr) {
    uint8_t tx[6];
    uint8_t rx[256];

    tx[0] = slave_addr;
    tx[1] = FC_READ_INPUT_REG;
    tx[2] = (30001 >> 8) & 0xFF;
    tx[3] = 30001 & 0xFF;
    tx[4] = 0;
    tx[5] = 27;

    int rx_len = serial_transaction(fd, tx, 6, rx, sizeof(rx), 1000);
    if (rx_len < 0) {
        printf("[ERROR] Read input regs failed\n");
        return -1;
    }

    printf("[%s] Read Input Reg (30001-30027): byte_count=%d\n",
           "", rx[2]);

    printf("  POT: ");
    for (int i = 0; i < 5; i++) {
        uint16_t val = (rx[3 + i*2] << 8) | rx[4 + i*2];
        printf("%d=%d ", i+1, val);
    }
    printf("\n");

    return 0;
}

static int test_write_coil(int fd, int slave_addr, int coil_addr, int value) {
    uint8_t tx[6];
    uint8_t rx[256];

    tx[0] = slave_addr;
    tx[1] = FC_WRITE_SINGLE_COIL;
    tx[2] = 0;
    tx[3] = coil_addr;
    tx[4] = value ? 0xFF : 0x00;
    tx[5] = 0x00;

    int rx_len = serial_transaction(fd, tx, 6, rx, sizeof(rx), 1000);
    if (rx_len < 0) {
        printf("[ERROR] Write coil failed\n");
        return -1;
    }

    printf("[%s] Write Coil %d=%d: OK\n", "", coil_addr, value);

    return 0;
}

void rtu_master_run(const char *device, int baudrate, int slave_addr,
                   int interval_ms, int count, int verbose) {
    (void)verbose;

    printf("[RTU] Opening %s @ %d...\n", device, baudrate);

    int fd = open_serial(device, baudrate);
    if (fd < 0) {
        perror("[RTU] open");
        return;
    }

    printf("[RTU] Opened!\n");

    int test_num = 0;
    uint64_t start_time = get_time_ms();
    double total_latency = 0;
    double max_latency = 0;
    double min_latency = 999999;

    while (!g_quit) {
        uint64_t t1 = get_time_ms();

        print_timestamp();
        printf("\n");

        test_read_input_regs(fd, slave_addr);
        test_write_coil(fd, slave_addr, (test_num % 12) + 1, test_num % 2);

        uint64_t t2 = get_time_ms();
        double lat = t2 - t1;
        total_latency += lat;
        if (lat > max_latency) max_latency = lat;
        if (lat < min_latency) min_latency = lat;

        test_num++;

        if (count > 0 && test_num >= count) {
            break;
        }

        usleep(interval_ms * 1000);
    }

    close(fd);

    printf("\n===========================================\n");
    printf("  Test Summary\n");
    printf("===========================================\n");
    printf("Tests:    %d\n", test_num);
    printf("Duration: %.2fs\n", (get_time_ms() - start_time) / 1000.0);
    printf("Latency: avg=%.2fms, min=%.2fms, max=%.2fms\n",
           total_latency / test_num, min_latency, max_latency);
    printf("===========================================\n");
}
