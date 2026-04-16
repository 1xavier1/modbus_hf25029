/* Copyright 2024. All Rights Reserved. */
/* Serial Port Implementation */

#include "serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

int serial_open(const char *dev, int baudrate, char parity, int databits, int stopbits) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("[SERIAL] open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("[SERIAL] tcgetattr");
        close(fd);
        return -1;
    }

    // Set baudrate
    speed_t speed;
    switch (baudrate) {
        case 1200:   speed = B1200;   break;
        case 2400:   speed = B2400;   break;
        case 4800:   speed = B4800;   break;
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Set data bits
    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 7: tty.c_cflag |= CS7; break;
        case 8:
        default: tty.c_cflag |= CS8; break;
    }

    // Set parity
    switch (parity) {
        case 'o':
        case 'O':
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            tty.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            tty.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'n':
        case 'N':
        default:
            tty.c_cflag &= ~PARENB;
            break;
    }

    // Set stop bits
    if (stopbits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        tty.c_cflag &= ~CSTOPB;
    }

    // Enable receiver and set local mode
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disable hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Output processing
    tty.c_oflag &= ~OPOST;

    // No echo, no canonical mode
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Non-blocking read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;  // 1 second timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[SERIAL] tcsetattr");
        close(fd);
        return -1;
    }

    // Flush buffers
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void serial_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

int serial_read(int fd, uint8_t *buf, int len, int timeout_ms) {
    fd_set rfds;
    struct timeval tv;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (ret < 0) {
        return -1;
    } else if (ret == 0) {
        return 0;  // Timeout
    }

    return read(fd, buf, len);
}

int serial_write(int fd, const uint8_t *buf, int len) {
    return write(fd, buf, len);
}

void serial_flush(int fd) {
    tcflush(fd, TCIOFLUSH);
}
