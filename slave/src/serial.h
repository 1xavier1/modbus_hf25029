/* Serial Port Header */

#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

// Open serial port
int serial_open(const char *dev, int baudrate, char parity, int databits, int stopbits);

// Close serial port
void serial_close(int fd);

// Read from serial
int serial_read(int fd, uint8_t *buf, int len, int timeout_ms);

// Write to serial
int serial_write(int fd, const uint8_t *buf, int len);

// Flush buffers
void serial_flush(int fd);

#endif /* SERIAL_H */
