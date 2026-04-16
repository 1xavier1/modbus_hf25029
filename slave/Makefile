# Slave Makefile

CC = gcc
CFLAGS = -g -Wall -O2 -I../common/include
LIBS = -lpthread -lrt -lm
LDFLAGS =

COMMON_DIR = ../common
COMMON_LIB = $(COMMON_DIR)/libcommon.a

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = bin/modbus_slave

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ) $(COMMON_LIB)
	@mkdir -p bin
	$(CC) -o $@ $^ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o bin/$(TARGET)

# Cross-compile for ARM (if needed)
CROSS = arm-linux-gnueabihf-
CROSS_CC = $(CROSS)gcc
CROSS_CFLAGS = $(CFLAGS) -march=armv7-a -mfpu=neon -mfloat-abi=hard
CROSS_TARGET = bin/modbus_slave_arm

cross: $(SRC)
	$(CROSS_CC) $(CROSS_CFLAGS) -o $(CROSS_TARGET) $^ $(LIBS)
