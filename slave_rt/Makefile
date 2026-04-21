# Slave-RT Makefile (Linux-RT Version)

CC = gcc
CFLAGS = -g -Wall -O2 -I../common/include -I./src
LIBS = -lpthread -lrt -lm
LDFLAGS = -lrt

COMMON_DIR = ../common
COMMON_LIB = $(COMMON_DIR)/libcommon.a

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = bin/modbus_slave_rt

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ) $(COMMON_LIB)
	@mkdir -p bin
	$(CC) -o $@ $^ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o bin/$(TARGET)

# Cross-compile for ARM with hard float
CROSS = arm-linux-gnueabihf-
CROSS_CC = $(CROSS)gcc
CROSS_CFLAGS = $(CFLAGS) -march=armv7-a -mfpu=neon -mfloat-abi=hard
CROSS_TARGET = bin/modbus_slave_rt_arm

cross:
	$(CROSS_CC) $(CROSS_CFLAGS) -o $(CROSS_TARGET) $(SRC) $(LDFLAGS) $(COMMON_LIB) $(LIBS)
