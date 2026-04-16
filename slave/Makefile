# Slave Makefile

CC ?= gcc
CFLAGS = -g -Wall -O2 -I../common/include
LDFLAGS =

COMMON_DIR = ../common
COMMON_LIB = $(COMMON_DIR)/libcommon.a

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = bin/modbus_slave

# Check if cross-compiling
IS_CROSS = $(findstring arm,$(CC))

# Libraries - json-c only needed for ARM
LIBS_PC = -lpthread -lrt -lm
LIBS_ARM = -lpthread -lrt -lm -ljson-c

ifeq ($(IS_CROSS),arm)
LIBS = $(LIBS_ARM)
CFLAGS += -march=armv7-a -mfpu=neon -mfloat-abi=hard
else
LIBS = $(LIBS_PC)
endif

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ) $(COMMON_LIB)
	@mkdir -p bin
	$(CC) -o $@ $^ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o bin/$(TARGET)
