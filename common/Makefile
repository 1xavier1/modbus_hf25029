# Common module Makefile

CC = gcc
CFLAGS = -g -Wall -O2 -I./include
AR = ar
ARFLAGS = rcs

COMMON_DIR = .

# Source files
SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET_LIB = libcommon.a

.PHONY: all clean

all: $(TARGET_LIB)

$(TARGET_LIB): $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(TARGET_LIB)

install:
	cp -r include/* ../slave/include/
	cp -r include/* ../slave_rt/include/
	cp -r include/* ../master_sim/include/
	cp $(TARGET_LIB) ../slave/lib/
	cp $(TARGET_LIB) ../slave_rt/lib/
	cp $(TARGET_LIB) ../master_sim/lib/
