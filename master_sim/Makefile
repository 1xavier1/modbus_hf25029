# Master Simulator Makefile

CC = gcc
CFLAGS = -g -Wall -O2 -I./include
LIBS = -lpthread
LDFLAGS =

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = bin/master_sim

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p bin
	$(CC) -o $@ $^ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o bin/$(TARGET)
