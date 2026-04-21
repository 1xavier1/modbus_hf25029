# Common module Makefile
#
# 支持 AD7606 驱动

CC = gcc
CFLAGS = -g -Wall -O2 -I./include
AR = ar
ARFLAGS = rcs

COMMON_DIR = .

# Source files (排除 ad7606 目录)
SRC = $(wildcard src/*.c) $(wildcard src/ad7606/*.c)
# 排除 modbus_ad7606.c 因为它需要 modbus 环境
SRC := $(filter-out src/ad7606/modbus_ad7606.c,$(SRC))
OBJ = $(SRC:.c=.o)

TARGET_LIB = libcommon.a

.PHONY: all clean ad7606 ad7606_test

all: $(TARGET_LIB) ad7606

# 编译 common 库
$(TARGET_LIB): $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# 编译 AD7606 驱动
ad7606:
	$(MAKE) -C src/ad7606

# 编译 AD7606 测试程序
ad7606_test:
	$(MAKE) -C src/ad7606 test

# 清理
clean:
	rm -f $(OBJ) $(TARGET_LIB)
	$(MAKE) -C src/ad7606 clean

# 安装
install: install_common install_ad7606

install_common: $(TARGET_LIB)
	cp -r include/* ../slave/include/
	cp -r include/* ../slave_rt/include/
	cp -r include/* ../master_sim/include/
	cp $(TARGET_LIB) ../slave/lib/
	cp $(TARGET_LIB) ../slave_rt/lib/
	cp $(TARGET_LIB) ../master_sim/lib/

install_ad7606:
	$(MAKE) -C src/ad7606 install
