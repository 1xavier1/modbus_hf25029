# AD7606 驱动 Makefile
#
# 用法:
#   make           - 编译测试程序
#   make clean     - 清理
#   make install   - 安装到系统

CC = gcc
CFLAGS = -Wall -O2 -std=c99 -D_GNU_SOURCE
LDFLAGS = -lpthread -lrt

# 目标文件
AD7606_OBJS = ad7606.o
TEST_OBJS = ad7606_test.o

# 静态库
AD7606_LIB = libad7606.a

# 动态库
AD7606_SO = libad7606.so

# 默认目标
all: test

# 编译静态库
static_lib: $(AD7606_LIB)

$(AD7606_LIB): $(AD7606_OBJS)
	ar rcs $@ $^

# 编译动态库
shared_lib: $(AD7606_SO)

$(AD7606_SO): $(AD7606_OBJS)
	$(CC) -shared -fPIC -o $@ $^

# 编译测试程序 (链接静态库)
test: $(AD7606_LIB) ad7606_test
	@echo "Build complete: ad7606_test"

ad7606_test: $(TEST_OBJS) $(AD7606_LIB)
	$(CC) $(CFLAGS) -o $@ $(TEST_OBJS) $(AD7606_LIB) $(LDFLAGS)

# 编译测试程序 (链接动态库)
test_shared: $(AD7606_SO) ad7606_test_shared
	@echo "Build complete: ad7606_test_shared"

ad7606_test_shared: $(TEST_OBJS)
	$(CC) $(CFLAGS) -o $@ $(TEST_OBJS) -L. -lad7606 $(LDFLAGS)

# 编译目标文件
%.o: %.c ad7606.h
	$(CC) $(CFLAGS) -c $<

# 清理
clean:
	rm -f *.o $(AD7606_LIB) $(AD7606_SO) ad7606_test ad7606_test_shared

# 安装
PREFIX ?= /usr/local
install: test
	install -d $(PREFIX)/bin
	install -m 755 ad7606_test $(PREFIX)/bin/
	install -d $(PREFIX)/lib
	install -m 644 $(AD7606_LIB) $(PREFIX)/lib/
	install -d $(PREFIX)/include
	install -m 644 ad7606.h $(PREFIX)/include/
	ldconfig
	@echo "Installed to $(PREFIX)"

.PHONY: all static_lib shared_lib test test_shared clean install
