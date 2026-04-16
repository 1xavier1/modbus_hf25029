# HF25029-CDP Modbus Slave 实现

## 概述

本项目为 TLT153 (Linux / Linux-RT) 实现 HF25029-CDP 信号采集处理模块的 Modbus Slave 功能。

### 主要特性

- **Modbus RTU 从站**: RS-485 接口，115200 8N1
- **Modbus TCP 从站**: Ethernet 接口，端口 502
- **支持功能码**: 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F, 0x10
- **数据模拟器**: 无需真实硬件即可测试
- **两种实现**: Linux 标准版 和 Linux-RT 实时版

## 目录结构

```
modbus_hf25029/
├── common/                      # 共用模块
│   ├── include/
│   │   ├── register_map.h       # 寄存器地址定义
│   │   ├── data_sim.h          # 数据模拟器接口
│   │   └── modbus_func.h       # 公共定义
│   └── src/
│       ├── data_sim.c           # 数据模拟器实现
│       └── Makefile
│
├── slave/                       # Linux 标准版从站
│   ├── include/
│   │   ├── gpio.h
│   │   ├── serial.h
│   │   ├── modbus_rtu.h
│   │   └── modbus_tcp.h
│   └── src/
│       ├── main.c
│       ├── gpio.c
│       ├── serial.c
│       ├── modbus_rtu.c
│       ├── modbus_tcp.c
│       └── Makefile
│
├── slave_rt/                    # Linux-RT 实时版从站
│   ├── include/
│   │   ├── rt_utils.h
│   │   ├── rt_gpio.h
│   │   ├── rt_rtu.h
│   │   ├── rt_tcp.h
│   │   └── rt_sync.h
│   └── src/
│       ├── main_rt.c
│       ├── rt_utils.c
│       ├── rt_gpio.c
│       ├── rt_rtu.c
│       ├── rt_tcp.c
│       ├── rt_sync.c
│       └── Makefile
│
└── master_sim/                  # 模拟主站 (测试用)
    ├── include/
    │   └── master_sim.h
    └── src/
        ├── main.c
        ├── tcp_master.c
        ├── rtu_master.c
        └── Makefile
```

## 编译指南

### 编译环境

- **PC 开发环境**: Linux (Ubuntu/Debian)
- **目标板**: TLT153 (ARM Cortex-A)
- **交叉编译器**: `arm-linux-gnueabihf-gcc`

### 1. 编译 Linux 标准版

```bash
cd modbus_hf25029/slave
make
```

输出: `slave/bin/modbus_slave`

### 2. 编译 Linux-RT 实时版

```bash
cd modbus_hf25029/slave_rt
make
```

输出: `slave_rt/bin/modbus_slave_rt`

### 3. 编译模拟主站 (PC上运行)

```bash
cd modbus_hf25029/master_sim
make
```

输出: `master_sim/bin/master_sim`

### 4. 交叉编译 (在 PC 上编译目标板程序)

```bash
# 设置交叉编译器
export CROSS=arm-linux-gnueabihf-
export CC=${CROSS}gcc

# 编译 slave
cd modbus_hf25029/slave
make CROSS=${CROSS} CC=${CC}

# 编译 slave_rt
cd modbus_hf25029/slave_rt
make CROSS=${CROSS} CC=${CC}
```

## 运行指南

### 1. Linux 标准版从站

```bash
# 启用 RTU + TCP 双模式
./slave/bin/modbus_slave -m both

# 仅启用 RTU
./slave/bin/modbus_slave -m rtu -d /dev/ttyAS2 -b 115200

# 仅启用 TCP
./slave/bin/modbus_slave -m tcp -v

# 查看帮助
./slave/bin/modbus_slave -h
```

### 2. Linux-RT 实时版从站

```bash
# 启用 RTU + TCP 双模式
./slave_rt/bin/modbus_slave_rt -m both

# 启用 RTU 并设置参数
./slave_rt/bin/modbus_slave_rt -m rtu -d /dev/ttyAS2 -v
```

### 3. 模拟主站 (PC上运行)

```bash
# TCP 主站测试
./master_sim/bin/master_sim -t tcp -h 192.168.1.100 -p 502

# RTU 主站测试
./master_sim/bin/master_sim -t rtu -d /dev/ttyUSB0 -b 115200

# 指定轮询间隔
./master_sim/bin/master_sim -t tcp -i 50  # 50ms 间隔

# 指定测试次数
./master_sim/bin/master_sim -t tcp -c 100  # 运行 100 次后退出
```

## 测试方案

### 测试架构

```
PC (Modbus Master)                    TLT153 (Slave)
┌─────────────────────┐              ┌─────────────────────┐
│ master_sim         │              │ modbus_slave        │
│  - TCP/RTU 测试     │◄─── 网络 ────►│  - 数据模拟器       │
│  - 延迟测量         │◄─── RS485 ───►│  - 寄存器响应       │
└─────────────────────┘              └─────────────────────┘
```

### 测试项目

| # | 测试项 | 命令 | 预期结果 |
|---|--------|------|----------|
| 1 | TCP连接 | `master_sim -t tcp` | 连接成功 |
| 2 | 读取输入寄存器 | 自动测试 | 返回电位器+DI数据 |
| 3 | 读取线圈 | 自动测试 | 返回DO状态 |
| 4 | 写入线圈 | 自动测试 | 线圈值改变 |
| 5 | 响应延迟 | 自动测量 | TCP <5ms, RTU <2ms |

## 数据模拟器

无需真实硬件，模拟器生成以下数据：

| 数据类型 | 模拟方式 |
|----------|----------|
| DI (22路) | 随机 0/1，每 100ms 变化 |
| DO (12路) | 记录主站写入值 |
| 电位器 (5路) | 正弦波 0-4095，周期 10s |
| RS485 (5路) | 模拟帧，每 20ms 更新 |
| RS232 (3路) | 256 字节缓冲 |

## GPIO 配置

GPIO 引脚采用灵活配置，可在代码中修改 `gpio_config` 表：

```c
// DI 引脚配置 (common/include/register_map.h)
const gpio_di_config_t g_di_gpio_config[22] = {
    {34,  "PB2",  "DI_CH01"},
    {129, "PE1",  "DI_CH02"},
    // ...
};

// DO 引脚配置
const gpio_do_config_t g_do_gpio_config[12] = {
    {35,  "PB3",  "DO_CH01"},
    // ...
};
```

## 寄存器映射

### 输入寄存器 (30001-30864)

| 地址 | 功能 | 说明 |
|------|------|------|
| 30001-30005 | 电位器增益 | 0-4095 |
| 30006-30027 | DI锁存状态 | 位标志 |
| 30028-30032 | RS485长度 | 0-10字节 |
| 30033-30082 | RS485数据 | 5×10字节 |
| 30083-30087 | RS485状态 | 位标志 |
| 30088-30090 | RS232长度 | 0-256字节 |
| 30091-30858 | RS232数据 | 3×256字节 |
| 30859-30864 | RS232状态 | 位标志 |

### 保持寄存器 (40001-40836)

| 地址 | 功能 | 说明 |
|------|------|------|
| 40001 | 设备地址 | 1-247 |
| 40002 | 波特率 | 0-6 |
| 40020 | 保存参数 | 写入0x55AA |
| 40021-40042 | DI清除 | 清除锁存 |
| 40058-40825 | RS232发送 | 发送缓冲 |

## 实时特性 (Linux-RT版)

- **内存锁定**: `mlockall(MCL_CURRENT | MCL_FUTURE)`
- **线程调度**: `SCHED_FIFO`
- **RTU 线程**: PRI=99, 周期 1ms
- **TCP 线程**: PRI=90
- **SYNC 线程**: PRI=80, 周期 10ms

## 硬件连接

| 接口 | 设备节点 | 说明 |
|------|----------|------|
| RS-485 | /dev/ttyAS2 | 默认 RTU 设备 |
| Ethernet | eth0 | TCP 端口 502 |
| GPIO | sysfs 或 mmap | DI/DO 控制 |

## 故障排除

### 1. 编译错误

```bash
# 安装依赖
sudo apt install build-essential gcc g++ make

# ARM 交叉编译
sudo apt install gcc-arm-linux-gnueabihf
```

### 2. 连接失败

```bash
# 检查网络
ping 192.168.1.100

# 检查端口
netstat -an | grep 502
```

### 3. 权限问题

```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录

# 或使用 sudo 运行
sudo ./modbus_slave
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2024 | 初始版本 |

## 协议文档

参见 `../信号采集处理模块-Modbus接口协议-V1.0.docx`
