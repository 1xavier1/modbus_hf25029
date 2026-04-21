# AD7606 驱动使用说明

## 一、驱动位置

```
/home/tronlong/Workspace/modbus_hf25029/common/src/ad7606/
├── ad7606.h           # 头文件
├── ad7606.c           # 驱动实现
├── ad7606_test.c      # 测试程序
├── modbus_func.h      # Modbus 集成接口
├── modbus_ad7606.c    # Modbus 集成实现
└── Makefile
```

编译产物:
```
ad7606_test          # 测试程序
libad7606.a          # 静态库
```

## 二、编译

```bash
cd /home/tronlong/Workspace/modbus_hf25029/common/src/ad7606
make clean
make
```

## 三、部署到目标板

### 3.1 复制文件

```bash
# 复制测试程序 (需要 pthread 和 rt 库)
scp ad7606_test root@<目标IP>:/root/

# 或复制静态库
scp libad7606.a root@<目标IP>:/root/
```

### 3.2 在目标板上运行测试

```bash
# 基本测试 (10次采样)
./ad7606_test

# 1000Hz采样率，100次采样，详细输出
./ad7606_test -r 1000 -c 100 -v

# 带时间戳输出
./ad7606_test -r 500 -c 20 -v -t

# 无限采样直到 Ctrl+C
./ad7606_test -r 1000 -c 0 -v
```

## 四、GPIO 连接确认

驱动使用的 GPIO (设备树已配置):

| GPIO | 引脚 | 功能 |
|------|------|------|
| 320 | PK0 | RESET (复位，低有效) |
| 321 | PK1 | CONVST (转换启动) |
| 322 | PK2 | CS (片选，低有效) |
| 324 | PK4 | BUSY (忙标志) |
| 325 | PK5 | MISO (数据输出) |

确认 GPIO 已导出:
```bash
ls /sys/class/gpio/
# 应该看到 gpio320, gpio321, gpio322, gpio324, gpio325
```

## 五、输出格式

```
===========================================
  AD7606 ADC Test Program
===========================================
Channels: 5
Resolution: 16 bits
Sample rate: 1000 Hz
Sample count: 10
===========================================

[INFO] Initializing AD7606...
[INFO] Resetting AD7606...
[INFO] Starting acquisition (interval=1000 us)...

[INFO] CH1=    +0  (+0.0000V)  CH2=    +0  (+0.0000V)  CH3=    +0  (+0.0000V)  CH4=    +0  (+0.0000V)  CH5=    +0  (+0.0000V)
[INFO] CH1= +8192  (+1.2500V)  CH2= +8192  (+1.2500V)  CH3= +8192  (+1.2500V)  CH4= +8192  (+1.2500V)  CH5= +8192  (+1.2500V)
...

===========================================
  Acquisition Summary
===========================================
Total samples: 10
Errors: 0
Sample count: 10
===========================================
```

## 六、与 Modbus 集成

### 6.1 寄存器映射

| Modbus 地址 | 寄存器 | 说明 |
|------------|--------|------|
| 30001 | REG_ADC_CH1_RAW | AD7606 通道1原始值 |
| 30002 | REG_ADC_CH2_RAW | AD7606 通道2原始值 |
| 30003 | REG_ADC_CH3_RAW | AD7606 通道3原始值 |
| 30004 | REG_ADC_CH4_RAW | AD7606 通道4原始值 |
| 30005 | REG_ADC_CH5_RAW | AD7606 通道5原始值 |

### 6.2 集成到 modbus_slave

需要在 `slave/src/main.c` 中:

1. 初始化 AD7606:
```c
#include "modbus_func.h"

int main() {
    /* 初始化 AD7606 */
    modbus_init_ad7606();

    /* 启动采集线程 */
    pthread_create(&adc_thread, NULL, adc采集线程, NULL);

    /* 启动 Modbus 服务器 */
    modbus_tcp_server(502);
}
```

2. 在读取输入寄存器时返回 AD7606 数据:
```c
case 0x04:  /* Read Input Registers */
    modbus_read_input_registers(addr, count, response->data);
    break;
```

### 6.3 从主机读取 AD7606 数据

```python
# Python 示例
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('192.168.1.100', port=502)
client.connect()

# 读取 5 个通道的 AD7606 数据
result = client.read_input_registers(30001, 5, slave=1)

for i, reg in enumerate(result.registers):
    # 有符号转换
    if reg > 32767:
        value = reg - 65536
    else:
        value = reg
    voltage = (value / 32768.0) * 5.0
    print(f"CH{i+1}: raw={value}, voltage={voltage:.4f}V")
```

## 七、数据换算

AD7606 是 16 位有符号 ADC:

```
原始值范围: -32768 ~ +32767

电压换算 (±5V 输入范围, VREF=2.5V):
  voltage = (raw / 32768.0) * 5.0

百分比换算:
  percent = (raw / 32768.0) * 100.0

示例:
  raw = +32767 → voltage = +5.0000V → +100.00%
  raw =      0 → voltage =  0.0000V →   0.00%
  raw = -32768 → voltage = -5.0000V → -100.00%
```

## 八、注意事项

1. **GPIO 权限**: 运行程序需要 root 权限或适当的 GPIO 权限
2. **采样率**: 受 GPIO 操作延时限制，最大约 10kHz
3. **转换时间**: AD7606 转换时间约 3µs
4. **BUSY 超时**: 默认 10ms 超时

## 九、故障排除

### 9.1 GPIO 导出失败
```bash
# 检查 GPIO 是否已被占用
cat /sys/class/gpio/gpio320/value
```

### 9.2 读取全0
- 检查 AD7606 硬件连接
- 检查 CONVST 和 CS 信号
- 用示波器检查 SPI 时序

### 9.3 读取数据不稳定
- 检查模拟输入信号
- 检查参考电压稳定性
- 降低采样率试试
