/**
 * Modbus 函数头文件
 *
 * 定义 Modbus 公共接口
 */

#ifndef _MODBUS_FUNC_H_
#define _MODBUS_FUNC_H_

#include <stdint.h>
#include "ad7606.h"

/* ============================================
 * AD7606 与 Modbus 集成接口
 * ============================================ */

/**
 * 初始化 AD7606
 * @return 0 成功, -1 失败
 */
int modbus_init_ad7606(void);

/**
 * 释放 AD7606
 */
void modbus_release_ad7606(void);

/**
 * 单次读取 AD7606
 * @return 0 成功, -1 失败
 */
int modbus_ad7606_read_once(void);

/**
 * 获取 AD7606 采样数据
 */
void modbus_get_ad7606_samples(ad7606_sample_t *samples);

/**
 * 读取输入寄存器 (功能码 0x04)
 * @param start_addr 起始地址 (0-based)
 * @param count 寄存器数量
 * @param data 输出数据缓冲区
 * @return 0 成功, -1 失败
 */
int modbus_read_input_registers(uint16_t start_addr, uint16_t count, uint16_t *data);

/**
 * 读取保持寄存器 (功能码 0x03)
 */
int modbus_read_holding_registers(uint16_t start_addr, uint16_t count, uint16_t *data);

/**
 * 写入单个寄存器 (功能码 0x06)
 */
int modbus_write_single_register(uint16_t addr, uint16_t value);

/**
 * 写入多个寄存器 (功能码 0x10)
 */
int modbus_write_multiple_registers(uint16_t start_addr, uint16_t count, const uint16_t *data);

/**
 * 诊断: 打印 AD7606 状态
 */
void modbus_dump_ad7606_status(void);

#endif /* _MODBUS_FUNC_H_ */
