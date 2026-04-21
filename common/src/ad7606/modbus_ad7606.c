/**
 * Modbus AD7606 集成模块
 *
 * 功能: 将 AD7606 ADC 数据映射到 Modbus 输入寄存器
 */

#include "modbus_func.h"
#include "ad7606.h"
#include <stdio.h>
#include <string.h>

/* ============================================
 * 全局变量
 * ============================================ */

static ad7606_dev_t g_ad7606;
static int g_ad7606_initialized = 0;

/* ============================================
 * AD7606 初始化
 * ============================================ */

int modbus_init_ad7606(void)
{
    int ret;

    if (g_ad7606_initialized) {
        printf("[MODBUS] AD7606 already initialized\n");
        return 0;
    }

    ret = ad7606_init(&g_ad7606);
    if (ret != 0) {
        printf("[MODBUS] AD7606 init failed\n");
        return -1;
    }

    /* 复位 ADC */
    ad7606_reset(&g_ad7606);

    g_ad7606_initialized = 1;
    printf("[MODBUS] AD7606 initialized\n");
    return 0;
}

void modbus_release_ad7606(void)
{
    if (!g_ad7606_initialized) {
        return;
    }

    ad7606_release(&g_ad7606);
    g_ad7606_initialized = 0;
    printf("[MODBUS] AD7606 released\n");
}

int modbus_ad7606_read_once(void)
{
    if (!g_ad7606_initialized) {
        return -1;
    }
    return ad7606_read_once(&g_ad7606);
}

void modbus_get_ad7606_samples(ad7606_sample_t *samples)
{
    if (!g_ad7606_initialized || samples == NULL) {
        return;
    }
    ad7606_get_samples(&g_ad7606, samples);
}

/* ============================================
 * 读取 AD7606 寄存器 (功能码 0x04)
 * ============================================ */

/**
 * 读取输入寄存器 - AD7606 数据
 *
 * Modbus 地址:
 *   30001 = REG_ADC_CH1_RAW
 *   30002 = REG_ADC_CH2_RAW
 *   30003 = REG_ADC_CH3_RAW
 *   30004 = REG_ADC_CH4_RAW
 *   30005 = REG_ADC_CH5_RAW
 */
int modbus_read_input_registers(uint16_t start_addr, uint16_t count, uint16_t *data)
{
    int i;
    uint16_t reg_addr;

    if (data == NULL || count == 0) {
        return -1;
    }

    /* 清零 */
    memset(data, 0, count * sizeof(uint16_t));

    for (i = 0; i < count; i++) {
        reg_addr = start_addr + i;

        switch (reg_addr) {
            /* AD7606 ADC 通道数据 (有符号16位 -> 无符号16位) */
            case REG_ADC_CH1_RAW:
                /* 有符号转无符号: 负数 + 65536 */
                data[i] = (uint16_t)(int16_t)g_ad7606.samples[0].raw;
                break;
            case REG_ADC_CH2_RAW:
                data[i] = (uint16_t)(int16_t)g_ad7606.samples[1].raw;
                break;
            case REG_ADC_CH3_RAW:
                data[i] = (uint16_t)(int16_t)g_ad7606.samples[2].raw;
                break;
            case REG_ADC_CH4_RAW:
                data[i] = (uint16_t)(int16_t)g_ad7606.samples[3].raw;
                break;
            case REG_ADC_CH5_RAW:
                data[i] = (uint16_t)(int16_t)g_ad7606.samples[4].raw;
                break;

            default:
                data[i] = 0;
                break;
        }
    }

    return 0;
}

/* ============================================
 * 诊断功能
 * ============================================ */

void modbus_dump_ad7606_status(void)
{
    if (!g_ad7606_initialized) {
        printf("AD7606: Not initialized\n");
        return;
    }

    printf("=== AD7606 Status ===\n");
    printf("Sample Rate: %d Hz\n", g_ad7606.sample_rate);
    printf("Sample Count: %llu\n", (unsigned long long)g_ad7606.sample_count);
    printf("Error Count: %llu\n", (unsigned long long)g_ad7606.error_count);
    printf("Timestamp: %llu ns\n", (unsigned long long)g_ad7606.timestamp_ns);
    printf("\n");
    printf("Channel | Raw Value | Voltage (V)\n");
    printf("--------+-----------+-------------\n");
    for (int i = 0; i < AD7606_CHANNEL_NUM; i++) {
        printf("   CH%d   | %8d  |   %+7.4f\n",
               i + 1,
               g_ad7606.samples[i].raw,
               g_ad7606.samples[i].voltage);
    }
}
