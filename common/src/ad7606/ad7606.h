/**
 * AD7606 ADC 驱动头文件
 *
 * AD7606: 5通道 16位 同步采样 ADC
 * 硬件连接: TLT153 GPIO PK0-PK5
 *
 * GPIO 分配:
 *   PK0 (GPIO 320): RESET  - 复位，低有效
 *   PK1 (GPIO 321): CONVST - 转换启动
 *   PK2 (GPIO 322): CS     - 片选，低有效
 *   PK3 (GPIO 323): SCLK   - SPI时钟 (复用)
 *   PK4 (GPIO 324): BUSY   - 忙标志，输入
 *   PK5 (GPIO 325): MISO   - 数据输出 (复用)
 */

#ifndef _AD7606_H_
#define _AD7606_H_

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>

/* ============================================
 * GPIO 编号定义
 * ============================================ */
#define AD7606_GPIO_RESET   320  // PK0 - 复位，低有效
#define AD7606_GPIO_CONVST  321  // PK1 - 转换启动
#define AD7606_GPIO_CS      322  // PK2 - 片选，低有效
#define AD7606_GPIO_BUSY    324  // PK4 - 忙标志，输入
#define AD7606_GPIO_MISO    325  // PK5 - 数据输出

/* ============================================
 * AD7606 配置
 * ============================================ */
#define AD7606_CHANNEL_NUM     5       // 5通道
#define AD7606_BITS            16      // 16位精度
#define AD7606_MAX_SAMPLE_RATE 200000 // 最大200kSPS
#define AD7606_DEFAULT_RATE     1000   // 默认1kHz采样率

/* ============================================
 * SPI 时序参数 (微秒)
 * ============================================ */
#define AD7606_SPI_SCLK_LOW_US    1    // SCLK 低电平时间
#define AD7606_SPI_SCLK_HIGH_US   1    // SCLK 高电平时间
#define AD7606_CONV_PULSE_US     1    // CONVST 脉冲宽度
#define AD7606_CS_SETUP_US        1    // CS 建立时间
#define AD7606_BUSY_TIMEOUT_MS    10   // BUSY 等待超时

/* ============================================
 * 数据结构
 * ============================================ */

/* 单通道采样数据 */
typedef struct {
    int16_t raw;           // 原始ADC值 (-32768 ~ +32767)
    float voltage;          // 转换后电压 (V)
} ad7606_sample_t;

/* AD7606 设备 */
typedef struct ad7606_dev {
    int gpio_reset;         // RESET GPIO 编号
    int gpio_convst;       // CONVST GPIO 编号
    int gpio_cs;           // CS GPIO 编号
    int gpio_busy;         // BUSY GPIO 编号
    int gpio_miso;         // MISO GPIO 编号

    bool initialized;       // 初始化标志
    bool running;           // 运行标志

    int sample_rate;        // 采样率 (Hz)

    ad7606_sample_t samples[AD7606_CHANNEL_NUM];  // 最新采样值
    uint64_t timestamp_ns;  // 最后采样时间戳

    uint64_t sample_count;   // 总采样计数
    uint64_t error_count;   // 错误计数

    void *next;             // 链表指针
} ad7606_dev_t;

/* ============================================
 * 寄存器地址定义 (Modbus)
 * ============================================ */
#define REG_ADC_CH1_RAW   30001   // AD7606 通道1原始值
#define REG_ADC_CH2_RAW   30002   // AD7606 通道2原始值
#define REG_ADC_CH3_RAW   30003   // AD7606 通道3原始值
#define REG_ADC_CH4_RAW   30004   // AD7606 通道4原始值
#define REG_ADC_CH5_RAW   30005   // AD7606 通道5原始值

/* ============================================
 * 函数声明
 * ============================================ */

/* 初始化和释放 */
int ad7606_init(ad7606_dev_t *dev);
void ad7606_release(ad7606_dev_t *dev);

/* 基本操作 */
int ad7606_reset(ad7606_dev_t *dev);
int ad7606_start_conv(ad7606_dev_t *dev);
int ad7606_wait_busy_low(ad7606_dev_t *dev, uint32_t timeout_ms);
int ad7606_read_once(ad7606_dev_t *dev);

/* 数据访问 */
void ad7606_get_samples(ad7606_dev_t *dev, ad7606_sample_t *samples);
int16_t ad7606_get_raw(ad7606_dev_t *dev, int channel);
float ad7606_get_voltage(ad7606_dev_t *dev, int channel);
uint64_t ad7606_get_timestamp(ad7606_dev_t *dev);
uint64_t ad7606_get_sample_count(ad7606_dev_t *dev);
uint64_t ad7606_get_error_count(ad7606_dev_t *dev);

/* 工具函数 */
void ad7606_raw_to_voltage(int16_t raw, float *voltage);
void ad7606_raw_to_percent(int16_t raw, float *percent);
const char* ad7606_get_version(void);

#endif /* _AD7606_H_ */
