/**
 * AD7606 ADC 驱动实现
 *
 * 功能: 通过 GPIO 模拟 SPI 读取 AD7606 5通道16位ADC数据
 */

#include "ad7606.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

/* ============================================
 * GPIO 操作封装
 * ============================================ */

static int gpio_export(int gpio)
{
    char buf[64];
    int fd;
    ssize_t len;

    /* 导出 GPIO */
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("gpio_export open");
        return -1;
    }
    len = snprintf(buf, sizeof(buf), "%d", gpio);
    if (write(fd, buf, len) != len) {
        close(fd);
        return -1;
    }
    close(fd);

    return 0;
}

static int gpio_unexport(int gpio)
{
    char buf[64];
    int fd;
    ssize_t len;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        return -1;
    }
    len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);

    return 0;
}

static int gpio_direction(int gpio, const char *dir)
{
    char buf[128];
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "gpio_direction: cannot open gpio%d\n", gpio);
        return -1;
    }
    if (write(fd, dir, strlen(dir)) != strlen(dir)) {
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

static int gpio_write(int gpio, int value)
{
    char buf[64];
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "gpio_write: cannot open gpio%d\n", gpio);
        return -1;
    }
    write(fd, value ? "1" : "0", 1);
    close(fd);
    return 0;
}

static int gpio_read(int gpio)
{
    char buf[64];
    char val;
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(buf, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "gpio_read: cannot open gpio%d\n", gpio);
        return -1;
    }
    read(fd, &val, 1);
    close(fd);

    return (val == '1') ? 1 : 0;
}

/* ============================================
 * SPI 模拟时序
 * ============================================ */

/**
 * 发送一个 SPI 时钟脉冲并读取 1 位数据
 * AD7606 在 SCLK 下降沿后输出数据，有效数据在 SCLK 高电平期间采样
 */
static int spi_clk_and_read(ad7606_dev_t *dev)
{
    /* SCLK 低 -> 高: 数据建立 */
    /* 读取 MISO */
    int bit = gpio_read(dev->gpio_miso);

    /* 延时 */
    usleep(AD7606_SPI_SCLK_HIGH_US);

    /* SCLK 高 -> 低 */
    usleep(AD7606_SPI_SCLK_LOW_US);

    return bit;
}

/**
 * 读取 16 位数据 (MSB first)
 */
static int16_t spi_read_16bit(ad7606_dev_t *dev)
{
    int16_t data = 0;
    int bit;

    /* AD7606: SCLK 下降沿后 MISO 有效, MSB first */
    for (int i = 15; i >= 0; i--) {
        /* 产生时钟脉冲并读取 1 位 */
        bit = spi_clk_and_read(dev);
        data |= (bit << i);
    }

    return data;
}

/* ============================================
 * 公共函数实现
 * ============================================ */

const char* ad7606_get_version(void)
{
    return "AD7606 Driver v1.0.0";
}

int ad7606_init(ad7606_dev_t *dev)
{
    int ret;

    if (dev == NULL) {
        return -1;
    }

    /* 清零结构体 */
    memset(dev, 0, sizeof(ad7606_dev_t));

    /* 导出 GPIO */
    ret = gpio_export(AD7606_GPIO_RESET);
    if (ret != 0) {
        fprintf(stderr, "[AD7606] Failed to export RESET GPIO %d\n", AD7606_GPIO_RESET);
        return -1;
    }

    gpio_export(AD7606_GPIO_CONVST);
    gpio_export(AD7606_GPIO_CS);
    gpio_export(AD7606_GPIO_BUSY);
    gpio_export(AD7606_GPIO_MISO);

    /* 设置方向 */
    gpio_direction(AD7606_GPIO_RESET, "out");
    gpio_direction(AD7606_GPIO_CONVST, "out");
    gpio_direction(AD7606_GPIO_CS, "out");
    gpio_direction(AD7606_GPIO_BUSY, "in");
    gpio_direction(AD7606_GPIO_MISO, "in");

    /* 初始化引脚电平 */
    gpio_write(AD7606_GPIO_RESET, 1);   /* RESET = 1 (不复位) */
    gpio_write(AD7606_GPIO_CONVST, 1);   /* CONVST = 1 */
    gpio_write(AD7606_GPIO_CS, 1);       /* CS = 1 (不选中) */

    /* 保存 GPIO 编号 */
    dev->gpio_reset = AD7606_GPIO_RESET;
    dev->gpio_convst = AD7606_GPIO_CONVST;
    dev->gpio_cs = AD7606_GPIO_CS;
    dev->gpio_busy = AD7606_GPIO_BUSY;
    dev->gpio_miso = AD7606_GPIO_MISO;

    dev->initialized = true;
    dev->sample_rate = AD7606_DEFAULT_RATE;

    printf("[AD7606] Initialized: RESET=%d, CONVST=%d, CS=%d, BUSY=%d, MISO=%d\n",
           AD7606_GPIO_RESET, AD7606_GPIO_CONVST, AD7606_GPIO_CS,
           AD7606_GPIO_BUSY, AD7606_GPIO_MISO);

    return 0;
}

void ad7606_release(ad7606_dev_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }

    /* 停止所有操作 */
    dev->running = false;

    /* 设置所有输出为低 */
    gpio_write(AD7606_GPIO_RESET, 0);
    gpio_write(AD7606_GPIO_CONVST, 0);
    gpio_write(AD7606_GPIO_CS, 0);

    /* 取消导出 */
    gpio_unexport(AD7606_GPIO_RESET);
    gpio_unexport(AD7606_GPIO_CONVST);
    gpio_unexport(AD7606_GPIO_CS);
    gpio_unexport(AD7606_GPIO_BUSY);
    gpio_unexport(AD7606_GPIO_MISO);

    dev->initialized = false;

    printf("[AD7606] Released\n");
}

int ad7606_reset(ad7606_dev_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return -1;
    }

    /* RESET 脉冲: 低至少 50ns */
    gpio_write(AD7606_GPIO_RESET, 0);
    usleep(10);  /* 10us >> 50ns */
    gpio_write(AD7606_GPIO_RESET, 1);

    /* 等待 100us 确保复位完成 */
    usleep(100);

    printf("[AD7606] Reset done\n");
    return 0;
}

int ad7606_start_conv(ad7606_dev_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return -1;
    }

    /* CONVST 下降沿启动转换 */
    gpio_write(AD7606_GPIO_CONVST, 0);
    usleep(AD7606_CONV_PULSE_US);
    gpio_write(AD7606_GPIO_CONVST, 1);

    return 0;
}

int ad7606_wait_busy_low(ad7606_dev_t *dev, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    int busy;

    if (dev == NULL || !dev->initialized) {
        return -1;
    }

    /* 等待 BUSY 变低 (转换完成) */
    while (1) {
        busy = gpio_read(AD7606_GPIO_BUSY);
        if (busy == 0) {
            break;
        }

        usleep(10);  /* 10us 轮询 */
        elapsed += 10;

        if (elapsed >= timeout_ms * 1000) {
            printf("[AD7606] Timeout waiting for BUSY low (still high)\n");
            return -1;
        }
    }

    return 0;
}

int ad7606_read_once(ad7606_dev_t *dev)
{
    struct timespec ts;
    int ret;

    if (dev == NULL || !dev->initialized) {
        return -1;
    }

    /* 1. 启动转换 */
    ad7606_start_conv(dev);

    /* 2. 等待转换完成 */
    ret = ad7606_wait_busy_low(dev, AD7606_BUSY_TIMEOUT_MS);
    if (ret != 0) {
        dev->error_count++;
        return -1;
    }

    /* 3. 片选取通 */
    gpio_write(AD7606_GPIO_CS, 0);
    usleep(AD7606_CS_SETUP_US);

    /* 4. 读取 5 通道数据 */
    for (int ch = 0; ch < AD7606_CHANNEL_NUM; ch++) {
        int16_t raw = spi_read_16bit(dev);
        dev->samples[ch].raw = raw;

        /* 计算电压 (假设 ±5V 范围, VREF=2.5V) */
        float voltage;
        ad7606_raw_to_voltage(raw, &voltage);
        dev->samples[ch].voltage = voltage;
    }

    /* 5. 片选取消 */
    gpio_write(AD7606_GPIO_CS, 1);

    /* 6. 获取时间戳 */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    dev->timestamp_ns = (uint64_t)ts.tv_sec * 1000000000LL + (uint64_t)ts.tv_nsec;
    dev->sample_count++;

    return 0;
}

void ad7606_get_samples(ad7606_dev_t *dev, ad7606_sample_t *samples)
{
    if (dev == NULL || samples == NULL) {
        return;
    }

    for (int i = 0; i < AD7606_CHANNEL_NUM; i++) {
        samples[i] = dev->samples[i];
    }
}

int16_t ad7606_get_raw(ad7606_dev_t *dev, int channel)
{
    if (dev == NULL || channel < 0 || channel >= AD7606_CHANNEL_NUM) {
        return 0;
    }
    return dev->samples[channel].raw;
}

float ad7606_get_voltage(ad7606_dev_t *dev, int channel)
{
    if (dev == NULL || channel < 0 || channel >= AD7606_CHANNEL_NUM) {
        return 0.0f;
    }
    return dev->samples[channel].voltage;
}

uint64_t ad7606_get_timestamp(ad7606_dev_t *dev)
{
    if (dev == NULL) {
        return 0;
    }
    return dev->timestamp_ns;
}

uint64_t ad7606_get_sample_count(ad7606_dev_t *dev)
{
    if (dev == NULL) {
        return 0;
    }
    return dev->sample_count;
}

uint64_t ad7606_get_error_count(ad7606_dev_t *dev)
{
    if (dev == NULL) {
        return 0;
    }
    return dev->error_count;
}

void ad7606_raw_to_voltage(int16_t raw, float *voltage)
{
    /*
     * AD7606 电压换算:
     * - 16位有符号，满量程 = ±32768
     * - 默认 ±5V 输入范围 (可通过引脚配置为 ±10V, ±2.5V)
     * - VREF = 2.5V (内部基准)
     * - 电压 = (raw / 32768.0) * 5.0
     */
    if (voltage == NULL) {
        return;
    }
    *voltage = ((float)raw / 32768.0f) * 5.0f;
}

void ad7606_raw_to_percent(int16_t raw, float *percent)
{
    /*
     * 转换为百分比:
     * - -32768 -> -100%
     * - 0 -> 0%
     * - +32767 -> +100%
     */
    if (percent == NULL) {
        return;
    }
    *percent = ((float)raw / 32768.0f) * 100.0f;
}
