/**
 * AD7606 测试程序
 *
 * 用法: ./ad7606_test [-r rate] [-c count] [-v]
 *
 *   -r rate   采样率 Hz (默认 1000)
 *   -c count  采样次数 (默认 10, 0=无限)
 *   -v        详细输出
 *   -t        打印时间戳
 *   -h        帮助
 */

#include "ad7606.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

/* ============================================
 * 全局变量
 * ============================================ */

static volatile int g_running = 1;
static int g_verbose = 0;
static int g_print_timestamp = 0;

/* ============================================
 * 信号处理
 * ============================================ */

void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ============================================
 * 打印帮助
 * ============================================ */

void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -r <rate>   Sample rate in Hz (default: 1000, max: 10000)\n");
    printf("  -c <count>  Sample count (default: 10, 0=infinite)\n");
    printf("  -v          Verbose output\n");
    printf("  -t          Print timestamp\n");
    printf("  -h          Help\n");
    printf("\nExample:\n");
    printf("  %s -r 1000 -c 100 -v\n", prog);
}

/* ============================================
 * 主程序
 * ============================================ */

int main(int argc, char *argv[])
{
    ad7606_dev_t dev;
    int sample_rate = 1000;
    int sample_count = 10;
    int opt;
    int ret;
    int total = 0;
    int errors = 0;

    /* 解析参数 */
    while ((opt = getopt(argc, argv, "r:c:vth")) != -1) {
        switch (opt) {
            case 'r':
                sample_rate = atoi(optarg);
                if (sample_rate <= 0) sample_rate = 1000;
                if (sample_rate > 10000) sample_rate = 10000;
                break;
            case 'c':
                sample_count = atoi(optarg);
                break;
            case 'v':
                g_verbose = 1;
                break;
            case 't':
                g_print_timestamp = 1;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }

    printf("===========================================\n");
    printf("  AD7606 ADC Test Program\n");
    printf("===========================================\n");
    printf("Channels: %d\n", AD7606_CHANNEL_NUM);
    printf("Resolution: %d bits\n", AD7606_BITS);
    printf("Sample rate: %d Hz\n", sample_rate);
    printf("Sample count: %s\n", sample_count == 0 ? "infinite" : "10");
    printf("===========================================\n\n");

    /* 初始化 */
    printf("[INFO] Initializing AD7606...\n");
    ret = ad7606_init(&dev);
    if (ret != 0) {
        printf("[ERROR] Failed to initialize AD7606\n");
        return 1;
    }

    /* 复位 */
    printf("[INFO] Resetting AD7606...\n");
    ad7606_reset(&dev);

    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 计算采样间隔 */
    int interval_us = 1000000 / sample_rate;
    if (interval_us < 100) interval_us = 100;  /* 最小 100us */

    printf("[INFO] Starting acquisition (interval=%d us)...\n\n", interval_us);

    /* 采集循环 */
    while (g_running && (sample_count == 0 || total < sample_count)) {
        ad7606_sample_t samples[AD7606_CHANNEL_NUM];

        /* 读取数据 */
        ret = ad7606_read_once(&dev);
        if (ret != 0) {
            errors++;
            printf("[WARN] Read error %d\n", errors);
            continue;
        }

        /* 获取数据 */
        ad7606_get_samples(&dev, samples);

        /* 打印 */
        if (g_verbose) {
            if (g_print_timestamp) {
                printf("[%llu] ", (unsigned long long)dev.timestamp_ns);
            }
            for (int i = 0; i < AD7606_CHANNEL_NUM; i++) {
                printf("CH%d=%+8d (%+7.4fV)  ",
                       i + 1,
                       samples[i].raw,
                       samples[i].voltage);
            }
            printf("\n");
        } else {
            /* 简洁输出 */
            if (g_print_timestamp) {
                printf("[%llu] ", (unsigned long long)dev.timestamp_ns);
            }
            for (int i = 0; i < AD7606_CHANNEL_NUM; i++) {
                printf("%+7.4fV ", samples[i].voltage);
            }
            printf("\n");
        }

        total++;
        usleep(interval_us);
    }

    /* 打印统计 */
    printf("\n===========================================\n");
    printf("  Acquisition Summary\n");
    printf("===========================================\n");
    printf("Total samples: %d\n", total);
    printf("Errors: %d\n", errors);
    printf("Sample count: %llu\n", (unsigned long long)dev.sample_count);
    printf("===========================================\n");

    /* 清理 */
    ad7606_release(&dev);
    printf("[INFO] Done\n");

    return 0;
}
