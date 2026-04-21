/* Copyright 2024. All Rights Reserved. */
/* HF25029-CDP Modbus Slave - Linux-RT Version */

#include "modbus_func.h"
#include "register_map.h"
#include "hardware.h"
#include "rt_utils.h"
#include "rt_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

// ========================================================================
// Command Line Options
// ========================================================================

static const char *optstring = "m:d:b:p:n:c:vh";

static struct option longopts[] = {
    {"mode",         required_argument, NULL, 'm'},
    {"device",       required_argument, NULL, 'd'},
    {"baud",         required_argument, NULL, 'b'},
    {"parity",       required_argument, NULL, 'p'},
    {"net-config",   required_argument, NULL, 'n'},
    {"gpio-config",  required_argument, NULL, 'c'},
    {"verbose",      no_argument,       NULL, 'v'},
    {"help",         no_argument,       NULL, 'h'},
    {0, 0, 0, 0}
};

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -m, --mode <mode>       Operation mode: rtu, tcp, both (default: both)\n");
    printf("  -d, --device <dev>      RTU device path (default: /dev/ttyAS2)\n");
    printf("  -b, --baud <baud>       RTU baudrate (default: 115200)\n");
    printf("  -p, --parity <p>        RTU parity: n, o, e (default: n)\n");
    printf("  -n, --net-config <f>   Network config file\n");
    printf("  -c, --gpio-config <f>   GPIO config file\n");
    printf("  -v, --verbose           Enable verbose logging\n");
    printf("  -h, --help              Show this help\n");
    printf("\nRT Options:\n");
    printf("  Real-time threads with SCHED_FIFO scheduling\n");
    printf("  RTU thread: priority 99, period 1ms\n");
    printf("  TCP thread: priority 90\n");
    printf("  SYNC thread: priority 80, period 10ms\n");
}

// ========================================================================
// Global Variables
// ========================================================================

volatile bool g_quit = false;

pthread_t g_rtu_thread;
pthread_t g_tcp_thread;
pthread_t g_sync_thread;

int g_log_level = LOG_LEVEL_INFO;

// ========================================================================
// Signal Handler
// ========================================================================

static void signal_handler(int sig) {
    (void)sig;
    printf("\n[MAIN-RT] Received signal, shutting down...\n");
    g_quit = true;
}

// ========================================================================
// External Thread Functions
// ========================================================================

extern void* rt_rtu_thread(void *arg);
extern void* rt_tcp_thread(void *arg);
extern void* rt_sync_thread(void *arg);

// ========================================================================
// Main
// ========================================================================

int main(int argc, char *argv[]) {
    int mode_rtu = 1;
    int mode_tcp = 1;
    const char *rtu_device = MODBUS_RTU_DEFAULT_DEV;
    const char *network_config_file = NULL;
    const char *gpio_config_file = NULL;

    // Parse command line options
    int opt;
    while ((opt = getopt_long(argc, argv, optstring, longopts, NULL)) != -1) {
        switch (opt) {
        case 'm':
            if (strcmp(optarg, "rtu") == 0) {
                mode_rtu = 1;
                mode_tcp = 0;
            } else if (strcmp(optarg, "tcp") == 0) {
                mode_rtu = 0;
                mode_tcp = 1;
            } else if (strcmp(optarg, "both") == 0) {
                mode_rtu = 1;
                mode_tcp = 1;
            }
            break;
        case 'd':
            rtu_device = optarg;
            break;
        case 'n':
            network_config_file = optarg;
            break;
        case 'c':
            gpio_config_file = optarg;
            break;
        case 'v':
            g_log_level = LOG_LEVEL_DEBUG;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("===========================================\n");
    printf("  %s (Linux-RT)\n", PROGRAM_NAME);
    printf("  Version: %s\n", VERSION);
    printf("===========================================\n");
    printf("Mode: %s%s%s\n",
           mode_rtu ? "RTU " : "",
           mode_rtu && mode_tcp ? "+" : "",
           mode_tcp ? "TCP" : "");
    printf("===========================================\n");

    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Lock memory (prevent swapping for real-time determinism)
    printf("[MAIN-RT] Locking memory...\n");
    rt_lock_memory();

    // Initialize RT GPIO (memory-mapped)
    printf("[MAIN-RT] Initializing RT GPIO...\n");
    if (rt_gpio_init() == 0) {
        printf("[MAIN-RT] RT GPIO initialized (mmap mode)\n");
    } else {
        printf("[MAIN-RT] RT GPIO init failed, using sysfs mode\n");
    }

    // Initialize hardware
    printf("[MAIN-RT] Initializing hardware...\n");
    hardware_init();

    // Load network config if specified
    if (network_config_file) {
        printf("[MAIN-RT] Loading network config from %s...\n", network_config_file);
        hardware_load_config(network_config_file);
    }

    // Thread stack size (64KB for RT threads)
    #define RT_THREAD_STACK (64 * 1024)

    printf("[MAIN-RT] Creating real-time threads...\n");

    // Create RTU thread (highest priority)
    if (mode_rtu) {
        rt_create_thread(&g_rtu_thread, rt_rtu_thread,
                        (void*)rtu_device, RT_PRI_RTU, RT_THREAD_STACK);
    }

    // Create TCP thread
    if (mode_tcp) {
        rt_create_thread(&g_tcp_thread, rt_tcp_thread,
                        NULL, RT_PRI_TCP, RT_THREAD_STACK);
    }

    // Create SYNC thread (data synchronization)
    rt_create_thread(&g_sync_thread, rt_sync_thread,
                    NULL, RT_PRI_SYNC, RT_THREAD_STACK);

    printf("[MAIN-RT] All threads created, entering main loop...\n");

    // Main thread waits for signals
    while (!g_quit) {
        sleep(1);
    }

    // Wait for threads
    printf("[MAIN-RT] Waiting for threads to exit...\n");

    if (mode_rtu) {
        pthread_join(g_rtu_thread, NULL);
    }
    if (mode_tcp) {
        pthread_join(g_tcp_thread, NULL);
    }
    pthread_join(g_sync_thread, NULL);

    // Cleanup
    rt_gpio_cleanup();
    hardware_release();
    rt_unlock_memory();

    printf("[MAIN-RT] Exit\n");
    return 0;
}
