/* Copyright 2024. All Rights Reserved. */
/* HF25029-CDP Modbus Slave - Main Program */

#include "modbus_func.h"
#include "register_map.h"
#include "data_sim.h"
#include "gpio.h"
#include "serial.h"
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

// ========================================================================
// Command Line Options
// ========================================================================

static const char *optstring = "m:d:b:p:c:vh";

static struct option longopts[] = {
    {"mode",   required_argument, NULL, 'm'},
    {"device", required_argument, NULL, 'd'},
    {"baud",   required_argument, NULL, 'b'},
    {"parity", required_argument, NULL, 'p'},
    {"config", required_argument, NULL, 'c'},
    {"verbose", no_argument,       NULL, 'v'},
    {"help",   no_argument,       NULL, 'h'},
    {0, 0, 0, 0}
};

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -m, --mode <mode>    Operation mode: rtu, tcp, both (default: both)\n");
    printf("  -d, --device <dev>   RTU device path (default: /dev/ttyAS2)\n");
    printf("  -b, --baud <baud>    RTU baudrate (default: 115200)\n");
    printf("  -p, --parity <p>     RTU parity: n, o, e (default: n)\n");
    printf("  -c, --config <file>  GPIO config file (default: built-in)\n");
    printf("  -v, --verbose       Enable verbose logging\n");
    printf("  -h, --help          Show this help\n");
    printf("\nExample:\n");
    printf("  %s -m both -d /dev/ttyAS2 -b 115200\n", prog);
    printf("  %s -m tcp -c /root/gpio_config.json\n", prog);
}

// ========================================================================
// Global Variables
// ========================================================================

volatile bool g_quit = false;
pthread_t g_rtu_thread;
pthread_t g_tcp_thread;
pthread_mutex_t g_reg_mutex = PTHREAD_MUTEX_INITIALIZER;

int g_log_level = LOG_LEVEL_INFO;

// ========================================================================
// Signal Handler
// ========================================================================

static void signal_handler(int sig) {
    (void)sig;
    printf("\n[MAIN] Received signal, shutting down...\n");
    g_quit = true;
}

// ========================================================================
// Main
// ========================================================================

int main(int argc, char *argv[]) {
    int mode_rtu = 1;
    int mode_tcp = 1;
    const char *rtu_device = MODBUS_RTU_DEFAULT_DEV;
    int rtu_baud = MODBUS_RTU_DEFAULT_BAUD;
    char rtu_parity = MODBUS_RTU_DEFAULT_PARITY;
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
            } else {
                printf("Invalid mode: %s\n", optarg);
                print_usage(argv[0]);
                return 1;
            }
            break;
        case 'd':
            rtu_device = optarg;
            break;
        case 'b':
            rtu_baud = atoi(optarg);
            break;
        case 'p':
            rtu_parity = optarg[0];
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
    printf("  %s\n", PROGRAM_NAME);
    printf("  Version: %s\n", VERSION);
    printf("===========================================\n");
    printf("Mode: %s%s%s\n",
           mode_rtu ? "RTU " : "",
           mode_rtu && mode_tcp ? "+" : "",
           mode_tcp ? "TCP" : "");
    printf("RTU Device: %s @ %d %c%d\n",
           rtu_device, rtu_baud, rtu_parity, MODBUS_RTU_DEFAULT_BITS);
    printf("===========================================\n");

    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize data simulator
    printf("[MAIN] Initializing data simulator...\n");
    data_sim_init();

    // Initialize GPIO (optional, will use simulated data if fails)
    printf("[MAIN] Initializing GPIO from %s...\n",
           gpio_config_file ? gpio_config_file : "default config");
    if (gpio_init_with_config(gpio_config_file) == 0) {
        printf("[MAIN] GPIO initialized\n");
    } else {
        printf("[MAIN] GPIO init failed, using simulated data\n");
    }

    // Create threads
    if (mode_rtu) {
        printf("[MAIN] Starting RTU thread...\n");
        pthread_create(&g_rtu_thread, NULL, rtu_slave_thread, (void*)rtu_device);
    }

    if (mode_tcp) {
        printf("[MAIN] Starting TCP thread...\n");
        pthread_create(&g_tcp_thread, NULL, tcp_slave_thread, NULL);
    }

    // Main loop - update simulation
    printf("[MAIN] Entering main loop...\n");
    while (!g_quit) {
        data_sim_update(DATA_SIM_TICK_MS);
        msleep(DATA_SIM_TICK_MS);
    }

    // Wait for threads
    if (mode_rtu) {
        pthread_join(g_rtu_thread, NULL);
    }
    if (mode_tcp) {
        pthread_join(g_tcp_thread, NULL);
    }

    printf("[MAIN] Exit\n");
    return 0;
}
