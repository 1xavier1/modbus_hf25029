/* Copyright 2024. All Rights Reserved. */
/* Modbus Master Simulator - Main Program */

#include "master_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

volatile bool g_quit = false;

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -t, --type <type>    Master type: tcp, rtu (default: tcp)\n");
    printf("  -h, --host <ip>     TCP host (default: 192.168.1.100)\n");
    printf("  -p, --port <port>   TCP port (default: 502)\n");
    printf("  -d, --device <dev>  RTU device (default: /dev/ttyUSB0)\n");
    printf("  -b, --baud <baud>   RTU baudrate (default: 115200)\n");
    printf("  -a, --addr <addr>   Slave address (default: 1)\n");
    printf("  -i, --interval <ms> Poll interval (default: 100ms)\n");
    printf("  -c, --count <n>     Test count (0=infinite, default: 0)\n");
    printf("  -v, --verbose       Verbose output\n");
    printf("  -h, --help         Show this help\n");
    printf("\nExamples:\n");
    printf("  %s -t tcp -h 192.168.1.100 -p 502\n", prog);
    printf("  %s -t rtu -d /dev/ttyUSB0 -b 115200\n", prog);
}

static void signal_handler(int sig) {
    (void)sig;
    printf("\n[MASTER] Shutting down...\n");
    g_quit = true;
}

int main(int argc, char *argv[]) {
    const char *master_type = "tcp";
    const char *tcp_host = "192.168.1.100";
    int tcp_port = 502;
    const char *rtu_device = "/dev/ttyUSB0";
    int rtu_baud = 115200;
    int slave_addr = 1;
    int poll_interval = 100;
    int test_count = 0;
    int verbose = 0;

    // Parse options
    int opt;
    struct option longopts[] = {
        {"type", required_argument, NULL, 't'},
        {"host", required_argument, NULL, 'h'},
        {"port", required_argument, NULL, 'p'},
        {"device", required_argument, NULL, 'd'},
        {"baud", required_argument, NULL, 'b'},
        {"addr", required_argument, NULL, 'a'},
        {"interval", required_argument, NULL, 'i'},
        {"count", required_argument, NULL, 'c'},
        {"verbose", no_argument, NULL, 'v'},
        {"help", no_argument, NULL, 'H'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "t:h:p:d:b:a:i:c:vH", longopts, NULL)) != -1) {
        switch (opt) {
            case 't': master_type = optarg; break;
            case 'h': tcp_host = optarg; break;
            case 'p': tcp_port = atoi(optarg); break;
            case 'd': rtu_device = optarg; break;
            case 'b': rtu_baud = atoi(optarg); break;
            case 'a': slave_addr = atoi(optarg); break;
            case 'i': poll_interval = atoi(optarg); break;
            case 'c': test_count = atoi(optarg); break;
            case 'v': verbose = 1; break;
            case 'H':
            default:
                print_usage(argv[0]);
                return (opt == 'H') ? 0 : 1;
        }
    }

    printf("===========================================\n");
    printf("  Modbus Master Simulator\n");
    printf("===========================================\n");
    printf("Type:  %s\n", master_type);
    printf("Slave: %d\n", slave_addr);
    if (strcmp(master_type, "tcp") == 0) {
        printf("Host:  %s:%d\n", tcp_host, tcp_port);
    } else {
        printf("Dev:   %s @ %d\n", rtu_device, rtu_baud);
    }
    printf("Interval: %dms\n", poll_interval);
    printf("Count: %d\n", test_count);
    printf("===========================================\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (strcmp(master_type, "tcp") == 0) {
        tcp_master_run(tcp_host, tcp_port, slave_addr, poll_interval, test_count, verbose);
    } else {
        rtu_master_run(rtu_device, rtu_baud, slave_addr, poll_interval, test_count, verbose);
    }

    printf("[MASTER] Exit\n");
    return 0;
}
