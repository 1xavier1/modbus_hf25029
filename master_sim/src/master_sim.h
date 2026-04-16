/* Master Simulator Common Header */

#ifndef MASTER_SIM_H
#define MASTER_SIM_H

#include <stdbool.h>
#include <stdint.h>

// Global quit flag (defined in main.c)
extern volatile bool g_quit;

// TCP Master
void tcp_master_run(const char *host, int port, int slave_addr,
                   int interval_ms, int count, int verbose);

// RTU Master
void rtu_master_run(const char *device, int baudrate, int slave_addr,
                   int interval_ms, int count, int verbose);

#endif /* MASTER_SIM_H */
