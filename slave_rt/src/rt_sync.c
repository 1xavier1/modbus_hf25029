/* Copyright 2024. All Rights Reserved. */
/* Real-time Data Synchronization Thread */

#include "rt_sync.h"
#include "rt_utils.h"
#include "modbus_func.h"
#include "data_sim.h"
#include <stdio.h>
#include <time.h>

// Sync period: 10ms
#define SYNC_PERIOD_US   10000

void* rt_sync_thread(void *arg) {
    (void)arg;

    RT_LOG_INFO("SYNC thread started (period=%dus)", SYNC_PERIOD_US);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    uint64_t loop_count = 0;
    int64_t max_latency = 0;
    int64_t total_latency = 0;

    while (!g_quit) {
        uint64_t t1 = rt_get_time_us();

        // Update data simulation
        data_sim_update(DATA_SIM_TICK_MS);

        // Update next period
        rt_add_timespec(&next_time, SYNC_PERIOD_US);

        // Calculate latency
        uint64_t t2 = rt_get_time_us();
        int64_t latency = t2 - t1 - SYNC_PERIOD_US;

        if (latency > max_latency) {
            max_latency = latency;
        }
        total_latency += latency;

        loop_count++;
        if (loop_count % 100 == 0) {
            int64_t avg_latency = total_latency / loop_count;
            RT_LOG_INFO("SYNC: loops=%llu, max_lat=%lldus, avg_lat=%lldus",
                       loop_count, max_latency, avg_latency);
        }

        // Sleep until next period
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    RT_LOG_INFO("SYNC thread exiting");
    return NULL;
}
