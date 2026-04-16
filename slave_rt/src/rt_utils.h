/* Copyright 2024. All Rights Reserved. */
/* Linux-RT Real-time Utilities */

#ifndef RT_UTILS_H
#define RT_UTILS_H

#include <pthread.h>
#include <stdint.h>

// ========================================================================
// Thread Priorities
// ========================================================================

#define RT_PRI_RTU     99   // RTU 采集线程 - 最高优先级
#define RT_PRI_TCP     90   // TCP 响应线程 - 高优先级
#define RT_PRI_SYNC    80   // 数据同步线程 - 中优先级
#define RT_PRI_LOG     50   // 日志线程 - 低优先级

// ========================================================================
// Memory Locking
// ========================================================================

// Lock all current and future memory (prevent swapping)
int rt_lock_memory(void);

// Unlock memory
int rt_unlock_memory(void);

// ========================================================================
// Real-time Thread Creation
// ========================================================================

// Create a real-time thread with SCHED_FIFO policy
// thread: output thread handle
// start_routine: thread function
// arg: argument to thread function
// priority: SCHED_FIFO priority (1-99)
// stack_size: stack size in bytes (0 = default)
int rt_create_thread(pthread_t *thread,
                     void *(*start_routine)(void*),
                     void *arg,
                     int priority,
                     int stack_size);

// Set thread to SCHED_FIFO with given priority
int rt_set_thread_sched(pthread_t thread, int policy, int priority);

// Get current time in microseconds
uint64_t rt_get_time_us(void);

// Get current time in milliseconds
uint64_t rt_get_time_ms(void);

// Busy wait for specified microseconds
void rt_busy_wait_us(uint64_t us);

// ========================================================================
// Periodic Task
// ========================================================================

// Sleep until absolute time (for periodic tasks)
// Returns 0 on success, -1 on error
int rt_sleep_until(struct timespec *abs_time);

// Calculate next period time
void rt_add_timespec(struct timespec *ts, uint64_t us);

// ========================================================================
// Logging with timestamps
// ========================================================================

#define RT_LOG_DEBUG(fmt, ...) \
    do { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); \
         printf("[RT-DEBUG %ld.%06ld] " fmt "\n", ts.tv_sec, ts.tv_nsec/1000, ##__VA_ARGS__); } while(0)

#define RT_LOG_INFO(fmt, ...) \
    do { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); \
         printf("[RT-INFO  %ld.%06ld] " fmt "\n", ts.tv_sec, ts.tv_nsec/1000, ##__VA_ARGS__); } while(0)

#define RT_LOG_WARN(fmt, ...) \
    do { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); \
         printf("[RT-WARN  %ld.%06ld] " fmt "\n", ts.tv_sec, ts.tv_nsec/1000, ##__VA_ARGS__); } while(0)

#define RT_LOG_ERROR(fmt, ...) \
    do { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); \
         printf("[RT-ERROR %ld.%06ld] " fmt "\n", ts.tv_sec, ts.tv_nsec/1000, ##__VA_ARGS__); } while(0)

#endif /* RT_UTILS_H */
