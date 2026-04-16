/* Copyright 2024. All Rights Reserved. */
/* Linux-RT Real-time Utilities Implementation */

#include "rt_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

// ========================================================================
// Memory Locking
// ========================================================================

int rt_lock_memory(void) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("[RT] mlockall failed");
        return -1;
    }
    printf("[RT] Memory locked (MCL_CURRENT | MCL_FUTURE)\n");
    return 0;
}

int rt_unlock_memory(void) {
    if (munlockall() != 0) {
        perror("[RT] munlockall failed");
        return -1;
    }
    return 0;
}

// ========================================================================
// Real-time Thread Creation
// ========================================================================

int rt_create_thread(pthread_t *thread,
                    void *(*start_routine)(void*),
                    void *arg,
                    int priority,
                    int stack_size) {
    pthread_attr_t attr;
    struct sched_param param;

    if (pthread_attr_init(&attr) != 0) {
        perror("[RT] pthread_attr_init failed");
        return -1;
    }

    // Set stack size
    if (stack_size > 0) {
        if (pthread_attr_setstacksize(&attr, stack_size) != 0) {
            perror("[RT] pthread_attr_setstacksize failed");
            pthread_attr_destroy(&attr);
            return -1;
        }
    }

    // Set SCHED_FIFO policy (real-time scheduling)
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
        perror("[RT] pthread_attr_setschedpolicy failed");
        pthread_attr_destroy(&attr);
        return -1;
    }

    // Set priority
    param.sched_priority = priority;
    if (pthread_attr_setschedparam(&attr, &param) != 0) {
        perror("[RT] pthread_attr_setschedparam failed");
        pthread_attr_destroy(&attr);
        return -1;
    }

    // Use explicit scheduling parameters (don't inherit from parent)
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
        perror("[RT] pthread_attr_setinheritsched failed");
        pthread_attr_destroy(&attr);
        return -1;
    }

    // Create thread
    if (pthread_create(thread, &attr, start_routine, arg) != 0) {
        perror("[RT] pthread_create failed");
        pthread_attr_destroy(&attr);
        return -1;
    }

    pthread_attr_destroy(&attr);

    printf("[RT] Thread created (priority=%d, stack=%d)\n",
           priority, stack_size > 0 ? stack_size : 0);
    return 0;
}

int rt_set_thread_sched(pthread_t thread, int policy, int priority) {
    struct sched_param param;
    param.sched_priority = priority;

    if (pthread_setschedparam(thread, policy, &param) != 0) {
        perror("[RT] pthread_setschedparam failed");
        return -1;
    }

    return 0;
}

// ========================================================================
// Time Functions
// ========================================================================

uint64_t rt_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

uint64_t rt_get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000;
}

void rt_busy_wait_us(uint64_t us) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    uint64_t elapsed = 0;
    while (elapsed < us) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t elapsed_sec = now.tv_sec - start.tv_sec;
        uint64_t elapsed_nsec = now.tv_nsec - start.tv_nsec;
        elapsed = elapsed_sec * 1000000ULL + elapsed_nsec / 1000;
    }
}

// ========================================================================
// Sleep Functions
// ========================================================================

int rt_sleep_until(struct timespec *abs_time) {
    int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, abs_time, NULL);
    if (ret != 0 && ret != EINTR) {
        perror("[RT] clock_nanosleep failed");
        return -1;
    }
    return 0;
}

void rt_add_timespec(struct timespec *ts, uint64_t us) {
    ts->tv_sec += us / 1000000;
    ts->tv_nsec += (us % 1000000) * 1000;

    // Normalize
    while (ts->tv_nsec >= 1000000000) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000;
    }
}
