#pragma once

#include <time.h>
#include <errno.h>

inline double pond_get_time()
{
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    return (double)ts.tv_sec + (double)ts.tv_nsec*1e-9;
}

inline double pond_get_monotonic_time()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (double)ts.tv_sec + (double)ts.tv_nsec*1e-9;
}

inline void pond_sleep(double duration)
{
    timespec ts{(__time_t)duration, (__syscall_slong_t)(duration * 1000000000) % 1000000000};

    while (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &ts) == EINTR);
}