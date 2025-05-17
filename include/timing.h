#ifndef TIMING_H
#define TIMING_H

#include <time.h>
#include <errno.h>

struct timespec ts;

void microsleep(int microseconds) {
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        continue;
    }
}

#endif // TIMING_H