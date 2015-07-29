/*
 * stopwatch.h
 *
 *  Created on: 23/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <qglobal.h>
#ifdef Q_OS_MACX
#   include <mach/mach_time.h>
#elif defined(Q_OS_UNIX)
#   include <sys/time.h>
#elif defined(Q_OS_WIN)
#   include <windows.h>
#endif

namespace SLAM {
namespace Support {

#ifdef Q_OS_MACX
class Stopwatch {
    uint64_t total, tstart;
    double conversion;

public:
    inline Stopwatch() : total(0) {
        mach_timebase_info_data_t info;
        mach_timebase_info(&info);
        conversion = info.numer * 1e-9 / info.denom;
    }

    inline Stopwatch(const Stopwatch &s) :
        total(s.total), tstart(s.tstart), conversion(s.conversion) {
    }

    inline Stopwatch &operator=(const Stopwatch &s) {
        total = s.total;
        tstart = s.tstart;
        conversion = s.conversion;
        return *this;
    }

    inline void start() {
        tstart = mach_absolute_time();
    }

    inline void stop() {
        uint64_t tend = mach_absolute_time();
        total += tend - tstart;
    }

    inline void reset() {
        total = 0;
    }

    inline double time() {
        return conversion * total;
    }

};
#elif defined(Q_OS_UNIX)
class Stopwatch {
    struct timeval tstart;
    struct timezone tz;
    double total;

public:
    inline Stopwatch() : total(0) {
    }

    inline Stopwatch(const Stopwatch &s) :
        tstart(s.tstart), tz(s.tz), total(total) {
    }

    inline Stopwatch &operator=(const Stopwatch &s) {
        tstart = s.tstart;
        tz = s.tz;
        total = s.total;
        return *this;
    }

    inline void start() {
        gettimeofday(&tstart, &tz);
    }

    inline void stop() {
        struct timeval tend, tdiff;
        gettimeofday(&tend, &tz);
        timersub(&tend, &tstart, &tdiff);
        total += tdiff.tv_sec + 1e-6 * tdiff.tv_usec;
    }

    inline void reset() {
        total = 0;
    }

    inline double time() {
        return total;
    }

};
#elif defined(Q_OS_WIN)
class Stopwatch {
    __int64 total, tstart;
    double conversion;

public:
    inline Stopwatch() : total(0) {
        LARGE_INTEGER li;
        QueryPerformanceFrequency(&li);
        conversion = 1. / li.QuadPart;
    }

    inline Stopwatch(const Stopwatch &s) :
        total(s.total), tstart(s.tstart), conversion(s.conversion) {
    }

    inline Stopwatch &operator=(const Stopwatch &s) {
        total = s.total;
        tstart = s.tstart;
        conversion = s.conversion;
        return *this;
    }
    inline void start() {
        LARGE_INTEGER li;
        QueryPerformanceCounter(&li);
        tstart = li.QuadPart;
    }

    inline void stop() {
        LARGE_INTEGER li;
        QueryPerformanceCounter(&li);
        total += li.QuadPart - tstart;
    }

    inline void reset() {
        total = 0;
    }

    inline double time() {
        return conversion * total;
    }

};
#else
#   error Missing implementation of SLAM::Support::Stopwatch for this platform
#endif

} /* namespace Support */
} /* namespace SLAM */

#endif /* STOPWATCH_H_ */
