#include "gpssim_lib.h"
#include <math.h>

void date2gps(const datetime_t *t, gpstime_t *g)
{
    /* Days since 1980/1/6 */
    int y = t->y;
    int m = t->m;
    int d = t->d;

    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    int a = y / 100;
    int b = 2 - a + (a / 4);

    long jd = (long)(365.25 * (y + 4716))
            + (long)(30.6001 * (m + 1))
            + d + b - 1524.5;

    double day_frac = (t->hh * 3600.0 + t->mm * 60.0 + t->sec) / 86400.0;
    double jd_full = jd + day_frac;

    /* GPS epoch = Jan 6, 1980 (Julian Day 2444244.5) */
    double days = jd_full - 2444244.5;

    g->week = (int)(days / 7.0);
    g->sec = fmod(days, 7.0) * 86400.0;
}
