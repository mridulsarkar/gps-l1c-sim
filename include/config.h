
#ifndef CONFIG_H
#define CONFIG_H

/* General constants */
#define MAX_SAT            64      /* Support up to 63 PRNs */
#define EPHEM_ARRAY_SIZE   40      /* Store multiple sets of ephemerides */
#define MAX_CHAR           100     /* Max chars per RINEX line */

/* Time-related constants */
#define SECONDS_IN_HOUR    3600
#define SECONDS_IN_DAY     86400
#define SECONDS_IN_WEEK    604800

/* Earth and GPS system constants */
#define GM_EARTH           3.986005e14    /* Earth's universal gravitation constant [m^3/s^2] */
#define OMEGA_EARTH        7.2921151467e-5 /* Earth rotation rate [rad/s] */

/* Boolean flags */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif /* CONFIG_H */

