#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rinex.h"

#define MAX_EPHEM 1000

nav_t gnav;
int neph = 0;
static int rinex_version = 2; // default

/* Replace D with E in scientific notation */
void replaceExpDesignator(char *s, int n)
{
    for (int i = 0; i < n; i++)
    {
        if (s[i] == 'D' || s[i] == 'd')
            s[i] = 'E';
    }
}

/* Read header and detect RINEX version */
static int read_rinex_header(FILE *fp, ionoutc_t *ionoutc)
{
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "RINEX VERSION / TYPE")) {
            char ver[4];
            strncpy(ver, line, 3);
            ver[3] = '\0';
            rinex_version = atoi(ver);
        }
        else if (strstr(line, "ION ALPHA")) {
            sscanf(line, "%lE%lE%lE%lE",
                &ionoutc->alpha0, &ionoutc->alpha1,
                &ionoutc->alpha2, &ionoutc->alpha3);
            ionoutc->vflg = 1;
        }
        else if (strstr(line, "ION BETA")) {
            sscanf(line, "%lE%lE%lE%lE",
                &ionoutc->beta0, &ionoutc->beta1,
                &ionoutc->beta2, &ionoutc->beta3);
        }
        else if (strstr(line, "DELTA-UTC: A0,A1,T,W")) {
            sscanf(line, "%lE%lE%d%d",
                &ionoutc->A0, &ionoutc->A1,
                &ionoutc->tot, &ionoutc->wnt);
        }
        else if (strstr(line, "LEAP SECONDS")) {
            sscanf(line, "%d", &ionoutc->dtls);
        }
        else if (strstr(line, "END OF HEADER"))
            return 1;
    }
    return 0;
}

/* Parse RINEX-2 ephemeris block */
static void parse_rinex2_block(char *line, FILE *fp, ephem_t eph[][MAX_SAT], int ieph)
{
    int prn;
    sscanf(line, "%2d", &prn);
    if (prn < 1 || prn > MAX_SAT) return;
    
    int year, month, day, hour, minute;
    double second;
    sscanf(line + 2, "%d %d %d %d %d %lf", 
           &year, &month, &day, &hour, &minute, &second);

    // Store time in both datetime and GPS time
    eph[ieph][prn-1].t.y = year;
    eph[ieph][prn-1].t.m = month;
    eph[ieph][prn-1].t.d = day;
    eph[ieph][prn-1].t.hh = hour;
    eph[ieph][prn-1].t.mm = minute;
    eph[ieph][prn-1].t.sec = second;
    
    // Convert to GPS time
    date2gps(&eph[ieph][prn-1].t, &eph[ieph][prn-1].toe);

    // Clock parameters
    eph[ieph][prn-1].af0 = atof(line + 22);
    eph[ieph][prn-1].af1 = atof(line + 41);
    eph[ieph][prn-1].af2 = atof(line + 60);

    // Read next 7 lines for orbital parameters
    char temp[256];
    
    // Line 1
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].iode = (int)atof(temp + 3);
    eph[ieph][prn-1].crs = atof(temp + 22);
    eph[ieph][prn-1].deltan = atof(temp + 41);
    eph[ieph][prn-1].m0 = atof(temp + 60);

    // Line 2
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].cuc = atof(temp + 3);
    eph[ieph][prn-1].ecc = atof(temp + 22);
    eph[ieph][prn-1].cus = atof(temp + 41);
    eph[ieph][prn-1].sqrta = atof(temp + 60);

    // Line 3
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].toe.sec = atof(temp + 3);
    eph[ieph][prn-1].cic = atof(temp + 22);
    eph[ieph][prn-1].omg0 = atof(temp + 41);
    eph[ieph][prn-1].cis = atof(temp + 60);

    // Line 4
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].inc0 = atof(temp + 3);
    eph[ieph][prn-1].crc = atof(temp + 22);
    eph[ieph][prn-1].aop = atof(temp + 41);
    eph[ieph][prn-1].omgdot = atof(temp + 60);

    // Line 5
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].idot = atof(temp + 3);
    eph[ieph][prn-1].codeL2 = (int)atof(temp + 22);
    eph[ieph][prn-1].toe.week = (int)atof(temp + 41);
    
    // Line 6
    fgets(temp, sizeof(temp), fp);
    replaceExpDesignator(temp, strlen(temp));
    eph[ieph][prn-1].svhlth = (int)atof(temp + 22);
    eph[ieph][prn-1].tgd = atof(temp + 41);
    eph[ieph][prn-1].iodc = (int)atof(temp + 60);

    // Line 7
    fgets(temp, sizeof(temp), fp);
    
    // Set valid flag
    eph[ieph][prn-1].vflg = 1;
}

/* Read ephemeris from nav file */
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)
{
    FILE *fp;
    char line[256];

    // Initialize outputs
    memset(eph, 0, sizeof(ephem_t) * EPHEM_ARRAY_SIZE * MAX_SAT);
    memset(ionoutc, 0, sizeof(ionoutc_t));

    printf("Reading RINEX nav file: %s\n", fname);
    fp = fopen(fname, "r");
    if (!fp) {
        printf("Error: failed to open RINEX file.\n");
        return -1;
    }

    if (!read_rinex_header(fp, ionoutc)) {
        printf("Error: failed to read RINEX header.\n");
        fclose(fp);
        return -1;
    }

    int ieph = 0;
    while (fgets(line, sizeof(line), fp) && ieph < EPHEM_ARRAY_SIZE) {
        replaceExpDesignator(line, strlen(line));

        if (rinex_version == 2) {
            if (line[0] != ' ') {
                parse_rinex2_block(line, fp, eph, ieph);
                ieph++;
            }
        }
        // Add RINEX-3 support here if needed
    }

    fclose(fp);
    printf("Loaded %d sets of ephemerides\n", ieph);
    return ieph;
}
