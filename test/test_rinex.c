#include <stdio.h>
#include "gps-l1c-sim.h"
#include "rinex.h"

#define NAVFILE "brdc0010.22n"

int main(void) {
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    ionoutc_t ionoutc;

    printf("Reading RINEX nav file: %s\n", NAVFILE);
    int sets = readRinexNavAll(eph, &ionoutc, NAVFILE);

    if (sets < 0) {
        printf("Error: failed to read RINEX file.\n");
        return -1;
    }

    printf("Loaded %d sets of ephemerides\n", sets);

    for (int ieph = 0; ieph < sets; ieph++) {
        for (int sv = 0; sv < MAX_SAT; sv++) {
            if (eph[ieph][sv].vflg) {
                printf("\nSet %d, PRN %d\n", ieph, sv + 1);
                printf("  Toe: week %d sec %.1f\n", eph[ieph][sv].toe.week, eph[ieph][sv].toe.sec);
                printf("  ecc: %.12f\n", eph[ieph][sv].ecc);
                printf("  sqrta: %.6f\n", eph[ieph][sv].sqrta);
                printf("  inc0: %.6f\n", eph[ieph][sv].inc0);
                printf("  OMG0: %.6f\n", eph[ieph][sv].omg0);
                printf("  af0: %.6e\n", eph[ieph][sv].af0);
                printf("  af1: %.6e\n", eph[ieph][sv].af1);
            }
        }
    }

    return 0;
}
