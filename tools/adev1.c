/*
 * This tool downloaded from http://www.leapsecond.com/tools/adev1.c
 *
 */

//
// adev1.exe
//
// Given raw time-interval (phase) data compute both normal
// and overlapping Allan deviation statistics.
//
// 09-Jun-2000 /tvb
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char Usage[] =
    "\n"
    "Compute Allan Deviation over all tau from raw phase data\n"
    "\n"
    "Usage: adev1 period [scale] [bins] < input\n"
    "\n"
    "Arguments:\n"
    "    period  - data sample interval (e.g., 1 second, 60 seconds)\n"
    "    scale   - input data scale factor if units aren't seconds\n"
    "    bins    - adev points per decade (special: 0,1,2,3,4,5,10,29)\n"
    "\n"
    "Examples:\n"
    "    adev1 1 < 1hzdata.txt         [ pre-scaled 1 Hz TI readings ]\n"
    "    adev1 100 < hpdata.txt        [ 100 sec HP time interval readings ]\n"
    "    adev1 60 1e-6 < mins.txt      [ 1 minute phase data, 1 us units ]\n"
    "    adev1 1 200e-9 < tsc.dat      [ TSC 5110A 200 ns 1 Hz phase data ]\n"
    "    adev1 1 2e-7 50 <tsc.dat      [ 50 adev points per decade ]\n"
    "\n"
    "Help: http://www.LeapSecond.com/tools/adev1.htm\n"
    ;

//
// Get command line options, read data from stdin, and compute ADEV.
//

double *read_data (double scale, long *count);
void adev_all_tau (double *phase, long count, long period, int bins);

int main (int argc, char *argv[])
{
    int bins;
    long period, count;
    double scale;
    double *data;

    if (argc == 1) {
        fprintf(stderr, Usage);
        exit(1);
    }

    period = (argc > 1) ? atol(argv[1]) : 1;
    scale = (argc > 2) ? atof(argv[2]) : 1.0;
    bins = (argc > 3) ? atoi(argv[3]) : 5;

    if (period < 1) {
        fprintf(stderr, "Error: period should be 1 second or more: %ld\n", period);
        exit(1);
    }
    if (scale <= 0.0) {
        fprintf(stderr, "Error: scale factor should be positive: %lg\n", scale);
        exit(1);
    }
    if (bins < 0) {
        fprintf(stderr, "Error: number of bins should be positive: %d\n", bins);
        exit(1);
    }

    printf("\n");
    printf("** Sampling period: %ld s\n", period);
    printf("** Phase data scale factor: %.3le\n", scale);
    data = read_data(scale, &count);
    if (count < 6) {
        fprintf(stderr, "Error: too few samples: %ld\n", count);
        exit(1);
    }
    printf("** Total phase samples: %ld\n", count);
    printf("** Normal and Overlapping Allan deviation:\n");
    printf("\n");
    adev_all_tau(data, count, period, bins);

    return 0;
}

//
// Read entire data series into auto-sizing array.
//

#define MEM_CHECK(p, s) \
     if (p == NULL) { \
        fprintf(stderr, "File %s line %d: malloc failed (size = %ld)\n", __FILE__, __LINE__, s); \
        exit(1); \
    }

double hp53131 (char *line);

double *read_data (double scale, long *count)
{
    double *array;
    long cells, n;
    char line[1000];
    double value, units;

    cells = 1000;
    array = (double *) malloc(sizeof(double) * cells);
    MEM_CHECK(array, sizeof(double) * cells);

    n = 0;
    while (fgets(line, sizeof line, stdin) != NULL) {
        if ((units = hp53131(line)) == 0.0) {
            continue;
        }
        if (units != 1.0 && scale != 1.0) {
            fprintf(stderr, "Error: user-specified scale conflicts with phase units in data: %s\n", line);
            exit(1);
        }
        sscanf(line, "%lf", &value);
        if (n == cells) {
            cells *= 2;
            array = (double *) realloc(array, sizeof(double) * cells);
            MEM_CHECK(array, sizeof(double) * cells);
        }
        array[n] = value * units * scale;
        n += 1;
    }

    *count = n;
    return array;
}

//
// Compute Allan deviation statistics across a full range of tau.
//

double adev1 (double *phase, long count, long tau, long *pairs, int overlap);
long next_tau (long tau, int bins);

void adev_all_tau (double *phase, long count, long period, int bins)
{
    double adev, oadev;
    char buf1[100], buf2[100];
    long n, on, tau;

    tau = 1;
    while (1) {
        adev = adev1(phase, count, tau, &n, 0);
        oadev = adev1(phase, count, tau, &on, 1);
        if (on < 4) {
            break;
        }
        sprintf(buf1, "%8ld tau, %.4le adev(n=%ld), ", tau * period, adev / period, n);
        sprintf(buf2, "%.4le oadev(n=%ld)", oadev / period, on);
        printf("%-44s%s\n", buf1, buf2);
        tau = next_tau(tau, bins);
    }
}

//
// For given sample interval (tau) compute the Allan deviation or
// Overlapping Allan deviation.
//

double adev1 (double *phase, long count, long tau, long *pairs, int overlap)
{
    long i, n;
    double sum, f1, f2, df, adev;

    n = 0;
    sum = 0.0;
    for (i = tau; i + tau < count; i += overlap ? 1 : tau) {
        f1 = phase[i + tau] - phase[i];
        f2 = phase[i] - phase[i - tau];
        df = f1 - f2;
        sum += df * df;
        n += 1;
    }
    *pairs = n;
    adev = (n > 0) ? sqrt(sum / (2.0 * n)) / tau : 0.0;
    return adev;
}

//
// Get next tau to calculate. Some tools calculate Allan deviation
// for only one point per decade; others two or three. Below are
// several unique alternatives that produce cleaner-looking plots.
//

long next_tau (long tau, int bins)
{
    long pow10, n;

    switch (bins) {

    case 0 :    // all tau (not practical)

        return tau + 1;

    case 1 :    // one per decade

        return tau * 10;

    case 2 :    // one per octave

        return tau * 2;

    case 3 :    // 3 dB
    case 4 :    // 1-2-4 decade
    case 5 :    // 1-2-5 decade
    case 10 :   // ten per decade

        pow10 = 1;
        while (tau >= 10) {
            pow10 *= 10;
            tau /= 10;
        }
        if (bins == 3) {
            return ((tau == 3) ? 10 : 3) * pow10;
        }
        if (bins == 4 && tau == 4) {
            return 10 * pow10;
        }
        if (bins == 5 && tau == 2) {
            return 5 * pow10;
        }
        if (bins == 10) {
            return (tau + 1) * pow10;
        }
        return tau * 2 * pow10;

    case 29 :    // 29 nice round numbers per decade

        pow10 = 1;
        while (tau > 100) {
            pow10 *= 10;
            tau /= 10;
        }
        if (tau < 22) {
            return (tau + 1) * pow10;

        } else if (tau < 40) {
            return (tau + 2) * pow10;

        } else if (tau < 60) {
            return (tau + 5) * pow10;

        } else {
            return (tau + 10) * pow10;
        }

    default :   // logarithmically evenly spaced divisions

        n = (long) (log10(tau) * (double)bins + 0.5) + 1;
        n = (long) pow(10.0, (double)n / (double)bins);
        return (n > tau) ? n : tau + 1;
    }
}

//
// Handle HP/Agilent 53131A/53132A RS-232 output format.
//

double hp53131 (char *line)
{
    char *p;

    // Ignore user comments
    if (line[0] == '#') {
        return 0;
    }

    // Ignore HP 53131A statistics
    if (strstr(line, ": ") != NULL) {
        return 0;
    }

    // Remove embedded commas (US version HP 53131A)
    while ((p = strchr(line, ',')) != NULL) {
        strcpy(p, p + 1);
    }

    // Handle HP microsecond and second units
    if (strstr(line, " us") != NULL) {
        return 1e-6;

    } else if (strstr(line, " s") != NULL) {
        return 1.0;
    }

    return 1.0;
}
