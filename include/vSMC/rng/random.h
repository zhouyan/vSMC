#ifndef V_SMC_RNG_RANDOM_H
#define V_SMC_RNG_RANDOM_H

#include <Random123/threefry.h>

#ifndef V_SMC_RNG_SEED
#define V_SMC_RNG_SEED 0xdeadbeefL
#endif // V_SMC_RNG_SEED

#include <math.h>

inline void drunif (double min, double max,
        double *r1, double *r2, double *r3, double *r4,
        long c1, long c2, long c3, long c4)
{
    const double multiple = 0.5 * (max - min) / (1UL<<63);

    threefry4x64_ctr_t c = {{}};
    threefry4x64_key_t k = {{}};
    k.v[0] = V_SMC_RNG_SEED;
    c.v[0] = c1;
    c.v[1] = c2;
    c.v[2] = c3;
    c.v[3] = c4;
    union {threefry4x64_ctr_t c; unsigned long n[4];} u;
    u.c = threefry4x64(c, k);

    if (r1)
        *r1 = u.n[0] * multiple + min;
    if (r2)
        *r2 = u.n[1] * multiple + min;
    if (r3)
        *r3 = u.n[2] * multiple + min;
    if (r4)
        *r4 = u.n[3] * multiple + min;
}

inline void drnorm(double mean, double sd,
        double *r1, double *r2, double *r3, double *r4,
        long c1, long c2, long c3, long c4)
{
    double u1, u2, u3, u4;
    drunif(0, 1, &u1, &u2, &u3, &u4, c1, c2, c3, c4);

    if (r1)
        *r1 = sqrt(-2 * log(u1)) * cos(2 * M_PI * u2) * sd + mean;
    if (r2)
        *r2 = sqrt(-2 * log(u1)) * sin(2 * M_PI * u2) * sd + mean;
    if (r3)
        *r3 = sqrt(-2 * log(u3)) * cos(2 * M_PI * u4) * sd + mean;
    if (r4)
        *r4 = sqrt(-2 * log(u3)) * sin(2 * M_PI * u4) * sd + mean;
}

#endif // V_SMC_RNG_RANDOM_H
