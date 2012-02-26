#ifndef V_SMC_RNG_RANDOM_H
#define V_SMC_RNG_RANDOM_H

#include <Random123/threefry.h>

#ifndef V_SMC_RNG_SEED
#define V_SMC_RNG_SEED 0xdeadbeefL
#endif // V_SMC_RNG_SEED

#include <math.h>

inline void drunif (double min, double max,
        double *r0, double *r1, double *r2, double *r3,
        long c0, long c1, long c2, long c3)
{
    const double multiple = 0.5 * (max - min) / (1UL<<63);

    threefry4x64_ctr_t c = {{}};
    threefry4x64_key_t k = {{}};
    k.v[0] = V_SMC_RNG_SEED;
    c.v[0] = c0;
    c.v[1] = c1;
    c.v[2] = c2;
    c.v[3] = c3;
    union {threefry4x64_ctr_t c; unsigned long n[4];} u;
    u.c = threefry4x64(c, k);

    if (r0) *r0 = u.n[0] * multiple + min;
    if (r1) *r1 = u.n[1] * multiple + min;
    if (r2) *r2 = u.n[2] * multiple + min;
    if (r3) *r3 = u.n[3] * multiple + min;
}

inline void drnorm(double mean, double sd,
        double *r0, double *r1, double *r2, double *r3,
        long c0, long c1, long c2, long c3)
{
    double u0, u1, u2, u3;
    drunif(0, 1, &u0, &u1, &u2, &u3, c0, c1, c2, c3);

    if (r0) *r0 = sqrt(-2 * log(u0)) * cos(2 * M_PI * u1) * sd + mean;
    if (r1) *r1 = sqrt(-2 * log(u0)) * sin(2 * M_PI * u1) * sd + mean;
    if (r2) *r2 = sqrt(-2 * log(u2)) * cos(2 * M_PI * u3) * sd + mean;
    if (r3) *r3 = sqrt(-2 * log(u2)) * sin(2 * M_PI * u3) * sd + mean;
}

#endif // V_SMC_RNG_RANDOM_H
