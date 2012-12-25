#ifndef VSMC_UTILITY_RANDOM_NORMAL01_H
#define VSMC_UTILITY_RANDOM_NORMAL01_H

#include <vsmc/utility/random/common.h>

#define VSMC_DEFINE_NORMAL01(W, F, FT) \
    typedef struct {                                                         \
        CBRNG4x##W##_KEY_T *k;                                               \
        CBRNG4x##W##_CTR_T *c;                                               \
        CBRNG4x##W##_CTR_T r;                                                \
        FT u[4];                                                             \
        uint8_t count;                                                       \
    } normal01_##W##_##F;                                                    \
                                                                             \
    VSMC_STATIC_INLINE void normal01_##W##_##F##_init (                      \
            normal01_##W##_##F *norm,                                        \
            CBRNG4x##W##_KEY_T *k, CBRNG4x##W##_CTR_T *c)                    \
    {                                                                        \
        norm->k = k;                                                         \
        norm->c = c;                                                         \
        norm->count = 0;                                                     \
    }                                                                        \
                                                                             \
    VSMC_STATIC_INLINE FT normal01_##W##_##F##_rand (                        \
            normal01_##W##_##F *norm)                                        \
    {                                                                        \
        if (!norm->count) {                                                  \
            norm->c->v[0]++;                                                 \
            norm->r = CBRNG4x##W(*(norm->c), *(norm->k));                    \
            norm->u[0] = u01_open_closed_##W##_##F(norm->r.v[0]);            \
            norm->u[1] = u01_open_closed_##W##_##F(norm->r.v[1]);            \
            norm->u[2] = u01_open_closed_##W##_##F(norm->r.v[2]);            \
            norm->u[3] = u01_open_closed_##W##_##F(norm->r.v[3]);            \
            norm->count = 4;                                                 \
        }                                                                    \
                                                                             \
        --norm->count;                                                       \
        switch (norm->count) {                                               \
            case 0 :                                                         \
                return sqrt(-2 * log(norm->u[0])) *                          \
                    cos(2 * M_PI_##F * norm->u[1]);                          \
            case 1 :                                                         \
                return sqrt(-2 * log(norm->u[0])) *                          \
                    sin(2 * M_PI_##F * norm->u[1]);                          \
            case 2 :                                                         \
                return sqrt(-2 * log(norm->u[2])) *                          \
                    cos(2 * M_PI_##F * norm->u[3]);                          \
            case 3 :                                                         \
                return sqrt(-2 * log(norm->u[2])) *                          \
                    sin(2 * M_PI_##F * norm->u[3]);                          \
            default :                                                        \
                return 0;                                                    \
        }                                                                    \
    }

VSMC_DEFINE_NORMAL01(32, 24, float);

#if R123_USE_U01_DOUBLE
VSMC_DEFINE_NORMAL01(32, 53, float);
VSMC_DEFINE_NORMAL01(64, 53, double);
#endif

#endif // VSMC_UTILITY_RANDOM_NORMAL01_H
