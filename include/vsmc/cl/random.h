#ifndef VSMC_CL_RANDOM_H
#define VSMC_CL_RANDOM_H

#include <vsmc/cl/config.h>

#if VSMC_USE_RANDOM123
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#endif

typedef struct r123array2x32 CBRNG2x32_KEY_T;
typedef struct r123array2x32 CBRNG2x32_CTR_T;

typedef struct r123array2x64 CBRNG2x64_KEY_T;
typedef struct r123array2x64 CBRNG2x64_CTR_T;

typedef struct r123array4x32 CBRNG4x32_KEY_T;
typedef struct r123array4x32 CBRNG4x32_CTR_T;

typedef struct r123array4x64 CBRNG4x64_KEY_T;
typedef struct r123array4x64 CBRNG4x64_CTR_T;

#define VSMC_DEFINE_NORMAL01(W, F, FT) \
    typedef struct {                                                         \
        CBRNG4x##W##_KEY_T *k;                                               \
        CBRNG4x##W##_CTR_T *c;                                               \
        CBRNG4x##W##_CTR_T r;                                                \
        FT##4 u;                                                             \
        FT##4 z;                                                             \
        uint8_t count;                                                       \
    } normal01_##W##_##F;                                                    \
                                                                             \
    void normal01_##W##_##F##_init (normal01_##W##_##F *,                    \
            CBRNG4x##W##_KEY_T *, CBRNG4x##W##_CTR_T *);                     \
    FT normal01_##W##_##F##_rand (normal01_##W##_##F *);                     \
                                                                             \
    void normal01_##W##_##F##_init (normal01_##W##_##F *norm,                \
            CBRNG4x##W##_KEY_T *k, CBRNG4x##W##_CTR_T *c)                    \
    {                                                                        \
        norm->k = k;                                                         \
        norm->c = c;                                                         \
        norm->count = 0;                                                     \
    }                                                                        \
                                                                             \
    FT normal01_##W##_##F##_rand (normal01_##W##_##F *norm)                  \
    {                                                                        \
        if (!norm->count) {                                                  \
            norm->c->v[0]++;                                                 \
            norm->r = CBRNG4x##W(*(norm->c), *(norm->k));                    \
            norm->u.x = u01_open_closed_##W##_##F(norm->r.v[0]);             \
            norm->u.y = u01_open_closed_##W##_##F(norm->r.v[1]);             \
            norm->u.z = u01_open_closed_##W##_##F(norm->r.v[2]);             \
            norm->u.w = u01_open_closed_##W##_##F(norm->r.v[3]);             \
            norm->count = 4;                                                 \
        }                                                                    \
                                                                             \
        --norm->count;                                                       \
        switch (norm->count) {                                               \
            case 0 :                                                         \
                return sqrt(-2 * log(norm->u.x)) *                           \
                    cos(2 * M_PI_##F * norm->u.y);                           \
            case 1 :                                                         \
                return sqrt(-2 * log(norm->u.x)) *                           \
                    sin(2 * M_PI_##F * norm->u.y);                           \
            case 2 :                                                         \
                return sqrt(-2 * log(norm->u.z)) *                           \
                    cos(2 * M_PI_##F * norm->u.w);                           \
            case 3 :                                                         \
                return sqrt(-2 * log(norm->u.z)) *                           \
                    sin(2 * M_PI_##F * norm->u.w);                           \
            default :                                                        \
                return 0;                                                    \
        }                                                                    \
    }

VSMC_DEFINE_NORMAL01(32, 24, float);

#if R123_USE_U01_DOUBLE
VSMC_DEFINE_NORMAL01(32, 53, float);
VSMC_DEFINE_NORMAL01(64, 53, double);
#endif

#endif // VSMC_CL_RANDOM_H
