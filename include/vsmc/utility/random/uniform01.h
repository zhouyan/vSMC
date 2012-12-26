#ifndef VSMC_UTILITY_RANDOM_UNIFORM01_H
#define VSMC_UTILITY_RANDOM_UNIFORM01_H

#include <vsmc/utility/random/common.h>
#include <stdio.h>

#define VSMC_DEFINE_UNIFORM01(W, F, FT) \
    typedef struct {                                                         \
        CBRNG4x##W##_KEY_T *k;                                               \
        CBRNG4x##W##_CTR_T *c;                                               \
        CBRNG4x##W##_CTR_T r;                                                \
        FT u[4];                                                             \
        unsigned char remain;                                                \
    } uniform01_##W##_##F;

#define VSMC_DEFINE_UNIFORM01_STEP(W, F, FT) \
    VSMC_STATIC_INLINE void uniform01_##W##_##F##_step (                     \
            uniform01_##W##_##F *runif)                                      \
    {                                                                        \
        runif->c->v[0]++;                                                    \
        runif->r = CBRNG4x##W(*(runif->c), *(runif->k));                     \
        runif->u[0] = u01_open_closed_##W##_##F(runif->r.v[0]);              \
        runif->u[1] = u01_open_closed_##W##_##F(runif->r.v[1]);              \
        runif->u[2] = u01_open_closed_##W##_##F(runif->r.v[2]);              \
        runif->u[3] = u01_open_closed_##W##_##F(runif->r.v[3]);              \
        runif->remain = 4;                                                   \
    }

#define VSMC_DEFINE_UNIFORM01_INIT(W, F, FT) \
    VSMC_STATIC_INLINE void uniform01_##W##_##F##_init (                     \
            uniform01_##W##_##F *runif,                                      \
            CBRNG4x##W##_KEY_T *k, CBRNG4x##W##_CTR_T *c)                    \
    {                                                                        \
        runif->k = k;                                                        \
        runif->c = c;                                                        \
        uniform01_##W##_##F##_step(runif);                                   \
    }

#define VSMC_DEFINE_UNIFORM01_RAND(W, F, FT) \
    VSMC_STATIC_INLINE FT uniform01_##W##_##F##_rand (                       \
            uniform01_##W##_##F *runif)                                      \
    {                                                                        \
        if (!runif->remain)                                                  \
            uniform01_##W##_##F##_step(runif);                               \
        runif->remain--;                                                     \
                                                                             \
        return runif->u[runif->remain];                                      \
    }

/// \ingroup Random
VSMC_DEFINE_UNIFORM01(32, 24, float);
VSMC_DEFINE_UNIFORM01_STEP(32, 24, float);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_INIT(32, 24, float);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_RAND(32, 24, float);

#if R123_USE_U01_DOUBLE
/// \ingroup Random
VSMC_DEFINE_UNIFORM01(32, 53, float);
VSMC_DEFINE_UNIFORM01_STEP(32, 53, float);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_INIT(32, 53, float);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_RAND(32, 53, float);

/// \ingroup Random
VSMC_DEFINE_UNIFORM01(64, 53, double);
VSMC_DEFINE_UNIFORM01_STEP(64, 53, double);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_INIT(64, 53, double);
/// \ingroup Random
VSMC_DEFINE_UNIFORM01_RAND(64, 53, double);
#endif

#endif // VSMC_UTILITY_RANDOM_UNIFORM01_H
