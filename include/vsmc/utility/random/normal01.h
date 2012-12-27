#ifndef VSMC_UTILITY_RANDOM_NORMAL01_H
#define VSMC_UTILITY_RANDOM_NORMAL01_H

#include <vsmc/utility/random/common.h>

#define VSMC_DEFINE_NORMAL01(N, W, F, FT) \
    typedef struct {                                                         \
        FT u1;                                                               \
        FT u2;                                                               \
        unsigned char saved;                                                 \
    } normal01_##N##x##W##_##F;

#define VSMC_DEFINE_NORMAL01_INIT(N, W, F, FT) \
    VSMC_STATIC_INLINE void normal01_##N##x##W##_##F##_init (                \
            normal01_##N##x##W##_##F *rnorm, cburng##N##x##W *rng)           \
    {                                                                        \
        rnorm->u1 = u01_open_closed_##W##_##F(cburng##N##x##W##_rand(rng));  \
        rnorm->u2 = u01_open_closed_##W##_##F(cburng##N##x##W##_rand(rng));  \
        rnorm->saved = 1;                                                    \
    }

#define VSMC_DEFINE_NORMAL01_RAND(N, W, F, FT) \
    VSMC_STATIC_INLINE FT normal01_##N##x##W##_##F##_rand (                  \
            normal01_##N##x##W##_##F *rnorm, cburng##N##x##W *rng)           \
    {                                                                        \
        if (rnorm->saved) {                                                  \
            rnorm->saved = 0;                                                \
            return sqrt(-2 * log(rnorm->u1)) * cos(M_2PI_##F * rnorm->u2);   \
        } else {                                                             \
            normal01_##N##x##W##_##F##_init(rnorm, rng);                     \
            return sqrt(-2 * log(rnorm->u1)) * sin(M_2PI_##F * rnorm->u2);   \
        }                                                                    \
    }

/// \ingroup Random
VSMC_DEFINE_NORMAL01(2, 32, 24, float);
/// \ingroup Random
VSMC_DEFINE_NORMAL01(4, 32, 24, float);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(2, 32, 24, float);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(4, 32, 24, float);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(2, 32, 24, float);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(4, 32, 24, float);

#if R123_USE_U01_DOUBLE
/// \ingroup Random
VSMC_DEFINE_NORMAL01(2, 32, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01(4, 32, 53, double);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(2, 32, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(4, 32, 53, double);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(2, 32, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(4, 32, 53, double);

/// \ingroup Random
VSMC_DEFINE_NORMAL01(2, 64, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01(4, 64, 53, double);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(2, 64, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_INIT(4, 64, 53, double);

/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(2, 64, 53, double);
/// \ingroup Random
VSMC_DEFINE_NORMAL01_RAND(4, 64, 53, double);
#endif

#endif // VSMC_UTILITY_RANDOM_NORMAL01_H
