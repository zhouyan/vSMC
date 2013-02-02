#ifndef VSMC_OPENCL_NORMAL01_H
#define VSMC_OPENCL_NORMAL01_H

#include <vsmc/opencl/urng.h>

#if VSMC_STATE_TYPE_IS_FLOAT
#define NORMAL01_2x32      normal01_2x32_24
#define NORMAL01_2x32_INIT normal01_2x32_24_init
#define NORMAL01_2x32_RAND normal01_2x32_24_rand

#define NORMAL01_4x32      normal01_4x32_24
#define NORMAL01_4x32_INIT normal01_4x32_24_init
#define NORMAL01_4x32_RAND normal01_4x32_24_rand
#endif // VSMC_STATE_TYPE_IS_FLOAT

#if VSMC_STATE_TYPE_IS_DOUBLE
#define NORMAL01_2x32      normal01_2x32_53
#define NORMAL01_2x32_INIT normal01_2x32_53_init
#define NORMAL01_2x32_RAND normal01_2x32_53_rand

#define NORMAL01_2x64      normal01_2x64_53
#define NORMAL01_2x64_INIT normal01_2x64_53_init
#define NORMAL01_2x64_RAND normal01_2x64_53_rand

#define NORMAL01_4x32      normal01_4x32_53
#define NORMAL01_4x32_INIT normal01_4x32_53_init
#define NORMAL01_4x32_RAND normal01_4x32_53_rand

#define NORMAL01_4x64      normal01_4x64_53
#define NORMAL01_4x64_INIT normal01_4x64_53_init
#define NORMAL01_4x64_RAND normal01_4x64_53_rand
#endif // VSMC_STATE_TYPE_IS_DOUBLE

#define M_2PI_24 6.2831853071795865F
#define M_2PI_53 6.2831853071795865

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

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(2, 32, 24, float)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(4, 32, 24, float)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(2, 32, 24, float)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(4, 32, 24, float)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(2, 32, 24, float)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(4, 32, 24, float)

#if R123_USE_U01_DOUBLE
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(2, 32, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(4, 32, 53, double)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(2, 32, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(4, 32, 53, double)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(2, 32, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(4, 32, 53, double)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(2, 64, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01(4, 64, 53, double)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(2, 64, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_INIT(4, 64, 53, double)

/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(2, 64, 53, double)
/// \ingroup OpenCL
VSMC_DEFINE_NORMAL01_RAND(4, 64, 53, double)
#endif

#endif // VSMC_OPENCL_NORMAL01_H
