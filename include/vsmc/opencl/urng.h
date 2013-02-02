#ifndef VSMC_OPENCL_URNG_H
#define VSMC_OPENCL_URNG_H

#if defined(cl_khr_fp64) || defined(cl_amd_fp64)
#define R123_USE_U01_DOUBLE 1
#else
#define R123_USE_U01_DOUBLE 0
#endif

#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#ifndef R123_STATIC_INLINE
#define R123_STATIC_INLINE static inline
#endif
#endif

#ifndef CBRNG2x32
#define CBRNG2x32 threefry2x32
#endif

#ifndef CBRNG2x64
#define CBRNG2x64 threefry2x64
#endif

#ifndef CBRNG4x32
#define CBRNG4x32 threefry4x32
#endif

#ifndef CBRNG4x64
#define CBRNG4x64 threefry4x64
#endif

#if VSMC_STATE_TYPE_IS_FLOAT
#define U01_OPEN_OPEN_32     u01_open_open_32_24
#define U01_OPEN_CLOSED_32   u01_open_closed_32_24
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_24
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_24
#endif // VSMC_STATE_TYPE_IS_FLOAT

#if VSMC_STATE_TYPE_IS_DOUBLE
#define U01_OPEN_OPEN_32     u01_open_open_32_53
#define U01_OPEN_CLOSED_32   u01_open_closed_32_53
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_53
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_53

#define U01_OPEN_OPEN_64     u01_open_open_64_53
#define U01_OPEN_CLOSED_64   u01_open_closed_64_53
#define U01_CLOSED_OPEN_64   u01_closed_closed_64_53
#define U01_CLOSED_CLOSED_64 u01_closed_closed_64_53
#endif // VSMC_STATE_TYPE_IS_DOUBLE

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>

#define VSMC_DEFINE_CBURNG(N, W) \
    typedef struct {                                                         \
        struct r123array##N##x##W key;                                       \
        struct r123array##N##x##W ctr;                                       \
        struct r123array##N##x##W rnd;                                       \
        unsigned char remain;                                                \
    } cburng##N##x##W;

#define VSMC_DEFINE_CBURNG_INIT(N, W) \
    VSMC_STATIC_INLINE void cburng##N##x##W##_init(cburng##N##x##W *rng)     \
    {                                                                        \
        struct r123array##N##x##W v = {{0}};                                 \
        rng->key = rng->ctr = rng->rnd = v;                                  \
        rng->remain = 0;                                                     \
    }

#define VSMC_DEFINE_CBURNG_RAND(N, W) \
    VSMC_STATIC_INLINE uint##W##_t cburng##N##x##W##_rand(                   \
            cburng##N##x##W *rng)                                            \
    {                                                                        \
        if (!rng->remain) {                                                  \
            rng->ctr.v[0]++;                                                 \
            rng->rnd = CBRNG##N##x##W(rng->ctr, rng->key);                   \
            rng->remain = N;                                                 \
        }                                                                    \
        rng->remain--;                                                       \
                                                                             \
        return rng->rnd.v[rng->remain];                                      \
    }

/// \ingroup OpenCL
VSMC_DEFINE_CBURNG(2, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG(2, 64)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG(4, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG(4, 64)

/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_INIT(2, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_INIT(2, 64)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_INIT(4, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_INIT(4, 64)

/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_RAND(2, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_RAND(2, 64)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_RAND(4, 32)
/// \ingroup OpenCL
VSMC_DEFINE_CBURNG_RAND(4, 64)

#endif // VSMC_OPENCL_URNG_H
