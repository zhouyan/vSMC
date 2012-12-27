#ifndef VSMC_UTILITY_RANDOM_COMMON_H
#define VSMC_UTILITY_RANDOM_COMMON_H

#ifdef __OPENCL_VERSION__

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

#else // __OPENCL_VERSION__

#include <math.h>
#define R123_USE_U01_DOUBLE 1

#endif // __OPENCL_VERSION__

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#include <vsmc/internal/compiler.h>

#define M_2PI_24 6.2831853071795865F
#define M_2PI_53 6.2831853071795865

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

typedef struct r123array2x32 CBRNG2x32_KEY_T;
typedef struct r123array2x32 CBRNG2x32_CTR_T;

typedef struct r123array2x64 CBRNG2x64_KEY_T;
typedef struct r123array2x64 CBRNG2x64_CTR_T;

typedef struct r123array4x32 CBRNG4x32_KEY_T;
typedef struct r123array4x32 CBRNG4x32_CTR_T;

typedef struct r123array4x64 CBRNG4x64_KEY_T;
typedef struct r123array4x64 CBRNG4x64_CTR_T;

#endif // VSMC_UTILITY_RANDOM_COMMON_H
