#ifndef VSMC_UTILITY_RANDOM_COMMON_H
#define VSMC_UTILITY_RANDOM_COMMON_H

#ifdef __OPENCL_VERSION__

#if defined(cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#elif defined(cl_amd_fp64)
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#endif

#if defined(cl_khr_fp64) || defined(cl_amd_fp64)
#define R123_USE_U01_DOUBLE 1
#else
#define R123_USE_U01_DOUBLE 0
#endif

#else // __OPENCL_VERSION__

#include <math.h>
#define R123_USE_U01_DOUBLE 1

#endif // __OPENCL_VERSION__

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#include <vsmc/internal/compiler.h>

#define M_PI_24 3.1415926535897932385F
#define M_PI_53 3.1415926535897932385

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
