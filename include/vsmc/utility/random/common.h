#ifndef VSMC_UTILITY_RANDOM_COMMON_H
#define VSMC_UTILITY_RANDOM_COMMON_H

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

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

#define M_2PI_24 6.2831853071795865F
#define M_2PI_53 6.2831853071795865

#include <vsmc/internal/compiler.h>
#include <vsmc/utility/random/urng.h>

#endif // VSMC_UTILITY_RANDOM_COMMON_H
