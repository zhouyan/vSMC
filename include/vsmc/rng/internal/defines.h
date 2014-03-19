#ifndef VSMC_OPENCL_INTERNAL_DEFINES_H
#define VSMC_OPENCL_INTERNAL_DEFINES_H

#ifndef VSMC_STATIC_INLINE
#ifdef __OPENCL_VERSION__
  #ifndef UINT64_C
  #define UINT64_C(x) ((ulong)(x##UL))
  #endif
  #if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
    #define VSMC_STATIC_INLINE static inline
  #else
    #define VSMC_STATIC_INLINE inline
  #endif
#else
  #ifdef VSMC_OPENCL_USE_DOUBLE
  #undef VSMC_OPENCL_USE_DOUBLE
  #endif
  #define VSMC_OPENCL_USE_DOUBLE 1
  #include <stdint.h>
  #ifndef UINT64_C
    #error __STDC_CONSTANT_MACROS not defined before #include<stdint.h>
  #endif
  #ifdef __cplusplus
    #define VSMC_STATIC_INLINE inline
    #include <cmath>
  #else
    #if defined(__STDC_VERSION__) && __STDC_VERSION__ > 199901L
      #define VSMC_STATIC_INLINE static inline
    #else
      #error VSMC_STATIC_INLINE not defined
    #endif
    #include <math.h>
  #endif
#endif
#endif // VSMC_STATIC_INLINE

/// \brief Enable <vsmc/rng/u01.h> etc., double precision when used with vSMC
/// \ingroup Config
#ifndef VSMC_OPENCL_USE_DOUBLE
#define VSMC_OPENCL_USE_DOUBLE 0
#endif

#endif // VSMC_OPENCL_INTERNAL_DEFINES_H
