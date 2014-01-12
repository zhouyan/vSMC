#ifndef VSMC_OPENCL_DEFINES_H
#define VSMC_OPENCL_DEFINES_H

#ifndef VSMC_STATIC_INLINE
#ifdef __OPENCL_VERSION__
  #if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
    #define VSMC_STATIC_INLINE static inline
  #else
    #define VSMC_STATIC_INLINE inline
  #endif
#else
  #ifdef __cplusplus
    #define VSMC_STATIC_INLINE inline
  #else
    #if defined(__STDC_VERSION__) && __STDC_VERSION__ > 199901L
      #define VSMC_STATIC_INLINE static inline
    #else
      #error Define VSMC_STATIC_INLINE first
    #endif
  #endif
#endif
#endif // VSMC_STATIC_INLINE

#if defined(VSMC_FP_TYPE_IS_FLOAT) && defined(VSMC_FP_TYPE_IS_DOUBLE)
#if VSMC_FP_TYPE_IS_FLOAT && VSMC_FP_TYPE_IS_DOUBLE
#error VSMC_FP_TYPE_IS_FLOAT and VSMC_FP_TYPE_IS_DOUBLE cannot both be non-zero
#endif
#endif

#ifndef VSMC_FP_TYPE_IS_FLOAT
#define VSMC_FP_TYPE_IS_FLOAT 0
#endif

#ifndef VSMC_FP_TYPE_IS_DOUBLE
#define VSMC_FP_TYPE_IS_DOUBLE 0
#endif

#if !(VSMC_FP_TYPE_IS_FLOAT || VSMC_FP_TYPE_IS_DOUBLE)
#undef VSMC_FP_TYPE_IS_FLOAT
#define VSMC_FP_TYPE_IS_FLOAT 1
#endif

#endif // VSMC_OPENCL_DEFINES_H
