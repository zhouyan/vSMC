#ifndef VSMC_OPENCL_DEFINES_H
#define VSMC_OPENCL_DEFINES_H

#ifndef VSMC_STATIC_INLINE
#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#define VSMC_STATIC_INLINE static inline
#elif defined(__STDC__)
#define VSMC_STATIC_INLINE static inline
#else
#define VSMC_STATIC_INLINE inline
#endif
#endif

#if defined(VSMC_FP_TYPE_IS_FLOAT) && defined(VSMC_FP_TYPE_IS_DOUBLE)
#if VSMC_FP_TYPE_IS_FLOAT && VSMC_FP_TYPE_IS_DOUBLE
#error VSMC_FP_TYPE_IS_FLOAT and VSMC_FP_TYPE_IS_DOUBLE cannot both be non-zero
#endif
#endif

#endif // VSMC_OPENCL_DEFINES_H
