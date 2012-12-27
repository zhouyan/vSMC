#ifndef VSMC_INTERNAL_COMPILER_OPENCL_H
#define VSMC_INTERNAL_COMPILER_OPENCL_H

#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#ifndef VSMC_STATIC_INLINE
#define VSMC_STATIC_INLINE static inline
#endif
#else // OpenCL 1.2
#ifndef VSMC_STATIC_INLINE
#define VSMC_STATIC_INLINE inline
#endif
#endif // OpenCL 1.2

#if defined(cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#elif defined(cl_amd_fp64)
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#endif

#endif // VSMC_INTERNAL_COMPILER_OPENCL_H
