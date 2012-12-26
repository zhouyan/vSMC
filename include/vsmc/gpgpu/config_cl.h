#ifndef VSMC_GPGPU_CONFIG_CL_H
#define VSMC_GPGPU_CONFIG_CL_H

#if defined(cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#elif defined(cl_amd_fp64)
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#endif

#if VSMC_STATE_TYPE_IS_FLOAT
#define U01_OPEN_OPEN_32     u01_open_open_32_24
#define U01_OPEN_CLOSED_32   u01_open_closed_32_24
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_24
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_24

#define NORMAL01_32      normal01_32_24
#define NORMAL01_32_INIT normal01_32_24_init
#define NORMAL01_32_RAND normal01_32_24_rand
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

#define NORMAL01_32      normal01_32_53
#define NORMAL01_32_INIT normal01_32_53_init
#define NORMAL01_32_RAND normal01_32_53_rand

#define NORMAL01_64      normal01_64_53
#define NORMAL01_64_INIT normal01_64_53_init
#define NORMAL01_64_RAND normal01_64_53_rand
#endif // VSMC_STATE_TYPE_IS_DOUBLE

#endif // VSMC_GPGPU_CONFIG_CL_H
