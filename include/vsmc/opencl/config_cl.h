#ifndef VSMC_OPENCL_CONFIG_CL_H
#define VSMC_OPENCL_CONFIG_CL_H

#if VSMC_STATE_TYPE_IS_FLOAT
#define U01_OPEN_OPEN_32     u01_open_open_32_24
#define U01_OPEN_CLOSED_32   u01_open_closed_32_24
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_24
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_24

#define NORMAL01_2x32      normal01_2x32_24
#define NORMAL01_2x32_INIT normal01_2x32_24_init
#define NORMAL01_2x32_RAND normal01_2x32_24_rand

#define NORMAL01_4x32      normal01_4x32_24
#define NORMAL01_4x32_INIT normal01_4x32_24_init
#define NORMAL01_4x32_RAND normal01_4x32_24_rand
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

#endif // VSMC_OPENCL_CONFIG_CL_H
