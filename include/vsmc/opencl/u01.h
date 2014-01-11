/// \page u01 Uniform real distribution
/// 
/// - Header: `<vsmc/opencl/urng.h>`
/// - Distribution: Uniform distribution on `[0, 1]` and its (semi-) open/closed
///   variants
/// 
/// ### Functions
/// 
/// ~~~{.c}
/// <FT> u01_<bound>_<bound>_<W>_<F> (uint<W>_t);
/// ~~~
/// where `<bound>` is either `open` or `closed`.
/// 
/// For example, to generate uniform random variates on `(0, 1]`,
/// 
/// ### Macros
/// 
/// ~~~{.c}
/// U01_<BOUND>_<BOUND>_<W>
/// ~~~
/// where `<BOUND>` is either `OPEN` or `CLOSED`.
/// 
/// ### Examples
/// 
/// ~~~{.c}
/// #define VSMC_FP_TYPE_IS_DOUBLE 1
/// #include <vsmc/opencl/urng.h>
/// 
/// cburng4x32 rng;
/// cburng4x32_init(&rng);
/// double u_53 = u01_open_closed_32_53(cburng4x32_rand(&rng));
/// 
/// double u = U01_OPEN_CLOSED_32(cburng4x32_rand(&rng));
/// ~~~

#ifndef VSMC_OPENCL_U01_H
#define VSMC_OPENCL_U01_H

#include <vsmc/opencl/urng.h>

#define VSMC_0x1p_31f (1.f/(1024.f*1024.f*1024.f*2.f))
#define VSMC_0x1p_24f (128.f*VSMC_0x1p_31f)
#define VSMC_0x1p_23f (256.f*VSMC_0x1p_31f)
#define VSMC_0x1p_32  (1./(1024.*1024.*1024.*4.))
#define VSMC_0x1p_63  (2.*VSMC_0x1p_32*VSMC_0x1p_32)
#define VSMC_0x1p_53  (1024.*VSMC_0x1p_63)
#define VSMC_0x1p_52  (2048.*VSMC_0x1p_63)

/* narrowing conversions:  uint32_t to float */
/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE float u01_closed_closed_32_24 (uint32_t i)
{
    /* N.B.  we ignore the high bit, so output is not monotonic */
    return ((i&0x7fffffc0) + (i&0x40))*VSMC_0x1p_31f; /* 0x1.p-31f */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE float u01_closed_open_32_24 (uint32_t i)
{
    return (i>>8)*VSMC_0x1p_24f; /* 0x1.0p-24f; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE float u01_open_closed_32_24 (uint32_t i)
{
    return (1+(i>>8))*VSMC_0x1p_24f; /* *0x1.0p-24f; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE float u01_open_open_32_24 (uint32_t i)
{
    return (0.5f+(i>>9))*VSMC_0x1p_23f; /* 0x1.p-23f; */
}

#if VSMC_FP_TYPE_IS_DOUBLE
/* narrowing conversions:  uint64_t to double */
/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_closed_closed_64_53 (uint64_t i)
{
    /* N.B.  we ignore the high bit, so output is not monotonic */
    return ((i&R123_64BIT(0x7ffffffffffffe00)) + (i&0x200))*VSMC_0x1p_63; /* 0x1.p-63; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_closed_open_64_53 (uint64_t i)
{
    return (i>>11)*VSMC_0x1p_53; /* 0x1.0p-53; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_open_closed_64_53 (uint64_t i)
{
    return (1+(i>>11))*VSMC_0x1p_53; /* 0x1.0p-53; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_open_open_64_53 (uint64_t i)
{
    return (0.5+(i>>12))*VSMC_0x1p_52; /* 0x1.0p-52; */
}

/* widening conversions:  u32 to double */
/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_closed_closed_32_53 (uint32_t i)
{
    /* j = i+(i&1) takes on 2^31+1 possible values with a 'trapezoid'
     * distribution:
     * p_j =  1 0 2 0 2 .... 2 0 2 0 1
     * j   =  0 1 2 3 4 ....        2^32
     * by converting to double *before* doing the add, we don't wrap the high
     * bit.
     */
    return (((double)(i&1)) + i)*VSMC_0x1p_32; /* 0x1.p-32; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_closed_open_32_53 (uint32_t i)
{
    return i*VSMC_0x1p_32; /* 0x1.p-32; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_open_closed_32_53 (uint32_t i)
{
    return (1.+i)*VSMC_0x1p_32; /* 0x1.p-32; */
}

/// \ingroup OpenCLRNG
VSMC_STATIC_INLINE double u01_open_open_32_53 (uint32_t i)
{
    return (0.5+i)*VSMC_0x1p_32; /* 0x1.p-32; */
}

#endif // VSMC_FP_TYPE_IS_DOUBLE

#endif // VSMC_OPENCL_U01_H
