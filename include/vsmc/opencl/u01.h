#ifndef VSMC_OPENCL_U01_H
#define VSMC_OPENCL_U01_H

#define VSMC_0x1p_31f (1.f/(1024.f*1024.f*1024.f*2.f))
#define VSMC_0x1p_24f (128.f*VSMC_0x1p_31f)
#define VSMC_0x1p_23f (256.f*VSMC_0x1p_31f)
#define VSMC_0x1p_32  (1./(1024.*1024.*1024.*4.))
#define VSMC_0x1p_63  (2.*VSMC_0x1p_32*VSMC_0x1p_32)
#define VSMC_0x1p_53  (1024.*VSMC_0x1p_63)
#define VSMC_0x1p_52  (2048.*VSMC_0x1p_63)

/* narrowing conversions:  uint32_t to float */
VSMC_STATIC_INLINE float u01_closed_closed_32_24 (uint32_t i)
{
    /* N.B.  we ignore the high bit, so output is not monotonic */
    return ((i&0x7fffffc0) + (i&0x40))*VSMC_0x1p_31f; /* 0x1.p-31f */
}

VSMC_STATIC_INLINE float u01_closed_open_32_24 (uint32_t i)
{
    return (i>>8)*VSMC_0x1p_24f; /* 0x1.0p-24f; */
}

VSMC_STATIC_INLINE float u01_open_closed_32_24 (uint32_t i)
{
    return (1+(i>>8))*VSMC_0x1p_24f; /* *0x1.0p-24f; */
}

VSMC_STATIC_INLINE float u01_open_open_32_24 (uint32_t i)
{
    return (0.5f+(i>>9))*VSMC_0x1p_23f; /* 0x1.p-23f; */
}

#if VSMC_USE_U01_DOUBLE
/* narrowing conversions:  uint64_t to double */
VSMC_STATIC_INLINE double u01_closed_closed_64_53 (uint64_t i)
{
    /* N.B.  we ignore the high bit, so output is not monotonic */
    return ((i&R123_64BIT(0x7ffffffffffffe00)) + (i&0x200))*VSMC_0x1p_63; /* 0x1.p-63; */
}

VSMC_STATIC_INLINE double u01_closed_open_64_53 (uint64_t i)
{
    return (i>>11)*VSMC_0x1p_53; /* 0x1.0p-53; */
}

VSMC_STATIC_INLINE double u01_open_closed_64_53 (uint64_t i)
{
    return (1+(i>>11))*VSMC_0x1p_53; /* 0x1.0p-53; */
}

VSMC_STATIC_INLINE double u01_open_open_64_53 (uint64_t i)
{
    return (0.5+(i>>12))*VSMC_0x1p_52; /* 0x1.0p-52; */
}

/* widening conversions:  u32 to double */
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

VSMC_STATIC_INLINE double u01_closed_open_32_53 (uint32_t i)
{
    return i*VSMC_0x1p_32; /* 0x1.p-32; */
}

VSMC_STATIC_INLINE double u01_open_closed_32_53 (uint32_t i)
{
    return (1.+i)*VSMC_0x1p_32; /* 0x1.p-32; */
}

VSMC_STATIC_INLINE double u01_open_open_32_53 (uint32_t i)
{
    return (0.5+i)*VSMC_0x1p_32; /* 0x1.p-32; */
}

#endif // VSMC_USE_U01_DOUBLE


#endif // VSMC_OPENCL_U01_H
