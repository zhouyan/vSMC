//============================================================================
// vSMC/include/vsmc/rngc/u01.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RNGC_U01_H
#define VSMC_RNGC_U01_H

#include <vsmc/internal/config.h>

#ifdef VSMC_OPENCL
#define VSMC_RNGC_U01_23F (1.0f / 8388608.0f)
#define VSMC_RNGC_U01_24F (1.0f / 16777216.0f)
#define VSMC_RNGC_U01_31F (1.0f / 2147483648.0f)
#define VSMC_RNGC_U01_32F (1.0f / 4294967296.0f)
#define VSMC_RNGC_U01_33F (1.0f / 8589934592.0f)
#define VSMC_RNGC_U01_64F (1.0f / 18446744073709551616.0f)
#define VSMC_RNGC_U01_65F (1.0f / 36893488147419103232.0f)
#if VSMC_HAS_OPENCL_DOUBLE
#define VSMC_RNGC_U01_32D (1.0 / 4294967296.0)
#define VSMC_RNGC_U01_33D (1.0 / 8589934592.0)
#define VSMC_RNGC_U01_52D (1.0 / 4503599627370496.0)
#define VSMC_RNGC_U01_53D (1.0 / 9007199254740992.0)
#define VSMC_RNGC_U01_63D (1.0 / 9223372036854775808.0)
#define VSMC_RNGC_U01_64D (1.0 / 18446744073709551616.0)
#define VSMC_RNGC_U01_65D (1.0 / 36893488147419103232.0)
#endif // VSMC_HAS_OPENCL_DOUBLE
#else  // VSMC_OPENCL
static const float VSMC_RNGC_U01_23F = 1.0f / 8388608.0f;
static const float VSMC_RNGC_U01_24F = 1.0f / 16777216.0f;
static const float VSMC_RNGC_U01_31F = 1.0f / 2147483648.0f;
static const float VSMC_RNGC_U01_32F = 1.0f / 4294967296.0f;
static const float VSMC_RNGC_U01_33F = 1.0f / 8589934592.0f;
static const float VSMC_RNGC_U01_64F = 1.0f / 18446744073709551616.0f;
static const float VSMC_RNGC_U01_65F = 1.0f / 36893488147419103232.0f;
static const double VSMC_RNGC_U01_32D = 1.0 / 4294967296.0;
static const double VSMC_RNGC_U01_33D = 1.0 / 8589934592.0;
static const double VSMC_RNGC_U01_52D = 1.0 / 4503599627370496.0;
static const double VSMC_RNGC_U01_53D = 1.0 / 9007199254740992.0;
static const double VSMC_RNGC_U01_63D = 1.0 / 9223372036854775808.0;
static const double VSMC_RNGC_U01_64D = 1.0 / 18446744073709551616.0;
static const double VSMC_RNGC_U01_65D = 1.0 / 36893488147419103232.0;
static const long double VSMC_RNGC_U01_32L = 1.0l / 4294967296.0l;
static const long double VSMC_RNGC_U01_33L = 1.0l / 8589934592.0l;
static const long double VSMC_RNGC_U01_63L = 1.0l / 9223372036854775808.0l;
static const long double VSMC_RNGC_U01_64L = 1.0l / 18446744073709551616.0l;
static const long double VSMC_RNGC_U01_65L = 1.0l / 36893488147419103232.0l;
#endif // VSMC_OPENCL

/// \brief Converting 32-bit unsigned to single precision uniform \f$(0, 1)\f$
/// \ingroup U01C
static inline float vsmc_u01_u32f(uint32_t u)
{
    return u * VSMC_RNGC_U01_32F + VSMC_RNGC_U01_33F;
}

/// \brief Converting 64-bit unsigned to single precision uniform \f$(0, 1)\f$
/// \ingroup U01C
static inline float vsmc_u01_u64f(uint64_t u)
{
    return u * VSMC_RNGC_U01_64F + VSMC_RNGC_U01_65F;
}

#if !defined(VSMC_OPENCL) || VSMC_HAS_OPENCL_DOUBLE

/// \brief Converting 32-bit unsigned to double precision uniform \f$(0, 1)\f$
/// \ingroup U01C
static inline double vsmc_u01_u32d(uint32_t u)
{
    return u * VSMC_RNGC_U01_32D + VSMC_RNGC_U01_33D;
}

/// \brief Converting 64-bit unsigned to double precision uniform \f$(0, 1)\f$
/// \ingroup U01C
static inline double vsmc_u01_u64d(uint64_t u)
{
    return u * VSMC_RNGC_U01_64D + VSMC_RNGC_U01_65D;
}

#endif // !defined(VSMC_OPENCL) || VSMC_HAS_OPENCL_DOUBLE

#ifndef VSMC_OPENCL

/// \brief Converting 32-bit unsigned to long double precision uniform
/// \f$(0, 1)\f$
static inline long double vsmc_u01_u32l(uint32_t u)
{
    return u * VSMC_RNGC_U01_32L + VSMC_RNGC_U01_33L;
}

/// \brief Converting 64-bit unsigned to long double precision uniform
/// \f$(0, 1)\f$
static inline long double vsmc_u01_u64l(uint64_t u)
{
    return u * VSMC_RNGC_U01_64L + VSMC_RNGC_U01_65L;
}

#endif // VSMC_OPENCL

/// \brief Converting 32-bit unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
static inline float vsmc_u01_cc_u32f(uint32_t u)
{
    return ((u & UINT32_C(0x40)) + (u & UINT32_C(0x7FFFFFC0))) *
        VSMC_RNGC_U01_31F;
}

/// \brief Converting 32-bit unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
static inline float vsmc_u01_co_u32f(uint32_t u)
{
    return (u >> 8) * VSMC_RNGC_U01_24F;
}

/// \brief Converting 32-bit unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
static inline float vsmc_u01_oc_u32f(uint32_t u)
{
    return (u >> 8) * VSMC_RNGC_U01_24F + VSMC_RNGC_U01_24F;
}

/// \brief Converting 32-bit unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
static inline float vsmc_u01_oo_u32f(uint32_t u)
{
    return (u >> 9) * VSMC_RNGC_U01_23F + VSMC_RNGC_U01_24F;
}

/// \brief Converting 64-bit unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
static inline float vsmc_u01_cc_u64f(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_cc_u32f(static_cast<uint32_t>(u >> 32));
#else
    return vsmc_u01_cc_u32f(((uint32_t)(u >> 32)));
#endif
}

/// \brief Converting 64-bit unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
static inline float vsmc_u01_co_u64f(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_co_u32f(static_cast<uint32_t>(u >> 32));
#else
    return vsmc_u01_co_u32f(((uint32_t)(u >> 32)));
#endif
}

/// \brief Converting 64-bit unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
static inline float vsmc_u01_oc_u64f(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_oc_u32f(static_cast<uint32_t>(u >> 32));
#else
    return vsmc_u01_oc_u32f(((uint32_t)(u >> 32)));
#endif
}

/// \brief Converting 64-bit unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
static inline float vsmc_u01_oo_u64f(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_oo_u32f(static_cast<uint32_t>(u >> 32));
#else
    return vsmc_u01_oo_u32f(((uint32_t)(u >> 32)));
#endif
}

#if !defined(VSMC_OPENCL) || VSMC_HAS_OPENCL_DOUBLE

/// \brief Converting 32-bit unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
static inline double vsmc_u01_cc_u32d(uint32_t u)
{
#ifdef __cplusplus
    return (static_cast<double>(u & 1) + u) * VSMC_RNGC_U01_32D;
#else
    return (((double) (u & 1)) + u) * VSMC_RNGC_U01_32D;
#endif
}

/// \brief Converting 32-bit unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
static inline double vsmc_u01_co_u32d(uint32_t u)
{
    return u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 32-bit unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
static inline double vsmc_u01_oc_u32d(uint32_t u)
{
    return VSMC_RNGC_U01_32D + u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 32-bit unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
static inline double vsmc_u01_oo_u32d(uint32_t u)
{
    return VSMC_RNGC_U01_33D + u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 64-bit unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
static inline double vsmc_u01_cc_u64d(uint64_t u)
{
    return ((u & UINT64_C(0x7FFFFFFFFFFFFE00)) + (u & UINT64_C(0x200))) *
        VSMC_RNGC_U01_63D;
}

/// \brief Converting 64-bit unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
static inline double vsmc_u01_co_u64d(uint64_t u)
{
    return (u >> 11) * VSMC_RNGC_U01_53D;
}

/// \brief Converting 64-bit unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
static inline double vsmc_u01_oc_u64d(uint64_t u)
{
    return VSMC_RNGC_U01_53D + (u >> 11) * VSMC_RNGC_U01_53D;
}

/// \brief Converting 64-bit unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
static inline double vsmc_u01_oo_u64d(uint64_t u)
{
    return VSMC_RNGC_U01_53D + (u >> 12) * VSMC_RNGC_U01_52D;
}

#endif // !defined(VSMC_OPENCL) || VSMC_HAS_OPENCL_DOUBLE

#ifndef VSMC_OPENCL

/// \brief Converting 32-bit unsigned to long double precision uniform
/// \f$[0,1]\f$
/// \ingroup U01C
static inline long double vsmc_u01_cc_u32l(uint32_t u)
{
#ifdef __cplusplus
    return (static_cast<long double>(u & 1) + u) * VSMC_RNGC_U01_32L;
#else
    return (((long double) (u & 1)) + u) * VSMC_RNGC_U01_32L;
#endif
}

/// \brief Converting 32-bit unsigned to long double precision uniform
/// \f$[0,1)\f$
/// \ingroup U01C
static inline long double vsmc_u01_co_u32l(uint32_t u)
{
    return u * VSMC_RNGC_U01_32L;
}

/// \brief Converting 32-bit unsigned to long double precision uniform
/// \f$(0,1]\f$
/// \ingroup U01C
static inline long double vsmc_u01_oc_u32l(uint32_t u)
{
    return u * VSMC_RNGC_U01_32L + VSMC_RNGC_U01_32L;
}

/// \brief Converting 32-bit unsigned to long double precision uniform
/// \f$(0,1)\f$
/// \ingroup U01C
static inline long double vsmc_u01_oo_u32l(uint32_t u)
{
    return u * VSMC_RNGC_U01_32L + VSMC_RNGC_U01_33L;
}

/// \brief Converting 64-bit unsigned to long double precision uniform
/// \f$[0,1]\f$
/// \ingroup U01C
static inline long double vsmc_u01_cc_u64l(uint64_t u)
{
#ifdef __cplusplus
    return (static_cast<long double>(u & 1) + u) * VSMC_RNGC_U01_64L;
#else
    return (((long double) (u & 1)) + u) * VSMC_RNGC_U01_64L;
#endif
}

/// \brief Converting 64-bit unsigned to long double precision uniform
/// \f$[0,1)\f$
/// \ingroup U01C
static inline long double vsmc_u01_co_u64l(uint64_t u)
{
    return u * VSMC_RNGC_U01_64L;
}

/// \brief Converting 64-bit unsigned to long double precision uniform
/// \f$(0,1]\f$
/// \ingroup U01C
static inline long double vsmc_u01_oc_u64l(uint64_t u)
{
    return u * VSMC_RNGC_U01_64L + VSMC_RNGC_U01_64L;
}

/// \brief Converting 64-bit unsigned to long double precision uniform
/// \f$(0,1)\f$
/// \ingroup U01C
static inline long double vsmc_u01_oo_u64l(uint64_t u)
{
    return (u >> 1) * VSMC_RNGC_U01_63L + VSMC_RNGC_U01_64L;
}

#endif // VSMC_OPENCL

#endif // VSMC_RNGC_U01_H
