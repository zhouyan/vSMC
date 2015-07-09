//============================================================================
// vSMC/include/vsmc/rngc/u01.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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
#define VSMC_RNGC_U01_31F (1.0f / 2147483648.0f)
#define VSMC_RNGC_U01_24F (1.0f / 16777216.0f)
#define VSMC_RNGC_U01_23F (1.0f / 8388608.0f)
#define VSMC_RNGC_U01_33D (1.0 / 8589934592.0)
#define VSMC_RNGC_U01_32D (1.0 / 4294967296.0)
#define VSMC_RNGC_U01_63D (1.0 / 9223372036854775808.0)
#define VSMC_RNGC_U01_53D (1.0 / 9007199254740992.0)
#define VSMC_RNGC_U01_52D (1.0 / 4503599627370496.0)
#else  // VSMC_OPENCL
static const float VSMC_RNGC_U01_31F = 1.0f / 2147483648.0f;
static const float VSMC_RNGC_U01_24F = 1.0f / 16777216.0f;
static const float VSMC_RNGC_U01_23F = 1.0f / 8388608.0f;
static const double VSMC_RNGC_U01_33D = 1.0 / 8589934592.0;
static const double VSMC_RNGC_U01_32D = 1.0 / 4294967296.0;
static const double VSMC_RNGC_U01_63D = 1.0 / 9223372036854775808.0;
static const double VSMC_RNGC_U01_53D = 1.0 / 9007199254740992.0;
static const double VSMC_RNGC_U01_52D = 1.0 / 4503599627370496.0;
#endif // VSMC_OPENCL

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_closed_u32_f32(uint32_t u)
{
    return ((u & UINT32_C(0x7FFFFFC0)) + (u & UINT32_C(0x40))) *
        VSMC_RNGC_U01_31F;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_open_u32_f32(uint32_t u)
{
    return (u >> 8) * VSMC_RNGC_U01_24F;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_closed_u32_f32(uint32_t u)
{
    return VSMC_RNGC_U01_24F + (u >> 8) * VSMC_RNGC_U01_24F;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_open_u32_f32(uint32_t u)
{
    return VSMC_RNGC_U01_24F + (u >> 9) * VSMC_RNGC_U01_23F;
}

#if VSMC_HAS_RNGC_DOUBLE

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_closed_u32_f64(uint32_t u)
{
#ifdef __cplusplus
    return (static_cast<double>(u & 1) + u) * VSMC_RNGC_U01_32D;
#else
    return (((double) (u & 1)) + u) * VSMC_RNGC_U01_32D;
#endif
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_open_u32_f64(uint32_t u)
{
    return u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_closed_u32_f64(uint32_t u)
{
    return VSMC_RNGC_U01_32D + u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_open_u32_f64(uint32_t u)
{
    return VSMC_RNGC_U01_33D + u * VSMC_RNGC_U01_32D;
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_closed_u64_f32(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_closed_closed_u32_f32(static_cast<uint32_t>(u));
#else
    return vsmc_u01_closed_closed_u32_f32(((uint32_t) u));
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_open_u64_f32(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_closed_open_u32_f32(static_cast<uint32_t>(u));
#else
    return vsmc_u01_closed_open_u32_f32(((uint32_t) u));
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_closed_u64_f32(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_open_closed_u32_f32(static_cast<uint32_t>(u));
#else
    return vsmc_u01_open_closed_u32_f32(((uint32_t) u));
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_open_u64_f32(uint64_t u)
{
#ifdef __cplusplus
    return vsmc_u01_open_open_u32_f32(static_cast<uint32_t>(u));
#else
    return vsmc_u01_open_open_u32_f32(((uint32_t) u));
#endif
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_closed_u64_f64(uint64_t u)
{
    return ((u & UINT64_C(0x7FFFFFFFFFFFFE00)) + (u & UINT64_C(0x200))) *
        VSMC_RNGC_U01_63D;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_open_u64_f64(uint64_t u)
{
    return (u >> 11) * VSMC_RNGC_U01_53D;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_closed_u64_f64(uint64_t u)
{
    return VSMC_RNGC_U01_53D + (u >> 11) * VSMC_RNGC_U01_53D;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_open_u64_f64(uint64_t u)
{
    return VSMC_RNGC_U01_53D + (u >> 12) * VSMC_RNGC_U01_52D;
}

#endif // VSMC_HAS_RNGC_DOUBLE

#endif // VSMC_RNGC_U01_H
