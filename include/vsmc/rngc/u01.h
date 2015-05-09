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

#include <vsmc/rngc/internal/common.h>

#if VSMC_HAS_RNGC_DOUBLE

#define vsmc_u01_open_open_32 vsmc_u01_open_open_32_53
#define vsmc_u01_open_closed_32 vsmc_u01_open_closed_32_53
#define vsmc_u01_closed_open_32 vsmc_u01_closed_closed_32_53
#define vsmc_u01_closed_closed_32 vsmc_u01_closed_closed_32_53

#define vsmc_u01_open_open_64 vsmc_u01_open_open_64_53
#define vsmc_u01_open_closed_64 vsmc_u01_open_closed_64_53
#define vsmc_u01_closed_open_64 vsmc_u01_closed_closed_64_53
#define vsmc_u01_closed_closed_64 vsmc_u01_closed_closed_64_53

#else // VSMC_HAS_RNGC_DOUBLE

#define vsmc_u01_open_open_32 vsmc_u01_open_open_32_24
#define vsmc_u01_open_closed_32 vsmc_u01_open_closed_32_24
#define vsmc_u01_closed_open_32 vsmc_u01_closed_closed_32_24
#define vsmc_u01_closed_closed_32 vsmc_u01_closed_closed_32_24

#endif // VSMC_HAS_RNGC_DOUBLE

#define VSMC_RNGC_U01_31f (1.0f / (1024.0f * 1024.0f * 1024.0f * 2.0f))
#define VSMC_RNGC_U01_24f (128.0f * VSMC_RNGC_U01_31f)
#define VSMC_RNGC_U01_23f (256.0f * VSMC_RNGC_U01_31f)
#define VSMC_RNGC_U01_32 (1.0 / (1024.0 * 1024.0 * 1024.0 * 4.0))
#define VSMC_RNGC_U01_63 (2.0 * VSMC_RNGC_U01_32 * VSMC_RNGC_U01_32)
#define VSMC_RNGC_U01_53 (1024.0 * VSMC_RNGC_U01_63)
#define VSMC_RNGC_U01_52 (2048.0 * VSMC_RNGC_U01_63)

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_closed_32_24(uint32_t u)
{
    return ((u & 0x7fffffc0) + (u & 0x40)) * VSMC_RNGC_U01_31f;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_open_32_24(uint32_t u)
{
    return (u >> 8) * VSMC_RNGC_U01_24f;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_closed_32_24(uint32_t u)
{
    return (1.0f + (u >> 8)) * VSMC_RNGC_U01_24f;
}

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_open_32_24(uint32_t u)
{
    return (0.5f + (u >> 9)) * VSMC_RNGC_U01_23f;
}

#if VSMC_HAS_RNGC_DOUBLE

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_closed_32_53(uint32_t u)
{
#ifdef __cplusplus
    return (static_cast<double>(u & 1) + u) * VSMC_RNGC_U01_32;
#else
    return (((double) (u & 1)) + u) * VSMC_RNGC_U01_32;
#endif
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_open_32_53(uint32_t u)
{
    return u * VSMC_RNGC_U01_32;
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_closed_32_53(uint32_t u)
{
    return (1.0 + u) * VSMC_RNGC_U01_32;
}

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_open_32_53(uint32_t u)
{
    return (0.5 + u) * VSMC_RNGC_U01_32;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_closed_64_53(uint64_t u)
{
    return ((u & UINT64_C(0x7ffffffffffffe00)) + (u & 0x200)) *
        VSMC_RNGC_U01_63;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_closed_open_64_53(uint64_t u)
{
    return (u >> 11) * VSMC_RNGC_U01_53;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_closed_64_53(uint64_t u)
{
    return (1.0 + (u >> 11)) * VSMC_RNGC_U01_53;
}

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE double vsmc_u01_open_open_64_53(uint64_t u)
{
    return (0.5 + (u >> 12)) * VSMC_RNGC_U01_52;
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_closed_64_24(uint64_t u)
{
#ifdef __cplusplus
    return static_cast<float>(vsmc_u01_closed_closed_64_53(u));
#else
    return (float) vsmc_u01_closed_closed_64_53(u);
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_closed_open_64_24(uint64_t u)
{
#ifdef __cplusplus
    return static_cast<float>(vsmc_u01_closed_open_64_53(u));
#else
    return (float) vsmc_u01_closed_open_64_53(u);
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_closed_64_24(uint64_t u)
{
#ifdef __cplusplus
    return static_cast<float>(vsmc_u01_open_closed_64_53(u));
#else
    return (float) vsmc_u01_open_closed_64_53(u);
#endif
}

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01C
VSMC_STATIC_INLINE float vsmc_u01_open_open_64_24(uint64_t u)
{
#ifdef __cplusplus
    return static_cast<float>(vsmc_u01_open_open_64_53(u));
#else
    return (float) vsmc_u01_open_open_64_53(u);
#endif
}

#endif // VSMC_HAS_RNGC_DOUBLE

#endif // VSMC_RNGC_U01_H
