//============================================================================
// vSMC/include/vsmc/rng/normal01.h
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

#ifndef VSMC_RNG_NORMAL01_H
#define VSMC_RNG_NORMAL01_H

#include <vsmc/rng/internal/common.h>
#include <vsmc/rng/philox.h>
#include <vsmc/rng/u01.h>

#if VSMC_HAS_OPENCL_DOUBLE
#define vsmc_normal01 vsmc_normal01_53
#define vsmc_normal01_init vsmc_normal01_53_init
#define vsmc_normal01_rand vsmc_normal01_53_rand
#else // VSMC_HAS_OPENCL_DOUBLE
#define vsmc_normal01 vsmc_normal01_24
#define vsmc_normal01_init vsmc_normal01_24_init
#define vsmc_normal01_rand vsmc_normal01_24_rand
#endif // VSMC_HAS_OPENCL_DOUBLE

#define VSMC_DEFINE_RNG_NORMAL01(F, FT)                                       \
    typedef struct {                                                          \
        FT u1;                                                                \
        FT u2;                                                                \
        unsigned char saved;                                                  \
    } vsmc_normal01_##F;

#define VSMC_DEFINE_RNG_NORMAL01_INIT(F, FT)                                  \
    VSMC_STATIC_INLINE void vsmc_normal01_##F##_init(                         \
        vsmc_normal01_##F *rnorm, vsmc_rng *rng)                              \
    {                                                                         \
        rnorm->u1 = vsmc_u01_open_closed_32_##F(vsmc_rng_rand(rng));          \
        rnorm->u2 = vsmc_u01_open_closed_32_##F(vsmc_rng_rand(rng));          \
        rnorm->saved = 1;                                                     \
    }

#define VSMC_DEFINE_RNG_NORMAL01_RAND(F, FT)                                  \
    VSMC_STATIC_INLINE FT vsmc_normal01_##F##_rand(                           \
        vsmc_normal01_##F *rnorm, vsmc_rng *rng)                              \
    {                                                                         \
        const FT c_2pi = VSMC_FP##F##_C(6.2831853071795865);                  \
        if (rnorm->saved) {                                                   \
            rnorm->saved = 0;                                                 \
            return VSMC_SQRT##F(-2 * VSMC_LOG##F(rnorm->u1)) *                \
                VSMC_COS##F(c_2pi * rnorm->u2);                               \
        } else {                                                              \
            vsmc_normal01_##F##_init(rnorm, rng);                             \
            return VSMC_SQRT##F(-2 * VSMC_LOG##F(rnorm->u1)) *                \
                VSMC_SIN##F(c_2pi * rnorm->u2);                               \
        }                                                                     \
    }

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01(24, float)

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01_INIT(24, float)

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01_RAND(24, float)

#if VSMC_HAS_OPENCL_DOUBLE

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01(53, double)

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01_INIT(53, double)

/// \ingroup CRNG
VSMC_DEFINE_RNG_NORMAL01_RAND(53, double)

#endif // VSMC_HAS_OPENCL_DOUBLE

#endif // VSMC_RNG_NORMAL01_H
