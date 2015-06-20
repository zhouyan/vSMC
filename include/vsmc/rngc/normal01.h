//============================================================================
// vSMC/include/vsmc/rngc/normal01.h
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

#ifndef VSMC_RNGC_NORMAL01_H
#define VSMC_RNGC_NORMAL01_H

#include <vsmc/rngc/internal/common.h>
#include <vsmc/rngc/philox.h>
#include <vsmc/rngc/u01.h>

#if VSMC_HAS_RNGC_DOUBLE
#define vsmc_normal01 vsmc_normal01_f64
#define vsmc_normal01_init vsmc_normal01_f64_init
#define vsmc_normal01_rand vsmc_normal01_f64_rand
#else // VSMC_HAS_RNGC_DOUBLE
#define vsmc_normal01 vsmc_normal01_f32
#define vsmc_normal01_init vsmc_normal01_f32_init
#define vsmc_normal01_rand vsmc_normal01_f32_rand
#endif // VSMC_HAS_RNGC_DOUBLE

#define VSMC_DEFINE_RNGC_NORMAL01(F, FT)                                      \
    typedef struct {                                                          \
        FT u1;                                                                \
        FT u2;                                                                \
        unsigned char saved;                                                  \
    } vsmc_normal01_f##F;

#define VSMC_DEFINE_RNGC_NORMAL01_INIT(F, FT)                                 \
    VSMC_STATIC_INLINE void vsmc_normal01_f##F##_init(                        \
        vsmc_normal01_f##F *rnorm, vsmc_crng *rng)                            \
    {                                                                         \
        rnorm->u1 = vsmc_u01_open_closed_u32_f##F(vsmc_crng_rand(rng));       \
        rnorm->u2 = vsmc_u01_open_closed_u32_f##F(vsmc_crng_rand(rng));       \
        rnorm->saved = 1;                                                     \
    }

#define VSMC_DEFINE_RNGC_NORMAL01_RAND(F, FT)                                 \
    VSMC_STATIC_INLINE FT vsmc_normal01_f##F##_rand(                          \
        vsmc_normal01_f##F *rnorm, vsmc_crng *rng)                            \
    {                                                                         \
        const FT c_2pi = VSMC_F##F##_C(6.2831853071795865);                   \
        if (rnorm->saved) {                                                   \
            rnorm->saved = 0;                                                 \
            return VSMC_SQRT_F##F(-2 * VSMC_LOG_F##F(rnorm->u1)) *            \
                VSMC_COS_F##F(c_2pi * rnorm->u2);                             \
        } else {                                                              \
            vsmc_normal01_f##F##_init(rnorm, rng);                            \
            return VSMC_SQRT_F##F(-2 * VSMC_LOG_F##F(rnorm->u1)) *            \
                VSMC_SIN_F##F(c_2pi * rnorm->u2);                             \
        }                                                                     \
    }

/// \brief Normal(0, 1) structure (single precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01(32, float)

/// \brief Initialize Normal(0, 1) structure (single precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01_INIT(32, float)

/// \brief Generate Normal(0, 1) random numbers (single precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01_RAND(32, float)

#if VSMC_HAS_RNGC_DOUBLE

/// \brief Normal(0, 1) structure (double precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01(64, double)

/// \brief Initialize Normal(0, 1) structure (double precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01_INIT(64, double)

/// \brief Generate Normal(0, 1) random numbers (double precision)
/// \ingroup Normal01C
VSMC_DEFINE_RNGC_NORMAL01_RAND(64, double)

#endif // VSMC_HAS_RNGC_DOUBLE

#endif // VSMC_RNGC_NORMAL01_H
