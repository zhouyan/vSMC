//============================================================================
// vSMC/include/vsmc/rngc/internal/common.h
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

#ifndef VSMC_RNGC_INTERNAL_COMMON_H
#define VSMC_RNGC_INTERNAL_COMMON_H

#include <vsmc/internal/config.h>

#define VSMC_F32_C(x) (x##f)
#define VSMC_F64_C(x) (x)

#define VSMC_LOG_F64 log
#define VSMC_EXP_F64 exp
#define VSMC_EXPM1_F64 expm1
#define VSMC_SQRT_F64 sqrt
#define VSMC_SIN_F64 sin
#define VSMC_COS_F64 cos

#ifdef __OPENCL_VERSION__
#define VSMC_LOG_F32 log
#define VSMC_EXP_F32 exp
#define VSMC_EXPM1_F32 expm1
#define VSMC_SQRT_F32 sqrt
#define VSMC_SIN_F32 sin
#define VSMC_COS_F32 cos
#else // __OPENCL_VERSION__
#define VSMC_LOG_F32 logf
#define VSMC_EXP_F32 expf
#define VSMC_EXPM1_F32 expm1f
#define VSMC_SQRT_F32 sqrtf
#define VSMC_SIN_F32 sinf
#define VSMC_COS_F32 cosf
#endif // __OPENCL_VERSION__

/// \brief Default RNG state
/// \ingroup RNGC
#ifndef vsmc_crng
#define vsmc_crng vsmc_philox2x32
#endif

/// \brief Initialize default RNG state
/// \ingroup RNGC
#ifndef vsmc_crng_init
#define vsmc_crng_init vsmc_philox2x32_init
#endif

/// \brief Generate random 32-bits integers from default RNG
/// \ingroup RNGC
#ifndef vsmc_crng_rand
#define vsmc_crng_rand vsmc_philox2x32_rand
#endif

#endif // VSMC_RNGC_INTERNAL_COMMON_H
