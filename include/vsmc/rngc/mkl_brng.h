//============================================================================
// vSMC/include/vsmc/rngc/mkl_brng.h
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

#ifndef VSMC_RNGC_MKL_BRNG_H
#define VSMC_RNGC_MKL_BRNG_H

#ifdef VSMC_DEFINE_RNG_C_API_ENGINE
#undef VSMC_DEFINE_RNG_C_API_ENGINE
#endif

#ifdef VSMC_DEFINE_RNG_C_API_ENGINE_ALL
#undef VSMC_DEFINE_RNG_C_API_ENGINE_ALL
#endif

#define VSMC_DEFINE_RNG_C_API_ENGINE(RNGType, name) \
    int vsmc_mkl_brng_##name(void);

#define VSMC_DEFINE_RNG_C_API_ENGINE_ALL 1

#ifdef __cplusplus
extern "C" {
#endif

#include <vsmc/rng/internal/c_api_engine.hpp>

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_RNGC_MKL_BRNG_H
