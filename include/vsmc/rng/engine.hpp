//============================================================================
// vSMC/include/vsmc/rng/engine.hpp
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

#ifndef VSMC_RNG_ENGINE_HPP
#define VSMC_RNG_ENGINE_HPP

#include <vsmc/internal/config.h>

/// \brief Default `RNG` type
/// \ingroup Config
#ifndef VSMC_RNG_TYPE
#if VSMC_HAS_AES_NI
#define VSMC_RNG_TYPE ::vsmc::ARS
#else
#define VSMC_RNG_TYPE ::vsmc::Threefry
#endif
#endif

/// \brief Default `RNG_64` type
#ifndef VSMC_RNG_64_TYPE
#if VSMC_HAS_AES_NI
#define VSMC_RNG_64_TYPE ARS_64
#else
#define VSMC_RNG_64_TYPE Threefry_64
#endif
#endif

#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>

#if VSMC_HAS_AES_NI
#include <vsmc/rng/aes_ni.hpp>
#endif

#if VSMC_HAS_MKL
#include <vsmc/rng/mkl.hpp>
#endif

#if VSMC_HAS_RDRAND
#include <vsmc/rng/rdrand.hpp>
#endif

namespace vsmc
{

/// \brief The default 32-bits RNG
/// \ingroup RNG
using RNG = VSMC_RNG_TYPE;

/// \brief The default 64-bits RNG
/// \ingroup RNG
using RNG_64 = VSMC_RNG_64_TYPE;

/// \brief The 32-bits RNG with smallest state
/// \ingroup RNG
using RNGMini = Philox2x32;

/// \brief The 64-bits RNG with smallest state
/// \ingroup RNG
using RNGMini_64 = Philox2x32_64;

} // namespace vsmc

#endif // VSMC_RNG_ENGINE_HPP
