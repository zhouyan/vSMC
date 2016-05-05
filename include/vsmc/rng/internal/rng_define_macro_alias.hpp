//============================================================================
// vSMC/include/rng/internal/rng_define_macro_alias.hpp
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

#ifndef VSMC_RNG_DEFINE_MACRO_NA
#define VSMC_RNG_DEFINE_MACRO_NA(RNGType, Name, name)
#endif

VSMC_RNG_DEFINE_MACRO(::vsmc::RNG, RNG, rng)
VSMC_RNG_DEFINE_MACRO(::vsmc::RNG_64, RNG_64, rng_64)

VSMC_RNG_DEFINE_MACRO(::vsmc::RNGMini, RNGMini, rngmini)
VSMC_RNG_DEFINE_MACRO(::vsmc::RNGMini_64, RNGMini_64, rngmini_64)

VSMC_RNG_DEFINE_MACRO(::vsmc::Philox, Philox, philox)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox_64, Philox_64, philox_64)

VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry, Threefry, threefry)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry_64, Threefry_64, threefry_64)

#if VSMC_HAS_AES_NI
#define VSMC_RNG_DEFINE_MACRO_AES_NI VSMC_RNG_DEFINE_MACRO
#else
#define VSMC_RNG_DEFINE_MACRO_AES_NI VSMC_RNG_DEFINE_MACRO_NA
#endif

VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES128, AES128, aes128)
VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES128_64, AES128_64, aes128_64)

VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES192, AES192, aes192)
VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES192_64, AES192_64, aes192_64)

VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES256, AES256, aes256)
VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::AES256_64, AES256_64, aes256_64)

VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::ARS, ARS, ars)
VSMC_RNG_DEFINE_MACRO_AES_NI(::vsmc::ARS_64, ARS_64, ars_64)

#undef VSMC_RNG_DEFINE_MACRO_AES_NI
