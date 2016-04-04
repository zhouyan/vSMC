//============================================================================
// vSMC/rng/internal/rng_define_macro.hpp
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

VSMC_RNG_DEFINE_MACRO(std::mt19937, mt19937)
VSMC_RNG_DEFINE_MACRO(std::mt19937_64, mt19937_64)
VSMC_RNG_DEFINE_MACRO(std::minstd_rand0, minstd_rand0)
VSMC_RNG_DEFINE_MACRO(std::minstd_rand, minstd_rand)
VSMC_RNG_DEFINE_MACRO(std::ranlux24_base, ranlux24_base)
VSMC_RNG_DEFINE_MACRO(std::ranlux48_base, ranlux48_base)
VSMC_RNG_DEFINE_MACRO(std::ranlux24, ranlux24)
VSMC_RNG_DEFINE_MACRO(std::ranlux48, ranlux48)
VSMC_RNG_DEFINE_MACRO(std::knuth_b, knuth_b)

VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x32, philox2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x32, philox4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x64, philox2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x64, philox4x64)

VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x32, threefry2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x32, threefry4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x64, threefry2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x64, threefry4x64)
#if VSMC_HAS_SSE2
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x32SSE2, threefry2x32sse2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x32SSE2, threefry4x32sse2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x64SSE2, threefry2x64sse2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x64SSE2, threefry4x64sse2)
#endif // VSMC_HAS_SSE2
#if VSMC_HAS_AVX2
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x32AVX2, threefry2x32avx2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x32AVX2, threefry4x32avx2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x64AVX2, threefry2x64avx2)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x64AVX2, threefry4x64avx2)
#endif // VSMC_HAS_AVX2

#if VSMC_HAS_AES_NI
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_1x32, aes128_1x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_2x32, aes128_2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_4x32, aes128_4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_8x32, aes128_8x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_1x64, aes128_1x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_2x64, aes128_2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_4x64, aes128_4x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128_8x64, aes128_8x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_1x32, aes192_1x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_2x32, aes192_2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_4x32, aes192_4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_8x32, aes192_8x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_1x64, aes192_1x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_2x64, aes192_2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_4x64, aes192_4x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192_8x64, aes192_8x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_1x32, aes256_1x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_2x32, aes256_2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_4x32, aes256_4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_8x32, aes256_8x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_1x64, aes256_1x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_2x64, aes256_2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_4x64, aes256_4x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256_8x64, aes256_8x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_1x32, ars_1x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_2x32, ars_2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_4x32, ars_4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_8x32, ars_8x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_1x64, ars_1x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_2x64, ars_2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_4x64, ars_4x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARS_8x64, ars_8x64)
#endif // VSMC_HAS_AES_NI

#if VSMC_HAS_RDRAND
VSMC_RNG_DEFINE_MACRO(::vsmc::RDRAND32, rdrand32)
VSMC_RNG_DEFINE_MACRO(::vsmc::RDRAND64, rdrand64)
#endif // VSMC_HAS_RDRAND
