//============================================================================
// vSMC/rng/internal/c_api_engine.hpp
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

#ifdef VSMC_DEFINE_RNG_C_API_ENGINE

#ifndef VSMC_DEFINE_RNG_C_API_ENGINE_ALL
#define VSMC_DEFINE_RNG_C_API_ENGINE_ALL 0
#endif

VSMC_DEFINE_RNG_C_API_ENGINE(std::mt19937, mt19937)
VSMC_DEFINE_RNG_C_API_ENGINE(std::minstd_rand0, minstd_rand0)
VSMC_DEFINE_RNG_C_API_ENGINE(std::minstd_rand, minstd_rand)
VSMC_DEFINE_RNG_C_API_ENGINE(std::mt19937_64, mt19937_64)
VSMC_DEFINE_RNG_C_API_ENGINE(std::ranlux24_base, ranlux24_base)
VSMC_DEFINE_RNG_C_API_ENGINE(std::ranlux48_base, ranlux48_base)
VSMC_DEFINE_RNG_C_API_ENGINE(std::ranlux24, ranlux24)
VSMC_DEFINE_RNG_C_API_ENGINE(std::ranlux48, ranlux48)
VSMC_DEFINE_RNG_C_API_ENGINE(std::knuth_b, knuth_b)

VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift1x32, xorshift1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift2x32, xorshift2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift4x32, xorshift4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift8x32, xorshift8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift16x32, xorshift16x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift32x32, xorshift32x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift64x32, xorshift64x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift128x32, xorshift128x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift1x64, xorshift1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift2x64, xorshift2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift4x64, xorshift4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift8x64, xorshift8x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift16x64, xorshift16x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift32x64, xorshift32x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorshift64x64, xorshift64x64)

VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow1x32, xorwow1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow2x32, xorwow2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow4x32, xorwow4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow8x32, xorwow8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow16x32, xorwow16x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow32x32, xorwow32x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow64x32, xorwow64x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow128x32, xorwow128x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow1x64, xorwow1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow2x64, xorwow2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow4x64, xorwow4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow8x64, xorwow8x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow16x64, xorwow16x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow32x64, xorwow32x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Xorwow64x64, xorwow64x64)

VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Philox2x32, philox2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Philox4x32, philox4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Philox2x64, philox2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Philox4x64, philox4x64)

VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x32, threefry2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x32, threefry4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x64, threefry2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x64, threefry4x64)
#if VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_AVX2
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x32AVX2, threefry2x32avx2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x32AVX2, threefry4x32avx2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x64AVX2, threefry2x64avx2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x64AVX2, threefry4x64avx2)
#endif // VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_AVX2
#if VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_SSE2
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x32SSE2, threefry2x32sse2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x32SSE2, threefry4x32sse2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry2x64SSE2, threefry2x64sse2)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::Threefry4x64SSE2, threefry4x64sse2)
#endif // VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_SSE2

#if VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_AES_NI
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_1x32, aes128_1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_2x32, aes128_2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_4x32, aes128_4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_8x32, aes128_8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_1x64, aes128_1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_2x64, aes128_2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_4x64, aes128_4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES128_8x64, aes128_8x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_1x32, aes192_1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_2x32, aes192_2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_4x32, aes192_4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_8x32, aes192_8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_1x64, aes192_1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_2x64, aes192_2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_4x64, aes192_4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES192_8x64, aes192_8x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_1x32, aes256_1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_2x32, aes256_2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_4x32, aes256_4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_8x32, aes256_8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_1x64, aes256_1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_2x64, aes256_2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_4x64, aes256_4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::AES256_8x64, aes256_8x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_1x32, ars_1x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_2x32, ars_2x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_4x32, ars_4x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_8x32, ars_8x32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_1x64, ars_1x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_2x64, ars_2x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_4x64, ars_4x64)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::ARS_8x64, ars_8x64)
#endif // VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_AES_NI

#if VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_RDRAND
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::RDRAND16, rdrand16)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::RDRAND32, rdrand32)
VSMC_DEFINE_RNG_C_API_ENGINE(::vsmc::RDRAND64, rdrand64)
#endif // VSMC_DEFINE_RNG_C_API_ENGINE_ALL || VSMC_HAS_RDRAND

#undef VSMC_DEFINE_RNG_C_API_ENGINE
#undef VSMC_DEFINE_RNG_C_API_ENGINE_ALL

#endif // VSMC_DEFINE_RNG_C_API_ENGINE
