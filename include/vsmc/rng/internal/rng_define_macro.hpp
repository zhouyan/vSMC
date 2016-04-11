//============================================================================
// vSMC/include/rng/internal/rng_define_macro.hpp
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

VSMC_RNG_DEFINE_MACRO(::std::mt19937, mt19937, mt19937)
VSMC_RNG_DEFINE_MACRO(::std::mt19937_64, mt19937_64, mt19937_64)
VSMC_RNG_DEFINE_MACRO(::std::minstd_rand0, minstd_rand0, minstd_rand0)
VSMC_RNG_DEFINE_MACRO(::std::minstd_rand, minstd_rand, minstd_rand)
VSMC_RNG_DEFINE_MACRO(::std::ranlux24_base, ranlux24_base, ranlux24_base)
VSMC_RNG_DEFINE_MACRO(::std::ranlux48_base, ranlux48_base, ranlux48_base)
VSMC_RNG_DEFINE_MACRO(::std::ranlux24, ranlux24, ranlux24)
VSMC_RNG_DEFINE_MACRO(::std::ranlux48, ranlux48, ranlux48)
VSMC_RNG_DEFINE_MACRO(::std::knuth_b, knuth_b, knuth_b)

VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x32, Philox2x32, philox2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x32, Philox4x32, philox4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x64, Philox2x64, philox2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x64, Philox4x64, philox4x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x32_64, Philox2x32_64, philox2x32_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x32_64, Philox4x32_64, philox4x32_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox2x64_64, Philox2x64_64, philox2x64_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Philox4x64_64, Philox4x64_64, philox4x64_64)

VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x32, Threefry2x32, threefry2x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x32, Threefry4x32, threefry4x32)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry2x64, Threefry2x64, threefry2x64)
VSMC_RNG_DEFINE_MACRO(::vsmc::Threefry4x64, Threefry4x64, threefry4x64)
VSMC_RNG_DEFINE_MACRO(
    ::vsmc::Threefry2x32_64, Threefry2x32_64, threefry2x32_64)
VSMC_RNG_DEFINE_MACRO(
    ::vsmc::Threefry4x32_64, Threefry4x32_64, threefry4x32_64)
VSMC_RNG_DEFINE_MACRO(
    ::vsmc::Threefry2x64_64, Threefry2x64_64, threefry2x64_64)
VSMC_RNG_DEFINE_MACRO(
    ::vsmc::Threefry4x64_64, Threefry4x64_64, threefry4x64_64)

#if VSMC_HAS_AES_NI
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x1, AES128x1, aes128x1)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x2, AES128x2, aes128x2)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x4, AES128x4, aes128x4)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x8, AES128x8, aes128x8)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x1_64, AES128x1_64, aes128x1_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x2_64, AES128x2_64, aes128x2_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x4_64, AES128x4_64, aes128x4_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES128x8_64, AES128x8_64, aes128x8_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x1, AES192x1, aes192x1)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x2, AES192x2, aes192x2)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x4, AES192x4, aes192x4)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x8, AES192x8, aes192x8)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x1_64, AES192x1_64, aes192x1_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x2_64, AES192x2_64, aes192x2_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x4_64, AES192x4_64, aes192x4_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES192x8_64, AES192x8_64, aes192x8_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x1, AES256x1, aes256x1)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x2, AES256x2, aes256x2)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x4, AES256x4, aes256x4)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x8, AES256x8, aes256x8)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x1_64, AES256x1_64, aes256x1_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x2_64, AES256x2_64, aes256x2_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x4_64, AES256x4_64, aes256x4_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::AES256x8_64, AES256x8_64, aes256x8_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx1, ARSx1, arsx1)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx2, ARSx2, arsx2)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx4, ARSx4, arsx4)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx8, ARSx8, arsx8)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx1_64, ARSx1_64, arsx1_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx2_64, ARSx2_64, arsx2_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx4_64, ARSx4_64, arsx4_64)
VSMC_RNG_DEFINE_MACRO(::vsmc::ARSx8_64, ARSx8_64, arsx8_64)
#endif // VSMC_HAS_AES_NI

#if VSMC_HAS_RDRAND
VSMC_RNG_DEFINE_MACRO(::vsmc::RDRAND16, RDRAND16, rdrand16)
VSMC_RNG_DEFINE_MACRO(::vsmc::RDRAND32, RDRAND32, rdrand32)
VSMC_RNG_DEFINE_MACRO(::vsmc::RDRAND64, RDRAND64, rdrand64)
#endif // VSMC_HAS_RDRAND
