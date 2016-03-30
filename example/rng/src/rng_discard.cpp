//============================================================================
// vSMC/example/rng/src/rng_discard.hpp
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

#include "rng_discard.hpp"

int main(int argc, char **argv)
{
    unsigned N = 1000;
    if (argc > 1)
        N = static_cast<unsigned>(std::atoi(argv[1]));

    std::size_t m = 1000;
    if (argc > 1)
        m = static_cast<std::size_t>(std::atoi(argv[2]));

    std::cout << std::string(80, '-') << std::endl;

    VSMC_RNG_DISCARD_TEST(vsmc::Philox2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::Philox4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::Philox2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::Philox4x64);

    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x64);
#if VSMC_HAS_SSE2
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x32SSE2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x32SSE2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x64SSE2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x64SSE2);
#endif
#if VSMC_HAS_AVX2
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x32AVX2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x32AVX2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry2x64AVX2);
    VSMC_RNG_DISCARD_TEST(vsmc::Threefry4x64AVX2);
#endif

#if VSMC_HAS_AES_NI
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_1x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_8x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_1x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_4x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES128_8x64);

    VSMC_RNG_DISCARD_TEST(vsmc::AES192_1x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_8x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_1x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_4x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES192_8x64);

    VSMC_RNG_DISCARD_TEST(vsmc::AES256_1x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_8x32);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_1x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_4x64);
    VSMC_RNG_DISCARD_TEST(vsmc::AES256_8x64);

    VSMC_RNG_DISCARD_TEST(vsmc::ARS_1x32);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_2x32);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_4x32);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_8x32);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_1x64);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_2x64);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_4x64);
    VSMC_RNG_DISCARD_TEST(vsmc::ARS_8x64);
#endif

#if VSMC_HAS_MKL
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MCG59);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MCG59_64);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MT19937);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MT19937_64);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MT2203);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_MT2203_64);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_SFMT19937);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_SFMT19937_64);
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_ARS5);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_ARS5_64);
#endif
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_PHILOX4X32X10);
    VSMC_RNG_DISCARD_TEST(vsmc::MKL_PHILOX4X32X10_64);
#endif
#endif

    return 0;
}
