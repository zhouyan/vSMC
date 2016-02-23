//============================================================================
// vSMC/example/rng/src/rng_mkl.cpp
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

#include <vsmc/rng/mkl.hpp>
#include "rng_test.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_mkl);

    VSMC_RNG_TEST(vsmc::MKL_MCG59);
    VSMC_RNG_TEST(vsmc::MKL_MCG59_64);
    VSMC_RNG_TEST(vsmc::MKL_MT19937);
    VSMC_RNG_TEST(vsmc::MKL_MT19937_64);
    VSMC_RNG_TEST(vsmc::MKL_MT2203);
    VSMC_RNG_TEST(vsmc::MKL_MT2203_64);
    VSMC_RNG_TEST(vsmc::MKL_SFMT19937);
    VSMC_RNG_TEST(vsmc::MKL_SFMT19937_64);
#if VSMC_HAS_RDRAND
    VSMC_RNG_TEST(vsmc::MKL_NONDETERM);
    VSMC_RNG_TEST(vsmc::MKL_NONDETERM_64);
#endif
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_TEST(vsmc::MKL_ARS5);
    VSMC_RNG_TEST(vsmc::MKL_ARS5_64);
#endif
    VSMC_RNG_TEST(vsmc::MKL_PHILOX4X32X10);
    VSMC_RNG_TEST(vsmc::MKL_PHILOX4X32X10_64);
#endif

    VSMC_RNG_TEST_POST;

    return 0;
}
