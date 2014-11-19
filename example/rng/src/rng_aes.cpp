//============================================================================
// vSMC/example/rng/src/rng_aes.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#include "rng_test.hpp"
#include <vsmc/rng/aes.hpp>

int main (int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_aes);

    VSMC_RNG_TEST(vsmc::AES128_1x32);
    VSMC_RNG_TEST(vsmc::AES128_2x32);
    VSMC_RNG_TEST(vsmc::AES128_4x32);
    VSMC_RNG_TEST(vsmc::AES128_8x32);
    VSMC_RNG_TEST(vsmc::AES128_1x64);
    VSMC_RNG_TEST(vsmc::AES128_2x64);
    VSMC_RNG_TEST(vsmc::AES128_4x64);
    VSMC_RNG_TEST(vsmc::AES128_8x64);

    VSMC_RNG_TEST(vsmc::AES192_1x32);
    VSMC_RNG_TEST(vsmc::AES192_2x32);
    VSMC_RNG_TEST(vsmc::AES192_4x32);
    VSMC_RNG_TEST(vsmc::AES192_8x32);
    VSMC_RNG_TEST(vsmc::AES192_1x64);
    VSMC_RNG_TEST(vsmc::AES192_2x64);
    VSMC_RNG_TEST(vsmc::AES192_4x64);
    VSMC_RNG_TEST(vsmc::AES192_8x64);

    VSMC_RNG_TEST(vsmc::AES256_1x32);
    VSMC_RNG_TEST(vsmc::AES256_2x32);
    VSMC_RNG_TEST(vsmc::AES256_4x32);
    VSMC_RNG_TEST(vsmc::AES256_8x32);
    VSMC_RNG_TEST(vsmc::AES256_1x64);
    VSMC_RNG_TEST(vsmc::AES256_2x64);
    VSMC_RNG_TEST(vsmc::AES256_4x64);
    VSMC_RNG_TEST(vsmc::AES256_8x64);

    VSMC_RNG_TEST_POST;

    return 0;
}
