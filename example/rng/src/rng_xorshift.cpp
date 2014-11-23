//============================================================================
// vSMC/example/rng/src/rng_xorshift.cpp
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
#include <vsmc/rng/xorshift.hpp>

int main (int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_xorshift);

    VSMC_RNG_TEST(vsmc::Xorshift1x32);
    VSMC_RNG_TEST(vsmc::Xorshift2x32);
    VSMC_RNG_TEST(vsmc::Xorshift4x32);
    VSMC_RNG_TEST(vsmc::Xorshift8x32);
    VSMC_RNG_TEST(vsmc::Xorshift16x32);
    VSMC_RNG_TEST(vsmc::Xorshift32x32);
    VSMC_RNG_TEST(vsmc::Xorshift64x32);
    VSMC_RNG_TEST(vsmc::Xorshift128x32);
    VSMC_RNG_TEST(vsmc::Xorshift1x64);
    VSMC_RNG_TEST(vsmc::Xorshift2x64);
    VSMC_RNG_TEST(vsmc::Xorshift4x64);
    VSMC_RNG_TEST(vsmc::Xorshift8x64);
    VSMC_RNG_TEST(vsmc::Xorshift16x64);
    VSMC_RNG_TEST(vsmc::Xorshift32x64);
    VSMC_RNG_TEST(vsmc::Xorshift64x64);

    VSMC_RNG_TEST(vsmc::Xorwow1x32);
    VSMC_RNG_TEST(vsmc::Xorwow2x32);
    VSMC_RNG_TEST(vsmc::Xorwow4x32);
    VSMC_RNG_TEST(vsmc::Xorwow8x32);
    VSMC_RNG_TEST(vsmc::Xorwow16x32);
    VSMC_RNG_TEST(vsmc::Xorwow32x32);
    VSMC_RNG_TEST(vsmc::Xorwow64x32);
    VSMC_RNG_TEST(vsmc::Xorwow128x32);
    VSMC_RNG_TEST(vsmc::Xorwow1x64);
    VSMC_RNG_TEST(vsmc::Xorwow2x64);
    VSMC_RNG_TEST(vsmc::Xorwow4x64);
    VSMC_RNG_TEST(vsmc::Xorwow8x64);
    VSMC_RNG_TEST(vsmc::Xorwow16x64);
    VSMC_RNG_TEST(vsmc::Xorwow32x64);
    VSMC_RNG_TEST(vsmc::Xorwow64x64);

    VSMC_RNG_TEST_POST;

    return 0;
}
