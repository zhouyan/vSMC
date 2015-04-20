//============================================================================
// vSMC/example/rng/src/rng_std_testu01.cpp
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

#include "rng_testu01.hpp"

#define VSMC_RNG_TESTU01_FUNCTION_STD(Eng)                                 \
    extern "C" {                                                             \
    inline double rng_##Eng(void)                                            \
    {                                                                        \
        static std::Eng eng;                                                 \
        static std::uniform_real_distribution<double> runif(0, 1);           \
                                                                             \
        return runif(eng);                                                   \
    }                                                                        \
    }

#define VSMC_RNG_TESTU01_OPTION_STD(Eng)                                   \
    bool rng_testu01_##Eng = false;                                          \
    option.add(#Eng, "Test std::" #Eng, &rng_testu01_##Eng, false);

VSMC_RNG_TESTU01_FUNCTION_STD(mt19937)
VSMC_RNG_TESTU01_FUNCTION_STD(mt19937_64)
VSMC_RNG_TESTU01_FUNCTION_STD(minstd_rand0)
VSMC_RNG_TESTU01_FUNCTION_STD(minstd_rand)
VSMC_RNG_TESTU01_FUNCTION_STD(ranlux24_base)
VSMC_RNG_TESTU01_FUNCTION_STD(ranlux48_base)
VSMC_RNG_TESTU01_FUNCTION_STD(ranlux24)
VSMC_RNG_TESTU01_FUNCTION_STD(ranlux48)
VSMC_RNG_TESTU01_FUNCTION_STD(knuth_b)

int main(int argc, char **argv)
{
    VSMC_RNG_TESTU01_OPTION_PRE;

    VSMC_RNG_TESTU01_OPTION_STD(mt19937);
    VSMC_RNG_TESTU01_OPTION_STD(mt19937_64);
    VSMC_RNG_TESTU01_OPTION_STD(minstd_rand0);
    VSMC_RNG_TESTU01_OPTION_STD(minstd_rand);
    VSMC_RNG_TESTU01_OPTION_STD(ranlux24_base);
    VSMC_RNG_TESTU01_OPTION_STD(ranlux48_base);
    VSMC_RNG_TESTU01_OPTION_STD(ranlux24);
    VSMC_RNG_TESTU01_OPTION_STD(ranlux48);
    VSMC_RNG_TESTU01_OPTION_STD(knuth_b);

    VSMC_RNG_TESTU01_OPTION_POST;

    VSMC_RNG_TESTU01(mt19937);
    VSMC_RNG_TESTU01(mt19937_64);
    VSMC_RNG_TESTU01(minstd_rand0);
    VSMC_RNG_TESTU01(minstd_rand);
    VSMC_RNG_TESTU01(ranlux24_base);
    VSMC_RNG_TESTU01(ranlux48_base);
    VSMC_RNG_TESTU01(ranlux24);
    VSMC_RNG_TESTU01(ranlux48);
    VSMC_RNG_TESTU01(knuth_b);

    return 0;
}
