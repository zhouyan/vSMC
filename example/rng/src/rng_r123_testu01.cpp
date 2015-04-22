//============================================================================
// vSMC/example/rng/src/rng_r123_testu01.cpp
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

#if VSMC_HAS_AES_NI
#define R123_USE_AES_NI 1
#include <Random123/aes.h>
#include <Random123/ars.h>
#endif

#include <Random123/philox.h>
#include <Random123/threefry.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4521)
#endif
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#define VSMC_RNG_TESTU01_FUNCTION_R123(Eng)                                   \
    extern "C" {                                                              \
    inline double rng_##Eng(void)                                             \
    {                                                                         \
        static r123::Engine<r123::Eng> eng;                                   \
        static std::uniform_real_distribution<double> runif(0, 1);            \
                                                                              \
        return runif(eng);                                                    \
    }                                                                         \
    }

#define VSMC_RNG_TESTU01_OPTION_R123(Eng)                                     \
    bool rng_testu01_##Eng = false;                                           \
    option.add(#Eng, "Test r123::Engine<r123::" #Eng ">", &rng_testu01_##Eng, \
        false);

#if VSMC_HAS_AES_NI
namespace r123
{
typedef ARS4x32_R<7> ARS4x32_R7;
}
VSMC_RNG_TESTU01_FUNCTION_R123(AESNI4x32)
VSMC_RNG_TESTU01_FUNCTION_R123(ARS4x32_R7)
#endif
VSMC_RNG_TESTU01_FUNCTION_R123(Philox2x32)
VSMC_RNG_TESTU01_FUNCTION_R123(Philox4x32)
VSMC_RNG_TESTU01_FUNCTION_R123(Philox2x64)
VSMC_RNG_TESTU01_FUNCTION_R123(Philox4x64)
VSMC_RNG_TESTU01_FUNCTION_R123(Threefry2x32)
VSMC_RNG_TESTU01_FUNCTION_R123(Threefry4x32)
VSMC_RNG_TESTU01_FUNCTION_R123(Threefry2x64)
VSMC_RNG_TESTU01_FUNCTION_R123(Threefry4x64)

int main(int argc, char **argv)
{
    VSMC_RNG_TESTU01_OPTION_PRE;

#if VSMC_HAS_AES_NI
    VSMC_RNG_TESTU01_OPTION_R123(AESNI4x32);
    VSMC_RNG_TESTU01_OPTION_R123(ARS4x32_R7);
#endif
    VSMC_RNG_TESTU01_OPTION_R123(Philox2x32);
    VSMC_RNG_TESTU01_OPTION_R123(Philox4x32);
    VSMC_RNG_TESTU01_OPTION_R123(Philox2x64);
    VSMC_RNG_TESTU01_OPTION_R123(Philox4x64);
    VSMC_RNG_TESTU01_OPTION_R123(Threefry2x32);
    VSMC_RNG_TESTU01_OPTION_R123(Threefry4x32);
    VSMC_RNG_TESTU01_OPTION_R123(Threefry2x64);
    VSMC_RNG_TESTU01_OPTION_R123(Threefry4x64);

    VSMC_RNG_TESTU01_OPTION_POST;

#if VSMC_HAS_AES_NI
    VSMC_RNG_TESTU01(AESNI4x32);
    VSMC_RNG_TESTU01(ARS4x32_R7);
#endif
    VSMC_RNG_TESTU01(Philox2x32);
    VSMC_RNG_TESTU01(Philox4x32);
    VSMC_RNG_TESTU01(Philox2x64);
    VSMC_RNG_TESTU01(Philox4x64);
    VSMC_RNG_TESTU01(Threefry2x32);
    VSMC_RNG_TESTU01(Threefry4x32);
    VSMC_RNG_TESTU01(Threefry2x64);
    VSMC_RNG_TESTU01(Threefry4x64);

    return 0;
}
