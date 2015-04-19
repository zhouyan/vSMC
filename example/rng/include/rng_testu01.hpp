//============================================================================
// vSMC/example/rng/include/rng_testu01.hpp
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

#ifndef VSMC_EXAMPLE_RNG_TESTU01_HPP
#define VSMC_EXAMPLE_RNG_TESTU01_HPP

#include <vsmc/utility/program_option.hpp>

extern "C" {
#include <unif01.h>
#include <bbattery.h>
}

#define VSMC_RNG_TESTU01_FUNCTION(Eng)                                       \
    extern "C" {                                                             \
    inline double rng_##Eng(void)                                            \
    {                                                                        \
        static vsmc::Eng eng;                                                \
        static std::uniform_real_distribution<double> runif(0, 1);           \
                                                                             \
        return runif(eng);                                                   \
    }                                                                        \
    }

#define VSMC_RNG_TESTU01_OPTION_PRE                                          \
    vsmc::ProgramOptionMap option;                                           \
    bool SmallCrush = false;                                                 \
    bool Crush = false;                                                      \
    bool BigCrush = false;                                                   \
    bool DieHard = false;                                                    \
    bool FIPS_140_2 = false;                                                 \
    option.add("SmallCrush", "Test SmallCrush", &SmallCrush, false);         \
    option.add("Crush", "Test Crush", &Crush, false);                        \
    option.add("BigCrush", "Test BigCrush", &BigCrush, false);               \
    option.add("DieHard", "Test DieHard", &DieHard, false);                  \
    option.add("FIPS_140_2", "Test FIPS_140_2", &FIPS_140_2, false);

#define VSMC_RNG_TESTU01_OPTION(Eng)                                         \
    bool rng_testu01_##Eng = false;                                          \
    option.add(#Eng, "Test vsmc::" #Eng, &rng_testu01_##Eng, false);

#define VSMC_RNG_TESTU01_OPTION_POST option.process(argc, argv);

#define VSMC_RNG_TESTU01(Eng)                                                \
    if (rng_testu01_##Eng) {                                                 \
        char ename[] = #Eng;                                                 \
        unif01_Gen *gen = unif01_CreateExternGen01(ename, rng_##Eng);        \
        if (SmallCrush)                                                      \
            bbattery_SmallCrush(gen);                                        \
        if (Crush)                                                           \
            bbattery_Crush(gen);                                             \
        if (BigCrush)                                                        \
            bbattery_BigCrush(gen);                                          \
        if (DieHard)                                                         \
            bbattery_pseudoDIEHARD(gen);                                     \
        if (FIPS_140_2)                                                      \
            bbattery_FIPS_140_2(gen);                                        \
        unif01_DeleteExternGen01(gen);                                       \
    }

#endif  // VSMC_EXAMPLE_RNG_TESTU01_HPP
