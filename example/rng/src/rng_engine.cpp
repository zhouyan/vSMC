//============================================================================
// vSMC/example/rng/src/rng_engine.cpp
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

#include "rng_engine.hpp"

int main(int argc, char **argv)
{
    std::size_t N = 1000000;
    if (argc > 1)
        N = static_cast<std::size_t>(std::atoi(argv[1]));

    const int nwid = 30;
    const int swid = 5;
    const int twid = 15;
    const std::size_t lwid = nwid + swid + twid * 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "RNGType";
    std::cout << std::right << std::setw(swid) << "Size";
    std::cout << std::right << std::setw(twid) << "N/ns (Loop)";
    std::cout << std::right << std::setw(twid) << "N/ns (Batch)";
    std::cout << std::right << std::setw(twid) << "GB/s (Loop)";
    std::cout << std::right << std::setw(twid) << "GB/s (Batch)";
    std::cout << std::right << std::setw(twid) << "Deterministics";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    rng_engine_test<RNGType>(N, #Name);

#include <vsmc/rng/internal/rng_define_macro.hpp>

#define VSMC_EXAMPLE_RNG_MKL_ENGINE(Name)                                     \
    VSMC_RNG_DEFINE_MACRO(vsmc::Name, Name, Name)

#if VSMC_HAS_MKL
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MCG59);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MCG59_64);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MT19937);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MT19937_64);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MT2203);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_MT2203_64);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_SFMT19937);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_SFMT19937_64);
#if VSMC_HAS_RDRAND
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_NONDETERM);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_NONDETERM_64);
#endif // VSMC_HAS_RDRAND
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_ARS5);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_ARS5_64);
#endif // VSMC_HAS_AES_NI
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_PHILOX4X32X10);
    VSMC_EXAMPLE_RNG_MKL_ENGINE(MKL_PHILOX4X32X10_64);
#endif // INTEL_MKL_VERSION >= 110300
#endif // VSMC_HAS_MKL

    std::cout << std::string(lwid, '-') << std::endl;

    return 0;
}
