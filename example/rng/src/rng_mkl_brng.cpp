//============================================================================
// vSMC/example/rng/src/rng_mkl_brng.cpp
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

#include "rng_mkl_brng.hpp"

int main()
{
    std::cout << std::string(110, '=') << std::endl;
    std::cout << std::setw(30) << std::left << "BRNG";
    std::cout << std::setw(20) << std::left << "vslLeapfrogStream";
    std::cout << std::setw(20) << std::left << "vslSkipAheadStream";
    std::cout << std::setw(20) << std::left << "viRngUniformBits32";
    std::cout << std::setw(20) << std::left << "viRngUniformBits64";
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_MCG31);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_R250);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_MRG32K3A);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_MCG59);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_WH);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_MT19937);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_MT2203);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_SFMT19937);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_SOBOL);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_NIEDERR);
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_NONDETERM);
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_ARS5);
#endif
    VSMC_RNG_MKL_BRNG_FEATURES(VSL_BRNG_PHILOX4X32X10);
#endif

    std::cout << std::string(110, '=') << std::endl;
    std::cout << std::setw(30) << std::left << "BRNG";
    std::cout << std::setw(20) << std::left << "NSeeds";
    std::cout << std::setw(20) << std::left << "IncludesZero";
    std::cout << std::setw(20) << std::left << "WordSize";
    std::cout << std::setw(20) << std::left << "NBits";
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_MCG31);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_R250);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_MRG32K3A);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_MCG59);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_WH);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_MT19937);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_MT2203);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_SFMT19937);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_SOBOL);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_NIEDERR);
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_NONDETERM);
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_ARS5);
#endif
    VSMC_RNG_MKL_BRNG_PROPERTIES(VSL_BRNG_PHILOX4X32X10);
#endif

    std::cout << std::string(110, '=') << std::endl;
    std::cout << std::setw(30) << std::left << "RNGType";
    std::cout << std::setw(20) << std::left << "NSeeds";
    std::cout << std::setw(20) << std::left << "IncludesZero";
    std::cout << std::setw(20) << std::left << "WordSize";
    std::cout << std::setw(20) << std::left << "NBits";
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    {                                                                         \
        int brng = vsmc::mkl_brng<RNGType>();                                 \
        rng_mkl_brng_properties(brng, #Name);                                 \
    }

#include <vsmc/rng/internal/rng_define_macro.hpp>

    return 0;
}
