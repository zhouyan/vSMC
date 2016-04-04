//============================================================================
// vSMC/example/rng/src/rng_mkl.cpp
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

#include "rng_mkl.hpp"

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

int main(int argc, char **argv)
{
    std::cout << std::string(110, '=') << std::endl;
    std::cout << std::setw(30) << std::left << "BRNG";
    std::cout << std::setw(20) << std::left << "vslLeapfrogStream";
    std::cout << std::setw(20) << std::left << "vslSkipAheadStream";
    std::cout << std::setw(20) << std::left << "viRngUniformBits32";
    std::cout << std::setw(20) << std::left << "viRngUniformBits64";
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_MCG31);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_R250);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_MRG32K3A);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_MCG59);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_WH);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_MT19937);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_MT2203);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_SFMT19937);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_SOBOL);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_NIEDERR);
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_NONDETERM);
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_ARS5);
#endif
    VSMC_RNG_MKL_FEATURES(VSL_BRNG_PHILOX4X32X10);
#endif

    std::cout << std::string(110, '=') << std::endl;
    std::cout << std::setw(30) << std::left << "BRNG";
    std::cout << std::setw(20) << std::left << "NSeeds";
    std::cout << std::setw(20) << std::left << "IncludesZero";
    std::cout << std::setw(20) << std::left << "WordSize";
    std::cout << std::setw(20) << std::left << "NBits";
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_MCG31);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_R250);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_MRG32K3A);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_MCG59);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_WH);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_MT19937);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_MT2203);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_SFMT19937);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_SOBOL);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_NIEDERR);
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_NONDETERM);
#if INTEL_MKL_VERSION >= 110300
#if VSMC_HAS_AES_NI
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_ARS5);
#endif
    VSMC_RNG_MKL_PROPERTIES(VSL_BRNG_PHILOX4X32X10);
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, name)                                  \
    int name = vsmc::mkl_brng<RNGType>();                                     \
    VSMC_RNG_MKL_PROPERTIES(name);

#if VSMC_HAS_RUNTIME_LIBRARY
#include <vsmc/rng/internal/rng_define_macro.hpp>
#endif

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
