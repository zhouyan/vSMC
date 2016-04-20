//============================================================================
// vSMC/example/rng/include/rng_mkl_brng.hpp
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

#ifndef VSMC_EXAMPLE_RNG_MKL_BRNG_HPP
#define VSMC_EXAMPLE_RNG_MKL_BRNG_HPP

#include <vsmc/rng/engine.hpp>

#define VSMC_RNG_MKL_BRNG_FEATURES(BRNG)                                      \
    rng_mkl_brng_features(BRNG, #BRNG, nwid, twid)

#define VSMC_RNG_MKL_BRNG_PROPERTIES(BRNG)                                    \
    rng_mkl_brng_properties(BRNG, #BRNG, nwid, twid)

inline void rng_mkl_brng_features(
    int brng, const std::string &name, int nwid, int twid)
{
    bool has_u32 = vsmc::MKLStream::has_uniform_bits32(brng);
    bool has_u64 = vsmc::MKLStream::has_uniform_bits64(brng);
    bool has_skip_ahead = vsmc::MKLStream::has_skip_ahead(brng);
    bool has_leap_frog = vsmc::MKLStream::has_leap_frog(brng);
    std::cout << std::setw(nwid) << std::left << name;
    std::cout << std::setw(twid) << std::right
              << (has_leap_frog ? "Yes" : "No");
    std::cout << std::setw(twid) << std::right
              << (has_skip_ahead ? "Yes" : "No");
    std::cout << std::setw(twid) << std::right << (has_u32 ? "Yes" : "No");
    std::cout << std::setw(twid) << std::right << (has_u64 ? "Yes" : "No");
    std::cout << std::endl;
}

inline void rng_mkl_brng_properties(
    int brng, const std::string &name, int nwid, int twid)
{
    VSLBRngProperties properties;
    vsmc::MKLStream::get_brng_properties(brng, &properties);
    std::cout << std::setw(nwid) << std::left << name;
    std::cout << std::setw(twid) << std::right << properties.NSeeds;
    std::cout << std::setw(twid) << std::right << properties.IncludesZero;
    std::cout << std::setw(twid) << std::right << properties.WordSize;
    std::cout << std::setw(twid) << std::right << properties.NBits;
    std::cout << std::endl;
}

inline void rng_mkl_brng()
{
    const int nwid = 25;
    const int twid = 10;
    const std::size_t lwid = nwid + twid * 4;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "BRNG";
    std::cout << std::setw(twid) << std::right << "leap";
    std::cout << std::setw(twid) << std::right << "skip";
    std::cout << std::setw(twid) << std::right << "ubits32";
    std::cout << std::setw(twid) << std::right << "ubits64";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
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
    std::cout << std::string(lwid, '-') << std::endl;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "BRNG";
    std::cout << std::setw(twid) << std::right << "NSeeds";
    std::cout << std::setw(twid) << std::right << "Zero";
    std::cout << std::setw(twid) << std::right << "WordSize";
    std::cout << std::setw(twid) << std::right << "NBits";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
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
    std::cout << std::string(lwid, '-') << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    {                                                                         \
        int brng = vsmc::mkl_brng<RNGType>();                                 \
        rng_mkl_brng_properties(brng, #Name, nwid, twid);                     \
    }

#include <vsmc/rng/internal/rng_define_macro.hpp>

    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_MKL_BRNG_HPP
