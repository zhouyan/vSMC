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

#include <vsmc/rng/mkl.hpp>

#define VSMC_RNG_MKL_BRNG_FEATURES(BRNG) rng_mkl_brng_features(BRNG, #BRNG)

#define VSMC_RNG_MKL_BRNG_PROPERTIES(BRNG) rng_mkl_brng_properties(BRNG, #BRNG)

inline void rng_mkl_brng_features(int brng, const std::string &name)
{
    bool has_u32 = vsmc::MKLStream::has_uniform_bits32(brng);
    bool has_u64 = vsmc::MKLStream::has_uniform_bits64(brng);
    bool has_skip_ahead = vsmc::MKLStream::has_skip_ahead(brng);
    bool has_leap_frog = vsmc::MKLStream::has_leap_frog(brng);
    std::cout << std::setw(30) << std::left << name;
    std::cout << std::setw(20) << std::left << (has_leap_frog ? "Yes" : "No");
    std::cout << std::setw(20) << std::left << (has_skip_ahead ? "Yes" : "No");
    std::cout << std::setw(20) << std::left << (has_u32 ? "Yes" : "No");
    std::cout << std::setw(20) << std::left << (has_u64 ? "Yes" : "No");
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
}

inline void rng_mkl_brng_properties(int brng, const std::string &name)
{
    VSLBRngProperties properties;
    vsmc::MKLStream::get_brng_properties(brng, &properties);
    std::cout << std::setw(30) << std::left << name;
    std::cout << std::setw(20) << std::left << properties.NSeeds;
    std::cout << std::setw(20) << std::left << properties.IncludesZero;
    std::cout << std::setw(20) << std::left << properties.WordSize;
    std::cout << std::setw(20) << std::left << properties.NBits;
    std::cout << std::endl;
    std::cout << std::string(110, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_MKL_BRNG_HPP
