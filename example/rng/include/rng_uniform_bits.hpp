//============================================================================
// vSMC/example/rng/include/rng_uniform_bits.hpp
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

#include "rng_common.hpp"

template <typename RNGType, typename UIntType>
inline void rng_uniform_bits(std::size_t N, std::size_t M, int nwid, int swid,
    int twid, const std::string &name)
{
    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<UIntType> r1;
    vsmc::Vector<UIntType> r2;
    r1.reserve(N);
    r2.reserve(N);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    bool pass = true;

    std::uniform_int_distribution<std::size_t> runif(N / 2, N);
    std::size_t num = 0;
    vsmc::UniformBitsDistribution<UIntType> rubits;
    for (std::size_t i = 0; i != M; ++i) {
        std::size_t K = runif(rng);
        num += K;
        r1.resize(K);
        r2.resize(K);

        watch1.start();
        for (std::size_t j = 0; j != K; ++j)
            r1[j] = rubits(rng1);
        watch1.stop();

        watch2.start();
        vsmc::rand(rng2, rubits, K, r2.data());
        watch2.stop();
        pass = pass && (r1 == r2 || rng != rng);
    }

    bool full = vsmc::RNGTraits<RNGType>::is_full_range;
    int rbits = vsmc::RNGTraits<RNGType>::bits;
    int tbits = std::numeric_limits<typename RNGType::result_type>::digits;
    int ubits = std::numeric_limits<UIntType>::digits;
    double bytes = static_cast<double>(sizeof(UIntType) * num);
    double g1 = bytes / watch1.nanoseconds();
    double g2 = bytes / watch2.nanoseconds();
    double c1 = watch1.cycles() / bytes;
    double c2 = watch2.cycles() / bytes;

    std::cout << std::setw(nwid) << std::left << name;
    std::cout << std::setw(swid) << std::right << (full ? "Yes" : "No");
    std::cout << std::setw(swid) << std::right << rbits;
    std::cout << std::setw(swid) << std::right << tbits;
    std::cout << std::setw(swid) << std::right << ubits;
    std::cout << std::setw(twid) << std::right << g1;
    std::cout << std::setw(twid) << std::right << g2;
    std::cout << std::setw(twid) << std::right << c1;
    std::cout << std::setw(twid) << std::right << c2;
    std::cout << std::setw(twid) << std::right << rng_pass(pass);
    std::cout << std::endl;
}

inline void rng_uniform_bits(std::size_t N, std::size_t M)
{
    const int nwid = 20;
    const int swid = 5;
    const int twid = 15;
    const std::size_t lwid = nwid + swid * 4 + twid * 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "RNGType";
    std::cout << std::setw(swid) << std::right << "Full";
    std::cout << std::setw(swid) << std::right << "R";
    std::cout << std::setw(swid) << std::right << "T";
    std::cout << std::setw(swid) << std::right << "U";
    std::cout << std::setw(twid) << std::right << "GB/s (Loop)";
    std::cout << std::setw(twid) << std::right << "GB/s (Batch)";
    std::cout << std::setw(twid) << std::right << "cpB (Loop)";
    std::cout << std::setw(twid) << std::right << "cpB (Batch)";
    std::cout << std::setw(twid) << std::right << "Deterministics";
    std::cout << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_MKL
#undef VSMC_RNG_DEFINE_MACRO_MKL
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    rng_uniform_bits<RNGType, std::uint8_t>(N, M, nwid, swid, twid, #Name);   \
    rng_uniform_bits<RNGType, std::uint16_t>(N, M, nwid, swid, twid, #Name);  \
    rng_uniform_bits<RNGType, std::uint32_t>(N, M, nwid, swid, twid, #Name);  \
    rng_uniform_bits<RNGType, std::uint64_t>(N, M, nwid, swid, twid, #Name);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::string(lwid, '-') << std::endl;

#include <vsmc/rng/internal/rng_define_macro_std.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

#include <vsmc/rng/internal/rng_define_macro_mkl.hpp>

    std::cout << std::string(lwid, '-') << std::endl;
}
