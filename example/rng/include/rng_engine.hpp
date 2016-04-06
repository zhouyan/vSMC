//============================================================================
// vSMC/example/rng/include/rng_engine.hpp
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

#ifndef VSMC_EXAMPLE_RNG_ENGINE_HPP
#define VSMC_EXAMPLE_RNG_ENGINE_HPP

#include <vsmc/rng/engine.hpp>
#include <vsmc/utility/stop_watch.hpp>

template <typename RNGType>
inline void rng_engine(std::size_t N, std::size_t M, int nwid, int swid,
    int twid, const std::string &name)
{
    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<typename RNGType::result_type> r1;
    vsmc::Vector<typename RNGType::result_type> r2;
    r1.reserve(N);
    r2.reserve(N);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    bool passed = true;

    std::uniform_int_distribution<std::size_t> runif(N / 2, N);
    std::size_t num = 0;
    for (std::size_t i = 0; i != M; ++i) {
        std::size_t K = runif(rng);
        num += K;
        r1.resize(K);
        r2.resize(K);

        watch1.start();
        for (std::size_t j = 0; j != K; ++j)
            r1[j] = rng1();
        watch1.stop();

        watch2.start();
        vsmc::rng_rand(rng2, K, r2.data());
        watch2.stop();
        passed = passed && r1 == r2;

        std::stringstream ss;
        ss << rng;
        vsmc::rng_rand(rng, K, r1.data());
        ss >> rng;
        vsmc::rng_rand(rng, K, r2.data());
        passed = passed && r1 == r2;

        rng1.discard(static_cast<unsigned>(K));
        typename RNGType::result_type next = rng1();
        for (std::size_t j = 0; j != K; ++j)
            rng2();
        bool find = false;
        for (std::size_t j = 0; j != 2; ++j)
            find = find || rng2() == next;
        passed = passed && find;
    }

    double n1 = watch1.nanoseconds() / num;
    double n2 = watch2.nanoseconds() / num;
    double g1 = sizeof(typename RNGType::result_type) / n1;
    double g2 = sizeof(typename RNGType::result_type) / n2;
    std::string pass = passed ? "Passed" : "Failed";

    std::cout << std::left << std::setw(nwid) << name;
    std::cout << std::right << std::setw(swid) << sizeof(RNGType);
    std::cout << std::right << std::setw(twid) << std::fixed << n1;
    std::cout << std::right << std::setw(twid) << std::fixed << n2;
    std::cout << std::right << std::setw(twid) << std::fixed << g1;
    std::cout << std::right << std::setw(twid) << std::fixed << g2;
    std::cout << std::right << std::setw(twid) << pass;
    std::cout << std::endl;
}

inline void rng_engine(std::size_t N, std::size_t M)
{
    const int nwid = 30;
    const int swid = 5;
    const int twid = 15;
    const std::size_t lwid = nwid + swid + twid * 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "RNGType";
    std::cout << std::right << std::setw(swid) << "Size";
    std::cout << std::right << std::setw(twid) << "ns (Loop)";
    std::cout << std::right << std::setw(twid) << "ns (Batch)";
    std::cout << std::right << std::setw(twid) << "GB/s (Loop)";
    std::cout << std::right << std::setw(twid) << "GB/s (Batch)";
    std::cout << std::right << std::setw(twid) << "Deterministics";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_MKL
#undef VSMC_RNG_DEFINE_MACRO_MKL
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    rng_engine<RNGType>(N, M, nwid, swid, twid, #Name);

#include <vsmc/rng/internal/rng_define_macro.hpp>
#include <vsmc/rng/internal/rng_define_macro_mkl.hpp>

    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_ENGINE_HPP
