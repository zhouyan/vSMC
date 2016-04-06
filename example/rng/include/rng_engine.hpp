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
inline void rng_engine_test(
    std::size_t N, std::size_t M, const std::string &name)
{
    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<typename RNGType::result_type> r1(N);
    vsmc::Vector<typename RNGType::result_type> r2(N);
    vsmc::Vector<double> r3(N);
    vsmc::Vector<double> r4(N);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    vsmc::StopWatch watch3;
    vsmc::StopWatch watch4;
    bool passed = true;

    std::uniform_int_distribution<std::size_t> runif(N / 2, N - 1);
    for (std::size_t i = 0; i != M; ++i) {
        watch1.start();
        for (std::size_t j = 0; j != N; ++j)
            r1[j] = rng1();
        watch1.stop();

        watch2.start();
        vsmc::rng_rand(rng2, N, r2.data());
        watch2.stop();
        passed = passed && r1 == r2;

        vsmc::U01Distribution<double> u01;
        watch3.start();
        for (std::size_t j = 0; j != N; ++j)
            r3[j] = u01(rng);
        watch3.stop();

        watch4.start();
        u01(rng, N, r4.data());
        watch4.stop();

        std::size_t k = runif(rng);

        std::fill(r1.begin(), r1.end(), 0);
        std::fill(r2.begin(), r2.end(), 0);
        std::stringstream ss;
        ss << rng;
        vsmc::rng_rand(rng, k, r1.data());
        ss >> rng;
        vsmc::rng_rand(rng, k, r2.data());
        passed = passed && r1 == r2;

        rng1.discard(static_cast<unsigned>(k));
        typename RNGType::result_type next = rng1();
        for (std::size_t j = 0; j != k; ++j)
            rng2();
        bool find = false;
        for (std::size_t j = 0; j != 2; ++j)
            find = find || rng2() == next;
        passed = passed && find;
    }

    const int nwid = 30;
    const int swid = 5;
    const int twid = 15;

    double n1 = watch1.nanoseconds() / (N * M);
    double n2 = watch2.nanoseconds() / (N * M);
    double u1 = watch3.nanoseconds() / (N * M);
    double u2 = watch4.nanoseconds() / (N * M);
    double g1 =
        N * M * sizeof(typename RNGType::result_type) / watch1.nanoseconds();
    double g2 =
        N * M * sizeof(typename RNGType::result_type) / watch2.nanoseconds();
    std::string pass = passed ? "Passed" : "Failed";

    std::cout << std::left << std::setw(nwid) << name;
    std::cout << std::right << std::setw(swid) << sizeof(RNGType);
    std::cout << std::right << std::setw(twid) << std::fixed << n1;
    std::cout << std::right << std::setw(twid) << std::fixed << n2;
    std::cout << std::right << std::setw(twid) << std::fixed << u1;
    std::cout << std::right << std::setw(twid) << std::fixed << u2;
    std::cout << std::right << std::setw(twid) << std::fixed << g1;
    std::cout << std::right << std::setw(twid) << std::fixed << g2;
    std::cout << std::right << std::setw(twid) << pass;
    std::cout << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_ENGINE_HPP
