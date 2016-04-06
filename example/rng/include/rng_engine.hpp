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
inline void rng_engine_test(std::size_t n, const std::string &name)
{
    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<typename RNGType::result_type> r1(n * 2, 0);
    vsmc::Vector<typename RNGType::result_type> r2(n * 2, 0);
    vsmc::Vector<double> r3(n * 2);
    vsmc::Vector<double> r4(n * 2);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    vsmc::StopWatch watch3;
    vsmc::StopWatch watch4;
    bool passed = true;
    std::size_t num = 0;

    std::uniform_int_distribution<std::size_t> runif(n, n * 2 - 1);
    for (std::size_t i = 0; i != 10; ++i) {
        std::size_t m = runif(rng);
        num += m;

        watch1.start();
        for (std::size_t j = 0; j != m; ++j)
            r1[j] = rng1();
        watch1.stop();

        watch2.start();
        vsmc::rng_rand(rng2, m, r2.data());
        watch2.stop();
        passed = passed && r1 == r2;

        vsmc::U01Distribution<double> u01;
        watch3.start();
        for (std::size_t j = 0; j != m; ++j)
            r3[j] = u01(rng);
        watch3.stop();

        watch4.start();
        u01(rng, m, r4.data());
        watch4.stop();

        std::stringstream ss;
        ss << rng;
        vsmc::rng_rand(rng, m, r1.data());
        ss >> rng;
        vsmc::rng_rand(rng, m, r2.data());
        passed = passed && r1 == r2;

        rng1.discard(1001);
        typename RNGType::result_type next = rng1();
        for (std::size_t j = 0; j != 1001; ++j)
            rng2();
        bool find = false;
        for (std::size_t j = 0; j != 2; ++j)
            find = find || rng2() == next;
        passed = passed && find;
    }

    const int nwid = 30;
    const int swid = 5;
    const int twid = 15;

    double n1 = num / watch1.seconds();
    double n2 = num / watch2.seconds();
    double u1 = num / watch3.seconds();
    double u2 = num / watch4.seconds();
    double g1 = n1 * sizeof(typename RNGType::result_type);
    double g2 = n2 * sizeof(typename RNGType::result_type);
    std::string pass = passed ? "Passed" : "Failed";

    std::cout << std::left << std::setw(nwid) << name;
    std::cout << std::right << std::setw(swid) << sizeof(RNGType);
    std::cout << std::right << std::setw(twid) << std::fixed << n1 / 1e9;
    std::cout << std::right << std::setw(twid) << std::fixed << n2 / 1e9;
    std::cout << std::right << std::setw(twid) << std::fixed << u1 / 1e9;
    std::cout << std::right << std::setw(twid) << std::fixed << u2 / 1e9;
    std::cout << std::right << std::setw(twid) << std::fixed << g1 / 1e9;
    std::cout << std::right << std::setw(twid) << std::fixed << g2 / 1e9;
    std::cout << std::right << std::setw(twid) << pass;
    std::cout << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_ENGINE_HPP
