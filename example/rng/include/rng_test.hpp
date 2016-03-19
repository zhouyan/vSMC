//============================================================================
// vSMC/example/rng/include/rng_test.hpp
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

#ifndef VSMC_EXAMPLE_RNG_TEST_HPP
#define VSMC_EXAMPLE_RNG_TEST_HPP

#include <vsmc/rng/uniform_real_distribution.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_RNG_TEST_PRE(prog)                                               \
    std::size_t N = 1000000;                                                  \
    if (argc > 1)                                                             \
        N = static_cast<std::size_t>(std::atoi(argv[1]));                     \
    std::string name(#prog);                                                  \
    vsmc::Vector<std::string> rng_name;                                       \
    vsmc::Vector<std::size_t> rng_size;                                       \
    vsmc::Vector<std::size_t> result_size;                                    \
    vsmc::Vector<std::size_t> num;                                            \
    vsmc::Vector<bool> test;                                                  \
    vsmc::Vector<vsmc::StopWatch> sw1;                                        \
    vsmc::Vector<vsmc::StopWatch> sw2;

#define VSMC_RNG_TEST(RNGType)                                                \
    rng_test<RNGType>(                                                        \
        N, #RNGType, rng_name, rng_size, result_size, num, test, sw1, sw2);

#define VSMC_RNG_TEST_POST                                                    \
    rng_test_output(                                                          \
        name, rng_name, rng_size, result_size, num, test, sw1, sw2);

template <typename RNGType>
inline void rng_test(std::size_t n, const std::string &name,
    vsmc::Vector<std::string> &rng_name, vsmc::Vector<std::size_t> &rng_size,
    vsmc::Vector<std::size_t> &result_size, vsmc::Vector<std::size_t> &num,
    vsmc::Vector<bool> &test, vsmc::Vector<vsmc::StopWatch> &sw1,
    vsmc::Vector<vsmc::StopWatch> &sw2)
{
    rng_name.push_back(name);
    rng_size.push_back(sizeof(RNGType));
    result_size.push_back(sizeof(typename RNGType::result_type));

    RNGType rng;
    RNGType rng1;
    RNGType rng2;
    vsmc::Vector<typename RNGType::result_type> r1(n * 2);
    vsmc::Vector<typename RNGType::result_type> r2(n * 2);
    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;
    bool passed = true;
    std::size_t number = 0;
    std::uniform_int_distribution<std::size_t> runif(n, n * 2 - 1);
    for (std::size_t i = 0; i != 10; ++i) {
        std::size_t m = runif(rng);

        watch1.start();
        for (std::size_t j = 0; j != m; ++j)
            r1[j] = rng1();
        watch1.stop();

        watch2.start();
        vsmc::rng_rand(rng2, m, r2.data());
        watch2.stop();

        number += m;
        for (std::size_t j = 0; j != m; ++j)
            if (r1[j] != r2[j])
                passed = false;
    }
    sw1.push_back(watch1);
    sw2.push_back(watch2);
    num.push_back(number);
    test.push_back(passed);
}

inline void rng_test_output(const std::string &name,
    const vsmc::Vector<std::string> &rng_name,
    const vsmc::Vector<std::size_t> &rng_size,
    const vsmc::Vector<std::size_t> &result_size,
    const vsmc::Vector<std::size_t> &num, const vsmc::Vector<bool> &test,
    const vsmc::Vector<vsmc::StopWatch> &sw1,
    const vsmc::Vector<vsmc::StopWatch> &sw2)
{
    const int nwid = 30;
    const int swid = 5;
    const int twid = 15;
    const std::size_t lwid = nwid + swid + twid * 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << name;
    std::cout << std::right << std::setw(swid) << "Size";
    std::cout << std::right << std::setw(twid) << "N/ns (Loop)";
    std::cout << std::right << std::setw(twid) << "N/ns (Batch)";
    std::cout << std::right << std::setw(twid) << "GB/s (Loop)";
    std::cout << std::right << std::setw(twid) << "GB/s (Batch)";
    std::cout << std::right << std::setw(twid) << "Test";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;

    for (std::size_t i = 0; i != rng_name.size(); ++i) {
        double n1 = num[i] / sw1[i].nanoseconds();
        double n2 = num[i] / sw2[i].nanoseconds();
        double g1 = result_size[i] * num[i] / sw1[i].nanoseconds();
        double g2 = result_size[i] * num[i] / sw2[i].nanoseconds();
        std::string ts = test[i] ? "Passed" : "Failed";
        std::cout << std::left << std::setw(nwid) << rng_name[i];
        std::cout << std::right << std::setw(swid) << rng_size[i];
        std::cout << std::right << std::setw(twid) << std::fixed << n1;
        std::cout << std::right << std::setw(twid) << std::fixed << n2;
        std::cout << std::right << std::setw(twid) << std::fixed << g1;
        std::cout << std::right << std::setw(twid) << std::fixed << g2;
        std::cout << std::right << std::setw(twid) << ts;
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_TEST_HPP
