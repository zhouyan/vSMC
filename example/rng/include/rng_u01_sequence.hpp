//============================================================================
// vSMC/example/rng/include/rng_u01_sequence.hpp
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

#ifndef VSMC_EXAMPLE_RNG_U01_SEQUENCE_HPP
#define VSMC_EXAMPLE_RNG_U01_SEQUENCE_HPP

#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/u01_sequence.hpp>
#include <vsmc/utility/stop_watch.hpp>

template <template <typename, typename> class U01SeqType>
inline void rng_u01_sequence_test(
    std::size_t n, std::size_t k, std::size_t m, const std::string &name)
{
    vsmc::RNG rng;
    vsmc::StopWatch watch_1;
    vsmc::StopWatch watch_t;
    vsmc::StopWatch watch_g;
    double abs_err_t = 0;
    double abs_err_g = 0;
    double rel_err_t = 0;
    double rel_err_g = 0;

    for (std::size_t j = 0; j != m; ++j) {
        rng.seed(101);
        vsmc::Vector<double> r1(n);
        watch_1.start();
        U01SeqType<vsmc::RNG, double> u01seq(n, rng);
        for (std::size_t i = 0; i != n; ++i)
            r1[i] = u01seq[i];
        watch_1.stop();

        rng.seed(101);
        watch_t.start();
        vsmc::Vector<double> r2(n);
        vsmc::u01_oc_distribution(rng, k, r2.data());
        u01seq.transform(n, r2.data(), r2.data());
        watch_t.stop();
        for (std::size_t i = 0; i != n; ++i)
            abs_err_t = std::max(abs_err_t, std::abs(r1[i] - r2[i]));
        for (std::size_t i = 0; i != n; ++i)
            rel_err_t = std::max(rel_err_t, std::abs(r1[i] - r2[i]) / r1[i]);

        rng.seed(101);
        watch_g.start();
        vsmc::Vector<double> r3(n);
        u01seq.generate(rng, n, r3.data());
        watch_g.stop();
        for (std::size_t i = 0; i != n; ++i)
            abs_err_g = std::max(abs_err_g, std::abs(r1[i] - r3[i]));
        for (std::size_t i = 0; i != n; ++i)
            rel_err_g = std::max(rel_err_g, std::abs(r1[i] - r3[i]) / r1[i]);
    }

    std::cout << std::string(80, '=') << std::endl;
    std::cout << name << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << std::setw(60) << std::left << "operator[]" << std::setw(20)
              << std::right << std::fixed << watch_1.milliseconds()
              << std::endl;
    std::cout << std::setw(60) << std::left << "Time (ms): transform"
              << std::setw(20) << std::right << std::fixed
              << watch_t.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Time (ms): generate"
              << std::setw(20) << std::right << std::fixed
              << watch_g.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Absolute error: transform"
              << std::setw(20) << std::right << std::fixed << abs_err_t
              << std::endl;
    std::cout << std::setw(60) << std::left << "Absolute error: generate"
              << std::setw(20) << std::right << std::fixed << abs_err_g
              << std::endl;
    std::cout << std::setw(60) << std::left << "Relative error: transform"
              << std::setw(20) << std::right << std::fixed << rel_err_t
              << std::endl;
    std::cout << std::setw(60) << std::left << "Relative error: generate"
              << std::setw(20) << std::right << std::fixed << rel_err_g
              << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

inline void rng_u01_sequence_test(std::size_t n, std::size_t m)
{
    rng_u01_sequence_test<vsmc::U01SequenceSorted>(
        n, n, m, "U01SequenceSorted");
    rng_u01_sequence_test<vsmc::U01SequenceStratified>(
        n, n, m, "U01SequenceStratified");
    rng_u01_sequence_test<vsmc::U01SequenceSystematic>(
        n, 1, m, "U01SequenceSystematic");
}

#endif // VSMC_EXAMPLE_RNG_U01_SEQUENCE_HPP
