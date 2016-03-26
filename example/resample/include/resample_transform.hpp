//============================================================================
// vSMC/example/resample/include/resample_transform.hpp
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

#ifndef VSMC_EXAMPLE_RESAMPLE_TRANSFROM_HPP
#define VSMC_EXAMPLE_RESAMPLE_TRANSFROM_HPP

#include "resample_test.hpp"

inline bool resample_trans_rep_index_check(
    const vsmc::Vector<std::size_t> &rep, const vsmc::Vector<std::size_t> &idx)
{
    const std::size_t N = rep.size();
    const std::size_t M = idx.size();
    const std::size_t K = std::min(N, M);

    std::stringstream ss;
    ss << "Failed (N = " << N << ", M = " << M << ")";
    std::string fail(ss.str());

    std::size_t sum_rep =
        std::accumulate(rep.begin(), rep.end(), static_cast<std::size_t>(0));
    if (sum_rep != M) {
        std::cout << std::setw(40) << std::left << fail
                  << "sum(rep) = " << std::setw(10) << std::left << sum_rep
                  << "rep.back() = " << rep.back() << std::endl;
        return false;
    }

    for (std::size_t i = 0; i != K; ++i) {
        if (rep[i] > 0 && idx[i] != i) {
            std::cout << std::setw(40) << std::left << fail
                      << "idx[i] != i when rep[i] > 0" << std::endl;
            return false;
        }
    }

    for (std::size_t i = 0; i != N; ++i) {
        std::size_t r = 0;
        for (std::size_t j = 0; j != M; ++j)
            if (idx[j] == i)
                ++r;
        if (r != rep[i]) {
            std::cout << std::setw(40) << std::left << fail
                      << "i = " << std::setw(10) << std::left << i
                      << "sum(idx[j] == i) = " << std::setw(10) << std::left
                      << r << "rep[i] = " << rep[i] << std::endl;
            return false;
        }
    }

    return true;
}

template <vsmc::ResampleScheme Scheme>
inline void resample_trans_rep_index_test(
    std::size_t N, std::size_t n, const std::string &scheme, bool fixed)
{
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(60) << std::left << "Resampling scheme"
              << std::setw(20) << std::right << scheme << std::endl;
    std::cout << std::setw(60) << std::left << "Sample size" << std::setw(20)
              << std::right << (fixed ? "Fixed" : "Random") << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    vsmc::RNG rng;
    std::uniform_int_distribution<std::size_t> runif(N / 2, N * 2);
    vsmc::Vector<double> w(N);
    vsmc::Weight weight(N);
    vsmc::Vector<std::size_t> rep(N);
    vsmc::Vector<std::size_t> idx;
    vsmc::ResampleType<Scheme> resample;
    vsmc::StopWatch watch_resample;
    vsmc::StopWatch watch_trans;
    bool passed = true;
    for (std::size_t i = 0; i != n; ++i) {
        const std::size_t M = fixed ? N : runif(rng);
        idx.resize(M);
        vsmc::u01_distribution(rng, N, w.data());
        weight.set(w.data());

        watch_resample.start();
        resample(N, M, rng, weight.data(), rep.data());
        watch_resample.stop();

        watch_trans.start();
        vsmc::resample_trans_rep_index(N, M, rep.data(), idx.data());
        watch_trans.stop();

        passed = passed && resample_trans_rep_index_check(rep, idx);
    }

    std::cout << std::setw(60) << std::left
              << "Time (ms) in resampling: " << std::setw(20) << std::right
              << std::fixed << watch_resample.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left
              << "Time (ms) in transoform: " << std::setw(20) << std::right
              << std::fixed << watch_trans.milliseconds() << std::endl;
    std::cout << std::setw(60) << std::left << "Test result" << std::setw(20)
              << std::right << (passed ? "Passed" : "Failed") << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RESAMPLE_TRANSFROM_HPP
