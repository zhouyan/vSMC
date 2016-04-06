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

template <typename RealType>
inline std::string rng_u01_sequence_err(RealType x)
{
    std::stringstream ss;
    ss << x / std::numeric_limits<RealType>::epsilon() << " eps";

    return ss.str();
}

template <template <typename, typename> class U01SeqType, typename RealType>
inline void rng_u01_sequence(std::size_t N, std::size_t K, std::size_t M,
    int nwid, int twid, const std::string &name)
{
    vsmc::RNG rng;
    vsmc::StopWatch watch_1;
    vsmc::StopWatch watch_t;
    vsmc::StopWatch watch_g;
    RealType abs_err_t = 0;
    RealType abs_err_g = 0;

    std::uniform_int_distribution<unsigned> runif(0, 100);
    for (std::size_t i = 0; i != M; ++i) {
        unsigned s = runif(rng);

        rng.seed(s);
        vsmc::Vector<RealType> r1(N);
        watch_1.start();
        U01SeqType<vsmc::RNG, RealType> u01seq(N, rng);
        for (std::size_t j = 0; j != N; ++j)
            r1[j] = u01seq[j];
        watch_1.stop();

        rng.seed(s);
        watch_t.start();
        vsmc::Vector<RealType> r2(N);
        vsmc::u01_oc_distribution(rng, K, r2.data());
        u01seq.transform(N, r2.data(), r2.data());
        watch_t.stop();
        for (std::size_t j = 0; j != N; ++j)
            abs_err_t = std::max(abs_err_t, std::abs(r1[j] - r2[j]));

        rng.seed(s);
        watch_g.start();
        vsmc::Vector<RealType> r3(N);
        u01seq.generate(rng, N, r3.data());
        watch_g.stop();
        for (std::size_t j = 0; j != N; ++j)
            abs_err_g = std::max(abs_err_g, std::abs(r1[j] - r3[j]));
    }

    double n_1 = watch_1.nanoseconds() / (N * M);
    double n_t = watch_t.nanoseconds() / (N * M);
    double n_g = watch_g.nanoseconds() / (N * M);
    std::string abs_err_t_str = rng_u01_sequence_err(abs_err_t);
    std::string abs_err_g_str = rng_u01_sequence_err(abs_err_g);
    if (sizeof(RealType) == sizeof(float))
        std::cout << std::setw(twid) << std::left << "float";
    if (sizeof(RealType) == sizeof(double))
        std::cout << std::setw(twid) << std::left << "double";
    if (sizeof(RealType) == sizeof(long double))
        std::cout << std::setw(twid) << std::left << "long double";
    std::cout << std::setw(nwid) << std::left << name;
    std::cout << std::setw(twid) << std::right << std::fixed << n_1;
    std::cout << std::setw(twid) << std::right << std::fixed << n_t;
    std::cout << std::setw(twid) << std::right << std::fixed << n_g;
    std::cout << std::setw(twid) << std::right << abs_err_t_str;
    std::cout << std::setw(twid) << std::right << abs_err_g_str;
    std::cout << std::endl;
}

template <typename RealType>
inline void rng_u01_sequence(std::size_t N, std::size_t M, int nwid, int twid)
{
    rng_u01_sequence<vsmc::U01SequenceSorted, RealType>(
        N, N, M, nwid, twid, "Sorted");
    rng_u01_sequence<vsmc::U01SequenceStratified, RealType>(
        N, N, M, nwid, twid, "Stratified");
    rng_u01_sequence<vsmc::U01SequenceSystematic, RealType>(
        N, 1, M, nwid, twid, "Systematic");
}

inline void rng_u01_sequence(std::size_t N, std::size_t M)
{
    const int nwid = 30;
    const int twid = 15;
    const std::size_t lwid = nwid + twid * 6;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(twid) << std::left << "Precision";
    std::cout << std::setw(nwid) << std::left << "Algorithm";
    std::cout << std::setw(twid) << std::right << "ns (operator[])";
    std::cout << std::setw(twid) << std::right << "ns (t)";
    std::cout << std::setw(twid) << std::right << "ns (g)";
    std::cout << std::setw(twid) << std::right << "abs.err (t)";
    std::cout << std::setw(twid) << std::right << "abs.err (g)";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
    rng_u01_sequence<float>(N, M, nwid, twid);
    rng_u01_sequence<double>(N, M, nwid, twid);
    rng_u01_sequence<long double>(N, M, nwid, twid);
    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_U01_SEQUENCE_HPP
