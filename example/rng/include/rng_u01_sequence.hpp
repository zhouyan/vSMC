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

#include "rng_common.hpp"

template <typename U01SeqType, typename RealType>
inline void rng_u01_sequence(std::size_t N, std::size_t M, std::size_t L,
    int twid, const std::string &name)
{
    vsmc::RNG rng;
    std::size_t num = 0;
    RealType error = 0;
    std::uniform_int_distribution<std::size_t> rsize(N / 2, N);
    U01SeqType u01seq;

    vsmc::RNG rng1;
    vsmc::RNG rng2;

    vsmc::StopWatch watch1;
    vsmc::StopWatch watch2;

    vsmc::Vector<RealType> r1;
    vsmc::Vector<RealType> r2;
    r1.reserve(N);
    r2.reserve(N);

    for (std::size_t i = 0; i != M; ++i) {
        std::size_t K = rsize(rng);
        num += K;
        r1.resize(K);
        r2.resize(K);

        watch1.start();
        vsmc::u01_distribution(rng1, std::min(L, K), r1.data());
        u01seq(K, r1.data(), r1.data());
        watch1.stop();

        watch2.start();
        u01seq(rng2, K, r2.data());
        watch2.stop();

        for (std::size_t j = 0; j != K; ++j)
            error = std::max(error, std::abs(r1[j] - r2[j]));
    }
    std::stringstream ss;
    ss << error / std::numeric_limits<RealType>::epsilon() << " eps";

    double c1 = watch1.cycles() / num;
    double c2 = watch2.cycles() / num;
    std::cout << std::setw(twid) << std::left << rng_type_name<RealType>();
    std::cout << std::setw(twid) << std::left << name;
    std::cout << std::setw(twid) << std::right << std::fixed << c1;
    std::cout << std::setw(twid) << std::right << std::fixed << c2;
    std::cout << std::setw(twid) << std::right << ss.str();
    std::cout << std::endl;
}

template <typename RealType>
inline void rng_u01_sequence(std::size_t N, std::size_t M, int twid)
{
    rng_u01_sequence<vsmc::U01SequenceSorted, RealType>(
        N, M, N, twid, "Sorted");
    rng_u01_sequence<vsmc::U01SequenceStratified, RealType>(
        N, M, N, twid, "Stratified");
    rng_u01_sequence<vsmc::U01SequenceSystematic, RealType>(
        N, M, 1, twid, "Systematic");
}

inline void rng_u01_sequence(std::size_t N, std::size_t M)
{
    const int twid = 15;
    const std::size_t lwid = twid * 5;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(twid) << std::left << "Precision";
    std::cout << std::setw(twid) << std::left << "Algorithm";
    std::cout << std::setw(twid) << std::right << "cpE (Transform)";
    std::cout << std::setw(twid) << std::right << "cpE (Generate)";
    std::cout << std::setw(twid) << std::right << "Error";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
    rng_u01_sequence<float>(N, M, twid);
    std::cout << std::string(lwid, '-') << std::endl;
    rng_u01_sequence<double>(N, M, twid);
    std::cout << std::string(lwid, '-') << std::endl;
    rng_u01_sequence<long double>(N, M, twid);
    std::cout << std::string(lwid, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_U01_SEQUENCE_HPP
