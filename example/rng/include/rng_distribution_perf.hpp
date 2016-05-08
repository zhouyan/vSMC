//============================================================================
// vSMC/example/rng/include/rng_distribution_perf.hpp
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

#include "rng_distribution.hpp"

template <typename vSMCDistType, typename RealType, std::size_t ParamNum>
inline void rng_distribution_test_perf(std::size_t N, std::size_t M,
    const std::array<RealType, ParamNum> &param, int nwid, int twid)
{
    using trait_type = DistTrait<vSMCDistType>;
    using STDDistType = typename trait_type::std_type;

    vsmc::RNG rng;
    RNG01<vsmc::RNG> rng01;
#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
#endif
    std::uniform_int_distribution<std::size_t> rsize(N / 2, N);
    vSMCDistType dist_vsmc(rng_distribution_init<vSMCDistType>(param));
    STDDistType dist_std(rng_distribution_init<STDDistType>(param));
    bool pass = true;

    vsmc::Vector<RealType> r(N);
    vsmc::rand(rng01, dist_vsmc, N, r.data());
    for (std::size_t i = 0; i != N; ++i) {
        pass = pass && std::isfinite(r[i]);
        pass = pass && std::isfinite(dist_vsmc(rng01));
    }

    vsmc::Vector<RealType> r1;
    vsmc::Vector<RealType> r2;
    vsmc::Vector<RealType> r3;
#if VSMC_HAS_MKL
    vsmc::Vector<RealType> r4;
#endif
    r1.reserve(N);
    r2.reserve(N);
    r3.reserve(N);
#if VSMC_HAS_MKL
    r4.reserve(N);
#endif

    double c1 = std::numeric_limits<double>::max();
    double c2 = std::numeric_limits<double>::max();
    double c3 = std::numeric_limits<double>::max();
#if VSMC_HAS_MKL
    double c4 = std::numeric_limits<double>::max();
#endif
    for (std::size_t k = 0; k != 10; ++k) {
        std::size_t num = 0;
        vsmc::StopWatch watch1;
        vsmc::StopWatch watch2;
        vsmc::StopWatch watch3;
#if VSMC_HAS_MKL
        vsmc::StopWatch watch4;
#endif
        for (std::size_t i = 0; i != M; ++i) {
            std::size_t K = rsize(rng);
            num += K;
            r1.resize(K);
            r2.resize(K);
            r3.resize(K);
#if VSMC_HAS_MKL
            r4.resize(K);
#endif

            watch1.start();
            for (std::size_t j = 0; j != K; ++j)
                r1[j] = dist_std(rng);
            watch1.stop();

            watch2.start();
            for (std::size_t j = 0; j != K; ++j)
                r2[j] = dist_vsmc(rng);
            watch2.stop();
            pass = pass && r1 != r2;

            watch3.start();
            vsmc::rand(rng, dist_vsmc, K, r3.data());
            watch3.stop();
            pass = pass && r1 != r3;

#if VSMC_HAS_MKL
            watch4.start();
            vsmc::rand(rng_mkl, dist_vsmc, K, r4.data());
            watch4.stop();
            pass = pass && r1 != r4;
#endif

            vsmc::RNG rng1(rng);
            vsmc::RNG rng2(rng);
            std::stringstream ss;
            ss.precision(20);
            ss << dist_vsmc;
            for (std::size_t j = 0; j != K; ++j)
                r1[j] = dist_vsmc(rng1);
            ss >> dist_vsmc;
            for (std::size_t j = 0; j != K; ++j)
                r2[j] = dist_vsmc(rng2);
            pass = pass && r1 == r2;
        }
        c1 = std::min(c1, watch1.cycles() / num);
        c2 = std::min(c2, watch2.cycles() / num);
        c3 = std::min(c3, watch3.cycles() / num);
#if VSMC_HAS_MKL
        c4 = std::min(c4, watch4.cycles() / num);
#endif
    }

    trait_type trait;
    std::cout << std::setw(nwid) << std::left << trait.name(param);
    std::cout << std::setw(twid) << std::right << c1;
    std::cout << std::setw(twid) << std::right << c2;
    std::cout << std::setw(twid) << std::right << c3;
#if VSMC_HAS_MKL
    std::cout << std::setw(twid) << std::right << c4;
#endif
    std::cout << std::setw(twid + 3) << std::right << rng_pass(pass);
    std::cout << std::endl;
}

template <typename vSMCDistType>
inline void rng_distribution_test_perf(
    std::size_t N, std::size_t M, int nwid, int twid)
{
    DistTrait<vSMCDistType> trait;
    auto params = trait.params();

    vsmc::Vector<std::string> names;
    for (const auto &param : params)
        rng_distribution_test_perf<vSMCDistType>(N, M, param, nwid, twid);
}

template <typename RealType>
inline void rng_distribution_perf(
    std::size_t N, std::size_t M, int nwid, int twid)
{
    VSMC_DEFINE_EXAMPLE_RNG_DISTRIBUTION_TEST(perf);
}

inline void rng_distribution_perf(std::size_t N, std::size_t M)
{
    int nwid = 30;
    int twid = 12;

    std::size_t lwid =
        static_cast<std::size_t>(nwid + twid * (4 + VSMC_HAS_MKL) + 3);

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::setw(nwid) << std::left << "Distribution";
    std::cout << std::setw(twid) << std::right << "STD";
    std::cout << std::setw(twid) << std::right << "vSMC";
    std::cout << std::setw(twid) << std::right << "Batch";
#if VSMC_HAS_MKL
    std::cout << std::setw(twid) << std::right << "MKL";
#endif
    std::cout << std::setw(twid + 3) << std::right << "Deterministics";
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;
    rng_distribution_perf<float>(N, M, nwid, twid);
    rng_distribution_perf<double>(N, M, nwid, twid);
    std::cout << std::string(lwid, '-') << std::endl;
}
