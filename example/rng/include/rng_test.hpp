//============================================================================
// vSMC/example/rng/include/rng_test.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_RNG_TEST_C_API
#define VSMC_RNG_TEST_C_API 0
#endif

#ifndef VSMC_RNG_TEST_MKL
#define VSMC_RNG_TEST_MKL 0
#endif

#include <vsmc/rng/rng.hpp>
#include <vsmc/utility/stop_watch.hpp>
#if VSMC_RNG_TEST_C_API
#include <vsmc/vsmc.h>
#if VSMC_HAS_MKL
#include <vsmc/rng/mkl_brng.hpp>
#endif
#endif

#define VSMC_RNG_TEST_PRE(prog)                                               \
    std::size_t N = 1000000;                                                  \
    std::string prog_name(#prog);                                             \
    if (argc > 1)                                                             \
        N = static_cast<std::size_t>(std::atoi(argv[1]));                     \
    vsmc::Vector<std::string> names;                                          \
    vsmc::Vector<std::size_t> size;                                           \
    vsmc::Vector<vsmc::StopWatch> sw;

#define VSMC_RNG_TEST(RNGType) rng_test<RNGType>(N, #RNGType, names, size, sw);

#define VSMC_RNG_TEST_POST rng_output_sw(prog_name, names, size, sw);

template <typename RNGType>
inline void rng_test(std::size_t N, const std::string &name,
    vsmc::Vector<std::string> &names, vsmc::Vector<std::size_t> &size,
    vsmc::Vector<vsmc::StopWatch> &sw)
{
    names.push_back(name);
    size.push_back(sizeof(RNGType));

    RNGType rng;
    vsmc::StopWatch watch;
    double result = 0;
    vsmc::Vector<double> r(N);
    vsmc::Vector<typename RNGType::result_type> u(N);
    MKL_INT n = static_cast<MKL_INT>(N);
    MKL_INT m = 1000;

#if VSMC_RNG_TEST_C_API && VSMC_HAS_MKL
#if VSMC_RNG_TEST_MKL
    MKL_INT brng = rng.stream().get_stream_state_brng();
#else
    MKL_INT brng = vsmc::mkl_brng<RNGType>();
#endif
    VSLStreamStatePtr stream = nullptr;
    vslNewStream(&stream, brng, 1);
    size.back() += static_cast<std::size_t>(vslGetStreamSize(stream));
#endif

    std::uniform_real_distribution<double> runif_std(0, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        r[i] = runif_std(rng);
    watch.stop();
    sw.push_back(watch);
    result += vsmc::math::asum(N, r.data(), 1);

    vsmc::UniformRealDistributionType<RNGType, double> runif_vsmc(0, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        r[i] = runif_vsmc(rng);
    watch.stop();
    sw.push_back(watch);
    result += vsmc::math::asum(N, r.data(), 1);

    vsmc::uniform_real_distribution(rng, N, r.data(), 0.0, 1.0);
    watch.reset();
    watch.start();
    vsmc::uniform_real_distribution(rng, N, r.data(), 0.0, 1.0);
    watch.stop();
    sw.push_back(watch);
    result += vsmc::math::asum(N, r.data(), 1);

#if VSMC_RNG_TEST_C_API && VSMC_HAS_MKL
    vdRngUniform(VSL_RNG_METHOD_UNIFORM_STD, stream, m, r.data(), 0, 1);
    watch.reset();
    watch.start();
    vdRngUniform(VSL_RNG_METHOD_UNIFORM_STD, stream, n, r.data(), 0, 1);
    watch.stop();
    sw.push_back(watch);
    result += r.back();
#endif

    std::normal_distribution<double> rnorm_std(0, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        r[i] = rnorm_std(rng);
    watch.stop();
    sw.push_back(watch);
    result += vsmc::math::asum(N, r.data(), 1);

    vsmc::NormalDistribution<double> rnorm_vsmc(0, 1);
    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != N; ++i)
        r[i] = rnorm_vsmc(rng);
    watch.stop();
    sw.push_back(watch);
    result += vsmc::math::asum(N, r.data(), 1);

    vsmc::normal_distribution(rng, N, r.data(), 0.0, 1.0);
    watch.reset();
    watch.start();
    vsmc::normal_distribution(rng, N, r.data(), 0.0, 1.0);
    watch.stop();
    sw.push_back(watch);
    result += r.back();

#if VSMC_RNG_TEST_C_API && VSMC_HAS_MKL
    vdRngGaussian(
        VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2, stream, m, r.data(), 0, 1);
    watch.reset();
    watch.start();
    vdRngGaussian(
        VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2, stream, n, r.data(), 0, 1);
    watch.stop();
    sw.push_back(watch);
    result += r.back();
#endif

#if VSMC_RNG_TEST_C_API && VSMC_HAS_MKL
    vslDeleteStream(&stream);
#endif

    std::ofstream rnd("rnd");
    rnd << result << std::endl;
    rnd.close();
}

inline void rng_output_sw(const std::string &prog_name,
    const vsmc::Vector<std::string> &names,
    const vsmc::Vector<std::size_t> &size,
    const vsmc::Vector<vsmc::StopWatch> &sw)
{
    std::size_t N = names.size();
    std::size_t R = sw.size() / N;
    std::size_t lwid = 165;
    int twid = 15;
    int swid = 5;
    int Twid = twid * static_cast<int>(R);
    int nwid = static_cast<int>(lwid) - swid - Twid;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << prog_name;
    std::cout << std::right << std::setw(swid) << "Size";
    switch (R) {
        case 1: // Only time
            std::cout << std::right << std::setw(twid) << "Time (ms)";
            break;
        case 2: // rng_dist
            std::cout << std::right << std::setw(twid) << "C++";
            std::cout << std::right << std::setw(twid) << "C";
            break;
        case 6: // rng_test without c api or mkl
            std::cout << std::right << std::setw(twid) << "U01 (STD)";
            std::cout << std::right << std::setw(twid) << "U01 (vSMC)";
            std::cout << std::right << std::setw(twid) << "U01 (Batch)";
            std::cout << std::right << std::setw(twid) << "Normal (STD)";
            std::cout << std::right << std::setw(twid) << "Normal (vSMC)";
            std::cout << std::right << std::setw(twid) << "Normal (Batch)";
            break;
        case 8: // rng_test with c api and mkl
            std::cout << std::right << std::setw(twid) << "U01 (STD)";
            std::cout << std::right << std::setw(twid) << "U01 (vSMC)";
            std::cout << std::right << std::setw(twid) << "U01 (Batch)";
            std::cout << std::right << std::setw(twid) << "U01 (MKL)";
            std::cout << std::right << std::setw(twid) << "Normal (STD)";
            std::cout << std::right << std::setw(twid) << "Normal (vSMC)";
            std::cout << std::right << std::setw(twid) << "Normal (Batch)";
            std::cout << std::right << std::setw(twid) << "Normal (MKL)";
            break;
    }
    std::cout << std::endl;
    std::cout << std::string(lwid, '-') << std::endl;

    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::left << std::setw(nwid) << names[i];
        std::cout << std::right << std::setw(swid) << size[i];
        for (std::size_t r = 0; r != R; ++r) {
            double time = sw[i * R + r].milliseconds();
            std::cout << std::right << std::setw(twid) << std::fixed << time;
        }
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_TEST_HPP
