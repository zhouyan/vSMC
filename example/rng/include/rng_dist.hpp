//============================================================================
// vSMC/example/rng/include/rng_dist.hpp
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

#ifndef VSMC_EXAMPLE_RNG_DIST_HPP
#define VSMC_EXAMPLE_RNG_DIST_HPP

#include <vsmc/rng/rng.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <boost/random/laplace_distribution.hpp>
#include <boost/random/beta_distribution.hpp>

#define VSMC_RNG_DIST_1(Name, STD, p1)                                        \
    param1[0] = p1;                                                           \
    rng_dist<STD<double>, vsmc::Name##Distribution<double>>(                  \
        N, param1, #Name, names, sw);

#define VSMC_RNG_DIST_2(Name, STD, p1, p2)                                    \
    param2[0] = p1;                                                           \
    param2[1] = p2;                                                           \
    rng_dist<STD<double>, vsmc::Name##Distribution<double>>(                  \
        N, param2, #Name, names, sw);

#define VSMC_RNG_DIST_ALL(TEST)                                               \
    VSMC_RNG_##TEST##_1(Exponential, std::exponential_distribution, 1);       \
    VSMC_RNG_##TEST##_1(Rayleigh, vsmc::RayleighDistribution, 1);             \
    VSMC_RNG_##TEST##_2(Cauchy, std::cauchy_distribution, 0, 1);              \
    VSMC_RNG_##TEST##_2(ExtremeValue, std::extreme_value_distribution, 0, 1); \
    VSMC_RNG_##TEST##_2(Laplace, boost::random::laplace_distribution, 0, 1);  \
    VSMC_RNG_##TEST##_2(Levy, vsmc::LevyDistribution, 0, 1);                  \
    VSMC_RNG_##TEST##_2(Logistic, vsmc::LogisticDistribution, 0, 1);          \
    VSMC_RNG_##TEST##_2(Lognormal, std::lognormal_distribution, 0, 1);        \
    VSMC_RNG_##TEST##_2(Normal, std::normal_distribution, 0, 1);              \
    VSMC_RNG_##TEST##_2(UniformReal, std::uniform_real_distribution, 0, 1);   \
    VSMC_RNG_##TEST##_2(Weibull, std::weibull_distribution, 1, 1);            \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 1, 1);                \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 0.1, 1);              \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 0.5, 1);              \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 0.7, 1);              \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 0.9, 1);              \
    VSMC_RNG_##TEST##_2(Gamma, std::gamma_distribution, 1.5, 1);              \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1, 1);        \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1, 0.5);      \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1, 1.5);      \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 0.5, 1);      \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1.5, 1);      \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1.5, 1.5);    \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 0.5, 0.5);    \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 0.9, 0.9);    \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 1.5, 0.5);    \
    VSMC_RNG_##TEST##_2(Beta, boost::random::beta_distribution, 0.5, 1.5);    \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 0.2);      \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 1);        \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 1.5);      \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 2);        \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 3);        \
    VSMC_RNG_##TEST##_1(ChiSquared, std::chi_squared_distribution, 30);       \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 0.2);          \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 1);            \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 1.5);          \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 2);            \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 3);            \
    VSMC_RNG_##TEST##_1(StudentT, std::student_t_distribution, 30);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 0.2, 0.2);       \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 0.2, 1);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 0.2, 1.5);       \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 0.2, 2);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 0.2, 3);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 1, 0.2);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 1, 1);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 1, 1.5);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 1, 2);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 1, 3);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 2, 0.2);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 2, 1);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 2, 1.5);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 2, 2);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 2, 3);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 3, 0.2);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 3, 1);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 3, 1.5);         \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 3, 2);           \
    VSMC_RNG_##TEST##_2(FisherF, std::fisher_f_distribution, 3, 3);

template <std::size_t K>
inline std::string rng_dist_name(
    const std::string &name, const std::array<double, K> &param)
{
    std::stringstream ss;
    ss << name;
    if (K > 0)
        ss << "(" << param[0];
    for (std::size_t k = 1; k < K; ++k)
        ss << "," << param[k];
    if (K > 0)
        ss << ")";

    return ss.str();
}

template <typename DistType>
inline DistType rng_dist_init(const std::array<double, 1> &param)
{
    return DistType(param[0]);
}

template <typename DistType>
inline DistType rng_dist_init(const std::array<double, 2> &param)
{
    return DistType(param[0], param[1]);
}

template <typename STDDistType, typename vSMCDistType, std::size_t K>
inline void rng_dist(std::size_t n, const std::array<double, K> &param,
    const std::string &name, vsmc::Vector<std::string> &names,
    vsmc::Vector<vsmc::StopWatch> &sw)
{
    names.push_back(rng_dist_name(name, param));

    vsmc::RNG rng;
    STDDistType dist_std(rng_dist_init<STDDistType>(param));
    vSMCDistType dist_vsmc(rng_dist_init<vSMCDistType>(param));
    vsmc::Vector<double> r(n);
    double result = 0;
    vsmc::StopWatch watch;

    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_std(rng);
    watch.stop();
    result += std::accumulate(r.begin(), r.end(), 0.0);
    sw.push_back(watch);

    watch.reset();
    watch.start();
    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_vsmc(rng);
    watch.stop();
    result += std::accumulate(r.begin(), r.end(), 0.0);
    sw.push_back(watch);

    dist_vsmc(rng, 1000, r.data());
    watch.reset();
    watch.start();
    dist_vsmc(rng, n, r.data());
    watch.stop();
    result += std::accumulate(r.begin(), r.end(), 0.0);
    sw.push_back(watch);

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    dist_vsmc(rng_mkl, 1000, r.data());
    watch.reset();
    watch.start();
    dist_vsmc(rng_mkl, n, r.data());
    watch.stop();
    result += std::accumulate(r.begin(), r.end(), 0.0);
    sw.push_back(watch);
#endif

    std::ofstream rnd("rnd");
    rnd << result << std::endl;
    rnd.close();
}

inline void rng_dist_output(const vsmc::Vector<std::string> &names,
    const vsmc::Vector<vsmc::StopWatch> &sw)
{
    std::size_t N = names.size();
    std::size_t R = sw.size() / N;
    std::size_t lwid = 80;
    int twid = 15;
    int Twid = twid * static_cast<int>(R);
    int nwid = static_cast<int>(lwid) - Twid;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "Distribution";
    std::cout << std::right << std::setw(twid) << "Time (STD)";
    std::cout << std::right << std::setw(twid) << "Time (vSMC)";
    std::cout << std::right << std::setw(twid) << "Time (Batch)";
#if VSMC_HAS_MKL
    std::cout << std::right << std::setw(twid) << "Time (MKL)";
#endif
    std::cout << std::endl;

    std::cout << std::string(lwid, '-') << std::endl;
    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::left << std::setw(nwid) << names[i];
        for (std::size_t r = 0; r != R; ++r) {
            double time = sw[i * R + r].milliseconds();
            std::cout << std::right << std::setw(twid) << std::fixed << time;
        }
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_DIST_HPP
