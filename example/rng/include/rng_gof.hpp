//============================================================================
// vSMC/example/rng/include/rng_gof.hpp
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

#ifndef VSMC_EXAMPLE_RNG_GOF_HPP
#define VSMC_EXAMPLE_RNG_GOF_HPP

#include "rng_dist.hpp"
#include <boost/math/distributions.hpp>

#define VSMC_RNG_GOF_1(Name, STD, p1)                                         \
    param1[0] = p1;                                                           \
    rng_gof<STD<double>, vsmc::Name##Distribution<double>>(                   \
        N, param1, #Name, names, pval);

#define VSMC_RNG_GOF_2(Name, STD, p1, p2)                                     \
    param2[0] = p1;                                                           \
    param2[1] = p2;                                                           \
    rng_gof<STD<double>, vsmc::Name##Distribution<double>>(                   \
        N, param2, #Name, names, pval);

template <typename BoostDistType>
inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const BoostDistType &dist)
{
    std::size_t k = n / 100;
    double h = 1.0 / k;
    vsmc::Vector<double> partition;
    for (std::size_t i = 0; i != k; ++i) {
        double p = h * (i + 1);
        p = std::max(p, 0.0);
        p = std::min(p, 1.0 - 1e-16);
        partition.push_back(boost::math::quantile(dist, p));
    }

    return partition;
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::UniformRealDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::uniform_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::NormalDistribution<double> &dist)
{
    return rng_gof_partition(n,
        boost::math::normal_distribution<double>(dist.mean(), dist.stddev()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ExponentialDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::exponential_distribution<double>(dist.lambda()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LaplaceDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::laplace_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::WeibullDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::weibull_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::CauchyDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::cauchy_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::RayleighDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::rayleigh_distribution<double>(dist.sigma()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LognormalDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::lognormal_distribution<double>(dist.m(), dist.s()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ExtremeValueDistribution<double> &dist)
{
    return rng_gof_partition(n,
        boost::math::extreme_value_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::GammaDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::gamma_distribution<double>(dist.alpha(), dist.beta()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::BetaDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::beta_distribution<double>(dist.alpha(), dist.beta()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ChiSquaredDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::chi_squared_distribution<double>(dist.n()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::StudentTDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::students_t_distribution<double>(dist.n()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::FisherFDistribution<double> &dist)
{
    return rng_gof_partition(
        n, boost::math::fisher_f_distribution<double>(dist.m(), dist.n()));
}

inline double rng_gof_chi2(
    vsmc::Vector<double> &r, const vsmc::Vector<double> &partition)
{
    vsmc::Vector<std::size_t> count(partition.size());
    std::sort(r.begin(), r.end());
    std::size_t j = 0;
    for (std::size_t i = 0; i != partition.size(); ++i) {
        std::size_t n = 0;
        while (j != r.size() && r[j] <= partition[i]) {
            ++n;
            ++j;
        }
        count[i] = n;
    }
    double e = 1.0 / partition.size() * r.size();
    double p = 0;
    for (std::size_t i = 0; i != partition.size(); ++i)
        p += (count[i] - e) * (count[i] - e) / e;
    boost::math::chi_squared_distribution<double> chi2(partition.size() - 1);

    return boost::math::cdf(chi2, p);
}

template <typename STDDistType, typename vSMCDistType, std::size_t K>
inline void rng_gof(std::size_t n, const std::array<double, K> &param,
    const std::string &name, vsmc::Vector<std::string> &names,
    vsmc::Vector<double> &pval)
{
    names.push_back(rng_dist_name(name, param));

    vsmc::RNG rng;
    STDDistType dist_std(rng_dist_init<STDDistType>(param));
    vSMCDistType dist_vsmc(rng_dist_init<vSMCDistType>(param));
    vsmc::Vector<double> r(n);
    vsmc::Vector<double> partition(rng_gof_partition(n, dist_vsmc));

    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_std(rng);
    pval.push_back(rng_gof_chi2(r, partition));

    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist_vsmc(rng);
    pval.push_back(rng_gof_chi2(r, partition));

    dist_vsmc(rng, n, r.data());
    pval.push_back(rng_gof_chi2(r, partition));

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    dist_vsmc(rng_mkl, n, r.data());
    pval.push_back(rng_gof_chi2(r, partition));
#endif
}

inline void rng_gof_output(
    const vsmc::Vector<std::string> &names, const vsmc::Vector<double> &pval)
{
    std::size_t N = names.size();
    std::size_t R = pval.size() / N;
    std::size_t lwid = 80;
    int twid = 15;
    int Twid = twid * static_cast<int>(R);
    int nwid = static_cast<int>(lwid) - Twid;

    std::cout << std::string(lwid, '=') << std::endl;
    std::cout << std::left << std::setw(nwid) << "Distribution";
    std::cout << std::right << std::setw(twid) << "Test (STD)";
    std::cout << std::right << std::setw(twid) << "Test (vSMC)";
    std::cout << std::right << std::setw(twid) << "Test (Batch)";
#if VSMC_HAS_MKL
    std::cout << std::right << std::setw(twid) << "Test (MKL)";
#endif
    std::cout << std::endl;

    std::cout << std::string(lwid, '-') << std::endl;
    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::left << std::setw(nwid) << names[i];
        for (std::size_t r = 0; r != R; ++r) {
            double value = pval[i * R + r];
            std::stringstream ss;
            if (value < 0.1 || value > 0.9)
                ss << "*";
            if (value < 0.01 || value > 0.99)
                ss << "*";
            if (value < 0.001 || value > 0.999)
                ss << "*";
            ss << std::fixed << value;
            std::cout << std::right << std::setw(twid) << ss.str();
        }
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_GOF_HPP
