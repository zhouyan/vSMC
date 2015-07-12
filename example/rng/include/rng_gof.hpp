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
        N, M, param1, #Name, names, pval);

#define VSMC_RNG_GOF_2(Name, STD, p1, p2)                                     \
    param2[0] = p1;                                                           \
    param2[1] = p2;                                                           \
    rng_gof<STD<double>, vsmc::Name##Distribution<double>>(                   \
        N, M, param2, #Name, names, pval);

template <typename QuantileType>
inline vsmc::Vector<double> rng_gof_partition_quantile(
    std::size_t n, const QuantileType &quantile)
{
    std::size_t k = n / 100;
    double h = 1.0 / k;
    vsmc::Vector<double> partition;
    for (std::size_t i = 0; i != k - 1; ++i) {
        double p = h * (i + 1);
        p = std::max(p, 0.0);
        p = std::min(p, 1.0);
        partition.push_back(quantile(p));
    }

    return partition;
}

template <typename BoostDistType>
inline vsmc::Vector<double> rng_gof_partition_boost(
    std::size_t n, const BoostDistType &dist)
{
    auto quantile = [&](double p) { return boost::math::quantile(dist, p); };

    return rng_gof_partition_quantile(n, quantile);
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::UniformRealDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::uniform_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::NormalDistribution<double> &dist)
{
    return rng_gof_partition_boost(n,
        boost::math::normal_distribution<double>(dist.mean(), dist.stddev()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ExponentialDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::exponential_distribution<double>(dist.lambda()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LaplaceDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::laplace_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LevyDistribution<double> &dist)
{
    boost::math::normal_distribution<double> normal(0, 1);
    auto quantile = [&](double p) {
        double q = boost::math::quantile(normal, 1 - 0.5 * p);

        return dist.a() + dist.b() / (q * q);
    };
    return rng_gof_partition_quantile(n, quantile);
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LogisticDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::logistic_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::WeibullDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::weibull_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::CauchyDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::cauchy_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::RayleighDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::rayleigh_distribution<double>(dist.sigma()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::LognormalDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::lognormal_distribution<double>(dist.m(), dist.s()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ExtremeValueDistribution<double> &dist)
{
    return rng_gof_partition_boost(n,
        boost::math::extreme_value_distribution<double>(dist.a(), dist.b()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::GammaDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::gamma_distribution<double>(dist.alpha(), dist.beta()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::BetaDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::beta_distribution<double>(dist.alpha(), dist.beta()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::ChiSquaredDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::chi_squared_distribution<double>(dist.n()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::StudentTDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::students_t_distribution<double>(dist.n()));
}

inline vsmc::Vector<double> rng_gof_partition(
    std::size_t n, const vsmc::FisherFDistribution<double> &dist)
{
    return rng_gof_partition_boost(
        n, boost::math::fisher_f_distribution<double>(dist.m(), dist.n()));
}

inline double rng_gof_chi2(
    const vsmc::Vector<double> &r, const vsmc::Vector<double> &partition)
{
    vsmc::Vector<std::size_t> count(partition.size() + 1);
    vsmc::Vector<double> rval(r);
    std::sort(rval.begin(), rval.end());
    std::size_t j = 0;
    for (std::size_t i = 0; i != partition.size(); ++i) {
        std::size_t n = 0;
        while (j != rval.size() && rval[j] <= partition[i]) {
            ++n;
            ++j;
        }
        count[i] = n;
    }
    count.back() = rval.size() - j;
    double e = 1.0 / partition.size() * rval.size();
    double p = 0;
    for (std::size_t i = 0; i != partition.size(); ++i)
        p += (count[i] - e) * (count[i] - e) / e;
    boost::math::chi_squared_distribution<double> chi2(partition.size() - 1);

    return boost::math::cdf(chi2, p);
}

inline double rng_gof_ksad(
    const vsmc::Vector<double> &r, const vsmc::Vector<double> &partition)
{
    const std::size_t n = 100;
    const std::size_t m = r.size() / n;
    vsmc::Vector<double> rval(m);
    vsmc::Vector<double> pval(n);
    vsmc::Vector<double> head(n);
    vsmc::Vector<double> tail(n);
    for (std::size_t i = 0; i != n; ++i) {
        std::copy(r.data() + i * m, r.data() + i * m + m, rval.data());
        pval[i] = rng_gof_chi2(rval, partition);
    }
    std::sort(pval.begin(), pval.end());
    vsmc::log(n, pval.data(), head.data());
    std::reverse(pval.begin(), pval.end());
    vsmc::sub(n, 1.0, pval.data(), pval.data());
    vsmc::log(n, pval.data(), tail.data());
    vsmc::add(n, head.data(), tail.data(), pval.data());
    for (std::size_t i = 0; i != n; ++i)
        pval[i] *= 2 * (i + 1) - 1.0;

    return -(n + std::accumulate(pval.begin(), pval.end(), 0.0) / n);
}

inline double rng_gof_pval_mean(const vsmc::Vector<double> &val)
{
    return std::accumulate(val.begin(), val.end(), 0.0) / val.size();
}

inline double rng_gof_pval_stddev(const vsmc::Vector<double> &val, double mean)
{
    return std::sqrt(
        vsmc::dot(val.size(), val.data(), 1, val.data(), 1) / val.size() -
        mean * mean);
}

inline void rng_gof_pval(const vsmc::Vector<double> &chi2,
    const vsmc::Vector<double> &ksad, vsmc::Vector<double> &pval1,
    vsmc::Vector<double> &pval2)
{
    std::size_t alpha1;
    std::size_t alpha5;
    std::size_t alpha10;

    alpha1 = alpha5 = alpha10 = 0;
    for (std::size_t i = 0; i != chi2.size(); ++i) {
        if (chi2[i] > 0.005 && chi2[i] < 1 - 0.005)
            ++alpha1;
        if (chi2[i] > 0.025 && chi2[i] < 1 - 0.025)
            ++alpha5;
        if (chi2[i] > 0.05 && chi2[i] < 1 - 0.05)
            ++alpha10;
    }
    double nchi2 = static_cast<double>(chi2.size());
    pval1.push_back(rng_gof_pval_mean(chi2));
    pval1.push_back(rng_gof_pval_stddev(chi2, pval1.back()));
    pval1.push_back(100.0 * alpha1 / nchi2);
    pval1.push_back(100.0 * alpha5 / nchi2);
    pval1.push_back(100.0 * alpha10 / nchi2);

    alpha1 = alpha5 = alpha10 = 0;
    for (std::size_t i = 0; i != ksad.size(); ++i) {
        if (ksad[i] < 3.857)
            ++alpha1;
        if (ksad[i] < 2.492)
            ++alpha5;
        if (ksad[i] < 1.933)
            ++alpha10;
    }
    double nksad = static_cast<double>(ksad.size());
    pval2.push_back(rng_gof_pval_mean(ksad));
    pval2.push_back(rng_gof_pval_stddev(ksad, pval2.back()));
    pval2.push_back(100.0 * alpha1 / nksad);
    pval2.push_back(100.0 * alpha5 / nksad);
    pval2.push_back(100.0 * alpha10 / nksad);
}

template <typename STDDistType, typename vSMCDistType, std::size_t K>
inline void rng_gof(std::size_t n, std::size_t m,
    const std::array<double, K> &param, const std::string &name,
    vsmc::Vector<std::string> &names, vsmc::Vector<double> &pval)
{
    names.push_back(rng_dist_name(name, param));

    vsmc::RNG rng;
    STDDistType dist_std(rng_dist_init<STDDistType>(param));
    vSMCDistType dist_vsmc(rng_dist_init<vSMCDistType>(param));
    vsmc::Vector<double> r(n);
    vsmc::Vector<double> chi2;
    vsmc::Vector<double> ksad;
    vsmc::Vector<double> pval1;
    vsmc::Vector<double> pval2;
    vsmc::Vector<double> partition(rng_gof_partition(n, dist_vsmc));

    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        for (std::size_t j = 0; j != n; ++j)
            r[j] = dist_std(rng);
        chi2.push_back(rng_gof_chi2(r, partition));
        ksad.push_back(rng_gof_ksad(r, partition));
    }
    rng_gof_pval(chi2, ksad, pval1, pval2);

    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        for (std::size_t j = 0; j != n; ++j)
            r[j] = dist_vsmc(rng);
        chi2.push_back(rng_gof_chi2(r, partition));
        ksad.push_back(rng_gof_ksad(r, partition));
    }
    rng_gof_pval(chi2, ksad, pval1, pval2);

    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        dist_vsmc(rng, n, r.data());
        chi2.push_back(rng_gof_chi2(r, partition));
        ksad.push_back(rng_gof_ksad(r, partition));
    }
    rng_gof_pval(chi2, ksad, pval1, pval2);

#if VSMC_HAS_MKL
    chi2.clear();
    ksad.clear();
    vsmc::MKL_SFMT19937 rng_mkl;
    for (std::size_t i = 0; i != m; ++i) {
        dist_vsmc(rng_mkl, n, r.data());
        chi2.push_back(rng_gof_chi2(r, partition));
        ksad.push_back(rng_gof_ksad(r, partition));
    }
    rng_gof_pval(chi2, ksad, pval1, pval2);
#endif

    for (std::size_t i = 0; i != pval1.size(); ++i)
        pval.push_back(pval1[i]);
    for (std::size_t i = 0; i != pval2.size(); ++i)
        pval.push_back(pval2[i]);
}

inline void rng_gof_output(
    const vsmc::Vector<std::string> &names, const vsmc::Vector<double> &pval)
{
    std::size_t N = names.size();
    std::size_t R = pval.size() / N / 10;
    std::size_t lwid = 80;
    int pwid = 10;
    int twid = 15;
    int nwid = static_cast<int>(lwid) - pwid * 3 - twid * 2;

    vsmc::Vector<std::string> tests;
    tests.push_back("Test 1 (STD)");
    tests.push_back("Test 1 (vSMC)");
    tests.push_back("Test 1 (Batch)");
#if VSMC_HAS_MKL
    tests.push_back("Test 1 (MKL)");
#endif
    tests.push_back("Test 2 (STD)");
    tests.push_back("Test 2 (vSMC)");
    tests.push_back("Test 2 (Batch)");
#if VSMC_HAS_MKL
    tests.push_back("Test 2 (MKL)");
#endif
    std::size_t j = 0;
    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::string(lwid, '=') << std::endl;
        std::cout << std::left << std::setw(nwid) << names[i];
        std::cout << std::right << std::setw(twid) << "Mean";
        std::cout << std::right << std::setw(twid) << "StdDev";
        std::cout << std::right << std::setw(pwid) << "0.01";
        std::cout << std::right << std::setw(pwid) << "0.05";
        std::cout << std::right << std::setw(pwid) << "0.1";
        std::cout << std::endl;
        std::cout << std::string(lwid, '-') << std::endl;
        for (std::size_t r = 0; r != R * 2; ++r) {
            std::cout << std::left << std::setw(nwid) << tests[r];
            std::cout << std::right << std::setw(twid) << std::fixed
                      << pval[j++];
            std::cout << std::right << std::setw(twid) << std::fixed
                      << pval[j++];
            for (std::size_t k = 0; k != 3; ++k) {
                std::stringstream ss;
                double p = pval[j++];
                if (p < 50)
                    ss << '*';
                ss << p << '%';
                std::cout << std::right << std::setw(pwid) << ss.str();
            }
            std::cout << std::endl;
            if (r + 1 == R)
                std::cout << std::string(lwid, '-') << std::endl;
        }
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_GOF_HPP
