//============================================================================
// vSMC/example/rng/include/rng_dist.hpp
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

#ifndef VSMC_EXAMPLE_RNG_DIST_HPP
#define VSMC_EXAMPLE_RNG_DIST_HPP

#include <boost/math/distributions/chi_squared.hpp>
#include <vsmc/rng/engine.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_RNG_DIST_1(Name, STD, p1)                                        \
    param[0] = p1;                                                            \
    rng_dist<STD<double>, vsmc::Name##Distribution<double>>(                  \
        N, M, param, #Name, names, mean, variance, pval1, pval2, sw);

#define VSMC_RNG_DIST_2(Name, STD, p1, p2)                                    \
    param[0] = p1;                                                            \
    param[1] = p2;                                                            \
    rng_dist<STD<double>, vsmc::Name##Distribution<double>>(                  \
        N, M, param, #Name, names, mean, variance, pval1, pval2, sw);

#define VSMC_RNG_DIST_ALL(TEST)                                               \
    VSMC_RNG_DIST_2(Levy, vsmc::LevyDistribution, 0, 1);                      \
    VSMC_RNG_DIST_2(Pareto, vsmc::ParetoDistribution, 1, 1);

#define VSMC_RNG_DIST_PRE(p)                                                  \
    std::size_t N = 10000;                                                    \
    if (argc > 1)                                                             \
        N = static_cast<std::size_t>(std::atoi(argv[1]));                     \
    std::size_t M = 100;                                                      \
    if (argc > 2)                                                             \
        M = static_cast<std::size_t>(std::atoi(argv[2]));                     \
    std::array<double, p> param;                                              \
    vsmc::Vector<std::string> names;                                          \
    vsmc::Vector<double> mean;                                                \
    vsmc::Vector<double> variance;                                            \
    vsmc::Vector<double> pval1;                                               \
    vsmc::Vector<double> pval2;                                               \
    vsmc::Vector<vsmc::StopWatch> sw;

#define VSMC_RNG_DIST_POST                                                    \
    rng_dist_output(names, mean, variance, pval1, pval2, sw);

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

template <typename QuantileType>
inline vsmc::Vector<double> rng_dist_partition_quantile(
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
inline vsmc::Vector<double> rng_dist_partition_boost(
    std::size_t n, const BoostDistType &dist)
{
    auto quantile = [&](double p) { return boost::math::quantile(dist, p); };

    return rng_dist_partition_quantile(n, quantile);
}

template <typename DistType>
inline vsmc::Vector<double> rng_dist_partition(std::size_t, DistType &);

template <typename Left, typename Right>
inline vsmc::Vector<double> rng_dist_partition(
    std::size_t n, vsmc::UniformRealLRDistribution<double, Left, Right> &dist)
{
    return rng_dist_partition_quantile(
        n, [&](double p) { return dist.a() + p * (dist.b() - dist.a()); });
}

inline double rng_dist_chi2(
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
    boost::math::chi_squared_distribution<double> chi2(
        static_cast<double>(partition.size() - 1));

    return boost::math::cdf(chi2, p);
}

inline double rng_dist_ksad(
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
        pval[i] = rng_dist_chi2(rval, partition);
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

inline void rng_dist_moments(const vsmc::Vector<double> &r,
    vsmc::Vector<double> &mean, vsmc::Vector<double> &variance)
{
    mean.push_back(std::accumulate(r.begin(), r.end(), 0.0) / r.size());
    variance.push_back(cblas_ddot(static_cast<VSMC_CBLAS_INT>(r.size()),
                           r.data(), 1, r.data(), 1) /
            r.size() -
        mean.back() * mean.back());
}

inline void rng_dist_pval(const vsmc::Vector<double> &chi2,
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
    pval2.push_back(100.0 * alpha1 / nksad);
    pval2.push_back(100.0 * alpha5 / nksad);
    pval2.push_back(100.0 * alpha10 / nksad);
}

template <typename STDDistType, typename vSMCDistType, std::size_t K>
inline void rng_dist(std::size_t n, std::size_t m,
    const std::array<double, K> &param, const std::string &name,
    vsmc::Vector<std::string> &names, vsmc::Vector<double> &mean,
    vsmc::Vector<double> &variance, vsmc::Vector<double> &pval1,
    vsmc::Vector<double> &pval2, vsmc::Vector<vsmc::StopWatch> &sw)
{
    names.push_back(rng_dist_name(name, param));

    vsmc::RNG rng;
    STDDistType dist_std(rng_dist_init<STDDistType>(param));
    vSMCDistType dist_vsmc(rng_dist_init<vSMCDistType>(param));
    vsmc::Vector<double> r(n);
    vsmc::Vector<double> chi2;
    vsmc::Vector<double> ksad;
    vsmc::Vector<double> partition(rng_dist_partition(n, dist_vsmc));
    vsmc::StopWatch watch;

    watch.reset();
    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        watch.start();
        for (std::size_t j = 0; j != n; ++j)
            r[j] = dist_std(rng);
        watch.stop();
        chi2.push_back(rng_dist_chi2(r, partition));
        ksad.push_back(rng_dist_ksad(r, partition));
    }
    sw.push_back(watch);
    rng_dist_moments(r, mean, variance);
    rng_dist_pval(chi2, ksad, pval1, pval2);

    watch.reset();
    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        watch.start();
        for (std::size_t j = 0; j != n; ++j)
            r[j] = dist_vsmc(rng);
        watch.stop();
        chi2.push_back(rng_dist_chi2(r, partition));
        ksad.push_back(rng_dist_ksad(r, partition));
    }
    sw.push_back(watch);
    rng_dist_moments(r, mean, variance);
    rng_dist_pval(chi2, ksad, pval1, pval2);

    watch.reset();
    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        watch.start();
        vsmc::rng_rand(rng, dist_vsmc, n, r.data());
        watch.stop();
        chi2.push_back(rng_dist_chi2(r, partition));
        ksad.push_back(rng_dist_ksad(r, partition));
    }
    sw.push_back(watch);
    rng_dist_moments(r, mean, variance);
    rng_dist_pval(chi2, ksad, pval1, pval2);

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    watch.reset();
    chi2.clear();
    ksad.clear();
    for (std::size_t i = 0; i != m; ++i) {
        watch.start();
        vsmc::rng_rand(rng_mkl, dist_vsmc, n, r.data());
        watch.stop();
        chi2.push_back(rng_dist_chi2(r, partition));
        ksad.push_back(rng_dist_ksad(r, partition));
    }
    sw.push_back(watch);
    rng_dist_moments(r, mean, variance);
    rng_dist_pval(chi2, ksad, pval1, pval2);
#endif
}

inline void rng_dist_output(const vsmc::Vector<std::string> &names,
    const vsmc::Vector<double> &mean, const vsmc::Vector<double> &variance,
    const vsmc::Vector<double> &pval1, const vsmc::Vector<double> &pval2,
    const vsmc::Vector<vsmc::StopWatch> &sw)
{
    std::size_t N = names.size();
    std::size_t R = sw.size() / N;
    std::size_t lwid = 80;
    int twid = 15;
    int Twid = twid * static_cast<int>(R);
    int nwid = static_cast<int>(lwid) - Twid;

    for (std::size_t i = 0; i != N; ++i) {
        std::cout << std::string(lwid, '=') << std::endl;
        std::cout << std::left << std::setw(nwid) << names[i];
        std::cout << std::right << std::setw(twid) << "STD";
        std::cout << std::right << std::setw(twid) << "vSMC";
        std::cout << std::right << std::setw(twid) << "Batch";
#if VSMC_HAS_MKL
        std::cout << std::right << std::setw(twid) << "MKL";
#endif
        std::cout << std::endl;
        std::cout << std::string(lwid, '-') << std::endl;
        std::cout << std::left << std::setw(nwid) << "Time";
        for (std::size_t r = 0; r != R; ++r) {
            double time = sw[i * R + r].milliseconds();
            std::cout << std::right << std::setw(twid) << time;
        }
        std::cout << std::endl;
        std::cout << std::left << std::setw(nwid) << "Mean";
        for (std::size_t r = 0; r != R; ++r) {
            double m = mean[i * R + r];
            std::cout << std::right << std::setw(twid) << m;
        }
        std::cout << std::endl;
        std::cout << std::left << std::setw(nwid) << "Variance";
        for (std::size_t r = 0; r != R; ++r) {
            double v = variance[i * R + r];
            std::cout << std::right << std::setw(twid) << v;
        }
        std::cout << std::endl;
        std::cout << std::left << std::setw(nwid) << "Single level test";
        for (std::size_t r = 0; r != R; ++r) {
            double p = pval1[i * R + r];
            std::stringstream ss;
            if (p < 50)
                ss << '*';
            ss << p << '%';
            std::cout << std::right << std::setw(twid) << ss.str();
        }
        std::cout << std::endl;
        std::cout << std::left << std::setw(nwid) << "Two level Test";
        for (std::size_t r = 0; r != R; ++r) {
            double p = pval2[i * R + r];
            std::stringstream ss;
            if (p < 50)
                ss << '*';
            ss << p << '%';
            std::cout << std::right << std::setw(twid) << ss.str();
        }
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '=') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_DIST_HPP
