//============================================================================
// vSMC/example/rng/include/rng_distribution.hpp
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

#ifndef VSMC_EXAMPLE_RNG_DISTRIBUTION_HPP
#define VSMC_EXAMPLE_RNG_DISTRIBUTION_HPP

#include <boost/math/distributions.hpp>
#include <boost/random.hpp>
#include "rng_common.hpp"

#define VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(K, Name, STD)                      \
    rng_distribution_test<float, K, vsmc::Name##Distribution<float>,          \
        STD<float>>(N, M, #Name, params);                                     \
    rng_distribution_test<double, K, vsmc::Name##Distribution<double>,        \
        STD<double>>(N, M, #Name, params);

#define VSMC_EXAMPLE_RNG_DISTRIBUTION(Name, name)                             \
    if (all ||                                                                \
        std::find(distribution.begin(), distribution.end(),                   \
            std::string(#Name)) != distribution.end()) {                      \
        rng_distribution_##name(N, M);                                        \
    }

template <typename RealType>
inline std::string rng_distribution_name(
    const std::string &name, const std::array<RealType, 0> &param)
{
    std::stringstream ss;
    ss << name << '<' << rng_type_name<RealType>() << '>';

    return ss.str();
}

template <typename RealType>
inline std::string rng_distribution_name(
    const std::string &name, const std::array<RealType, 1> &param)
{
    std::stringstream ss;
    ss << name << '<' << rng_type_name<RealType>() << '>';
    ss << "(" << param[0] << ")";

    return ss.str();
}

template <typename RealType>
inline std::string rng_distribution_name(
    const std::string &name, const std::array<RealType, 2> &param)
{
    std::stringstream ss;
    ss << name << '<' << rng_type_name<RealType>() << '>';
    ss << "(" << param[0] << "," << param[1] << ")";

    return ss.str();
}

template <typename RealType, typename DistType>
inline DistType rng_distribution_init(const std::array<RealType, 0> &)
{
    return DistType();
}

template <typename RealType, typename DistType>
inline DistType rng_distribution_init(const std::array<RealType, 1> &param)
{
    return DistType(param[0]);
}

template <typename RealType, typename DistType>
inline DistType rng_distribution_init(const std::array<RealType, 2> &param)
{
    return DistType(param[0], param[1]);
}

template <typename RealType, typename QuantileType>
inline vsmc::Vector<RealType> rng_distribution_partition_quantile(
    std::size_t n, QuantileType &&quantile)
{
    const std::size_t k = n < 2000 ? n / 100 : 20; // The number of cells
    const RealType p = static_cast<RealType>(1) / k;
    vsmc::Vector<RealType> partition(k - 1);
    for (std::size_t i = 0; i != k - 1; ++i)
        partition[i] = quantile(p * (i + 1));

    return partition;
}

template <typename RealType, typename BoostDistType>
inline vsmc::Vector<RealType> rng_distribution_partition_boost(
    std::size_t n, BoostDistType &&dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return boost::math::quantile(std::forward<BoostDistType>(dist), p);
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::BetaDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(n,
        boost::math::beta_distribution<RealType>(dist.alpha(), dist.beta()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::CauchyDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return dist.a() +
            dist.b() * std::tan(vsmc::const_pi<RealType>() *
                           (p - static_cast<RealType>(0.5)));
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::ChiSquaredDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(
        n, boost::math::chi_squared_distribution<RealType>(dist.n()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::ExponentialDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(
        n, [&](RealType p) { return -std::log(1 - p) / dist.lambda(); });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::ExtremeValueDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return dist.a() - dist.b() * std::log(-std::log(p));
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::FisherFDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(
        n, boost::math::fisher_f_distribution<RealType>(dist.m(), dist.n()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::GammaDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(n,
        boost::math::gamma_distribution<RealType>(dist.alpha(), dist.beta()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::LaplaceDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        RealType q = p - static_cast<RealType>(0.5);
        return q > 0 ? dist.a() - dist.b() * std::log(1 - 2 * q) :
                       dist.a() + dist.b() * std::log(1 + 2 * q);
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::LevyDistribution<RealType> &dist)
{
    boost::math::normal_distribution<RealType> normal(0, 1);
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        RealType q = boost::math::quantile(normal, 1 - p / 2);
        return dist.a() + dist.b() / (q * q);
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::LogisticDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return dist.a() + dist.b() * std::log(p / (1 - p));
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::LognormalDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(
        n, boost::math::lognormal_distribution<RealType>(dist.m(), dist.s()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::NormalDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(
        n, boost::math::normal_distribution<RealType>(
               dist.mean(), dist.stddev()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::ParetoDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return dist.b() / std::exp(std::log(1 - p) / dist.a());
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::RayleighDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return std::sqrt(-2 * std::log(1 - p) * dist.sigma() * dist.sigma());
    });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::StudentTDistribution<RealType> &dist)
{
    return rng_distribution_partition_boost<RealType>(
        n, boost::math::students_t_distribution<RealType>(dist.n()));
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::U01Distribution<RealType> &)
{
    return rng_distribution_partition_quantile<RealType>(
        n, [&](RealType p) { return p; });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::UniformRealDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(
        n, [&](RealType p) { return dist.a() + p * (dist.b() - dist.a()); });
}

template <typename RealType>
inline vsmc::Vector<RealType> rng_distribution_partition(
    std::size_t n, const vsmc::WeibullDistribution<RealType> &dist)
{
    return rng_distribution_partition_quantile<RealType>(n, [&](RealType p) {
        return dist.b() * std::pow(-std::log(1 - p), 1 / dist.a());
    });
}

#if VSMC_HAS_MKL

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::BetaDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.beta(static_cast<MKL_INT>(n), r, dist.alpha(), dist.beta(), 0, 1);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::CauchyDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.cauchy(static_cast<MKL_INT>(n), r, dist.a(), dist.b());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::ChiSquaredDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.gamma(static_cast<MKL_INT>(n), r, dist.n() / 2, 0, 2);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::ExponentialDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.exponential(static_cast<MKL_INT>(n), r, 0, 1 / dist.lambda());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::ExtremeValueDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.gumbel(static_cast<MKL_INT>(n), r, dist.a(), dist.b());
    vsmc::sub(n, 2 * dist.a(), r, r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::FisherFDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    const std::size_t k = vsmc::internal::BufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    vsmc::Array<RealType, k> s;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        stream.gamma(static_cast<MKL_INT>(k), s.data(), dist.m() / 2, 0, 2);
        stream.gamma(static_cast<MKL_INT>(k), r, dist.n() / 2, 0, 2);
        vsmc::mul(k, 1 / dist.m(), s.data(), s.data());
        vsmc::mul(k, 1 / dist.n(), r, r);
        vsmc::div(k, s.data(), r, r);
    }
    stream.gamma(static_cast<MKL_INT>(l), s.data(), dist.m() / 2, 0, 2);
    stream.gamma(static_cast<MKL_INT>(l), r, dist.n() / 2, 0, 2);
    vsmc::mul(l, 1 / dist.m(), s.data(), s.data());
    vsmc::mul(l, 1 / dist.n(), r, r);
    vsmc::div(l, s.data(), r, r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::GammaDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.gamma(static_cast<MKL_INT>(n), r, dist.alpha(), 0, dist.beta());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::LaplaceDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.laplace(static_cast<MKL_INT>(n), r, dist.a(), dist.b());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::LevyDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    const std::size_t k = vsmc::internal::BufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        stream.gaussian(static_cast<MKL_INT>(k), r, 0, 1);
        vsmc::sqr(k, r, r);
        vsmc::inv(k, r, r);
        vsmc::fma(k, r, dist.b(), dist.a(), r);
    }
    stream.gaussian(static_cast<MKL_INT>(l), r, 0, 1);
    vsmc::sqr(l, r, r);
    vsmc::inv(l, r, r);
    vsmc::fma(l, r, dist.b(), dist.a(), r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::LogisticDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    const std::size_t k = vsmc::internal::BufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    vsmc::Array<RealType, k> s;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        stream.uniform(static_cast<MKL_INT>(k), r, 0, 1);
        vsmc::sub(k, static_cast<RealType>(1), r, s.data());
        vsmc::div(k, r, s.data(), r);
        vsmc::log(k, r, r);
        vsmc::fma(k, r, dist.b(), dist.a(), r);
    }
    stream.uniform(static_cast<MKL_INT>(l), r, 0, 1);
    vsmc::sub(l, static_cast<RealType>(1), r, s.data());
    vsmc::div(l, r, s.data(), r);
    vsmc::log(l, r, r);
    vsmc::fma(l, r, dist.b(), dist.a(), r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::LognormalDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.lognormal(static_cast<MKL_INT>(n), r, dist.m(), dist.s(), 0, 1);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::NormalDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.gaussian(static_cast<MKL_INT>(n), r, dist.mean(), dist.stddev());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::ParetoDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    const std::size_t k = vsmc::internal::BufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        stream.exponential(static_cast<MKL_INT>(k), r, 0, 1 / dist.a());
        vsmc::exp(k, r, r);
        vsmc::mul(k, dist.b(), r, r);
    }
    stream.exponential(static_cast<MKL_INT>(l), r, 0, 1 / dist.a());
    vsmc::exp(l, r, r);
    vsmc::mul(l, dist.b(), r, r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::RayleighDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.rayleigh(static_cast<MKL_INT>(n), r, 0,
        vsmc::const_sqrt_2<RealType>() * dist.sigma());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::StudentTDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    const std::size_t k = vsmc::internal::BufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    vsmc::Array<RealType, k> s;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        stream.gamma(static_cast<MKL_INT>(k), r, dist.n() / 2, 0, 2);
        vsmc::mul(k, 1 / dist.n(), r, r);
        vsmc::sqrt(k, r, r);
        stream.gaussian(static_cast<MKL_INT>(k), s.data(), 0, 1);
        vsmc::div(k, s.data(), r, r);
    }
    stream.gamma(static_cast<MKL_INT>(l), r, dist.n() / 2, 0, 2);
    vsmc::mul(l, 1 / dist.n(), r, r);
    vsmc::sqrt(l, r, r);
    stream.gaussian(static_cast<MKL_INT>(l), s.data(), 0, 1);
    vsmc::div(l, s.data(), r, r);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::U01Distribution<RealType> &, std::size_t n, RealType *r)
{
    stream.uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::UniformRealDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.uniform(static_cast<MKL_INT>(n), r, dist.a(), dist.b());
}

template <typename RealType>
inline void rng_distribution_stream(vsmc::MKLStream &stream,
    vsmc::WeibullDistribution<RealType> &dist, std::size_t n, RealType *r)
{
    stream.weibull(static_cast<MKL_INT>(n), r, dist.a(), 0, dist.b());
}

#endif // VSMC_HAS_MKL

template <typename RealType, typename DistType>
inline RealType rng_distribution_chi2(
    std::size_t n, const RealType *r, const DistType &dist)
{
    vsmc::Vector<RealType> rval(r, r + n);
    std::sort(rval.begin(), rval.end());

    vsmc::Vector<RealType> partition(rng_distribution_partition(n, dist));

    const std::size_t k = partition.size() + 1;
    vsmc::Vector<RealType> count(k);
    std::size_t j = 0;
    for (std::size_t i = 0; i != k - 1; ++i) {
        std::size_t c = 0;
        while (j != rval.size() && rval[j] <= partition[i]) {
            ++c;
            ++j;
        }
        count[i] = c;
    }
    count.back() = rval.size() - j;

    const RealType np = n * static_cast<RealType>(1) / k;
    RealType s = 0;
    for (std::size_t i = 0; i != k; ++i)
        s += (count[i] - np) * (count[i] - np) / np;

    boost::math::chi_squared_distribution<RealType> chi2(
        static_cast<RealType>(k - 1));

    return boost::math::cdf(chi2, s);
}

template <typename RealType, typename DistType>
inline RealType rng_distribution_ksad(
    std::size_t n, const RealType *r, const DistType &dist)
{
    const std::size_t k = 10;
    const std::size_t m = n / k;
    vsmc::Vector<RealType> chi2(k);
    vsmc::Vector<RealType> head(k);
    vsmc::Vector<RealType> tail(k);
    for (std::size_t i = 0; i != k; ++i)
        chi2[i] = rng_distribution_chi2(m, r + i * m, dist);
    std::sort(chi2.begin(), chi2.end());
    vsmc::log(k, chi2.data(), head.data());
    std::reverse(chi2.begin(), chi2.end());
    vsmc::sub(k, static_cast<RealType>(1), chi2.data(), chi2.data());
    vsmc::log(k, chi2.data(), tail.data());
    vsmc::add(k, head.data(), tail.data(), chi2.data());
    for (std::size_t i = 0; i != k; ++i)
        chi2[i] *= 2 * i + 1;
    RealType s =
        std::accumulate(chi2.begin(), chi2.end(), static_cast<RealType>(0));

    return -(k + s / k);
}

template <typename RealType>
inline void rng_distribution_pval(const vsmc::Vector<RealType> &chi2,
    const vsmc::Vector<RealType> &ksad,
    std::array<vsmc::Vector<RealType>, 6> &pval)
{
    std::size_t alpha1;
    std::size_t alpha2;
    std::size_t alpha3;

    alpha1 = alpha2 = alpha3 = 0;
    for (std::size_t i = 0; i != chi2.size(); ++i) {
        if (chi2[i] > static_cast<RealType>(0.0125) &&
            chi2[i] < static_cast<RealType>(1 - 0.0125))
            ++alpha1;
        if (chi2[i] > static_cast<RealType>(0.025) &&
            chi2[i] < static_cast<RealType>(1 - 0.025))
            ++alpha2;
        if (chi2[i] > static_cast<RealType>(0.05) &&
            chi2[i] < static_cast<RealType>(1 - 0.05))
            ++alpha3;
    }
    pval[0].push_back(static_cast<RealType>(100.0 * alpha1 / chi2.size()));
    pval[1].push_back(static_cast<RealType>(100.0 * alpha2 / chi2.size()));
    pval[2].push_back(static_cast<RealType>(100.0 * alpha3 / chi2.size()));

    alpha1 = alpha2 = alpha3 = 0;
    for (std::size_t i = 0; i != ksad.size(); ++i) {
        if (ksad[i] < static_cast<RealType>(3.0916))
            ++alpha1;
        if (ksad[i] < static_cast<RealType>(2.4986))
            ++alpha2;
        if (ksad[i] < static_cast<RealType>(1.9355))
            ++alpha3;
    }
    pval[3].push_back(static_cast<RealType>(100.0 * alpha1 / ksad.size()));
    pval[4].push_back(static_cast<RealType>(100.0 * alpha2 / ksad.size()));
    pval[5].push_back(static_cast<RealType>(100.0 * alpha3 / ksad.size()));
}

template <typename RealType, typename vSMCDistType, typename STDDistType,
    std::size_t K>
inline void rng_distribution_test(std::size_t N, std::size_t M,
    const std::array<RealType, K> &param, const std::string &name,
    vsmc::Vector<std::string> &names,
    std::array<vsmc::Vector<RealType>, 6> &pval,
    vsmc::Vector<vsmc::StopWatch> &sw)
{
    names.push_back(rng_distribution_name(name, param));

    vsmc::RNG rng_vsmc;
    vSMCDistType dist_vsmc(
        rng_distribution_init<RealType, vSMCDistType>(param));

    std::mt19937 rng_std;
    STDDistType dist_std(rng_distribution_init<RealType, STDDistType>(param));

    vsmc::Vector<RealType> r(N);
    vsmc::Vector<RealType> chi2(M);
    vsmc::Vector<RealType> ksad(M);
    vsmc::StopWatch watch;

    watch.reset();
    for (std::size_t i = 0; i != M; ++i) {
        watch.start();
        for (std::size_t j = 0; j != N; ++j)
            r[j] = dist_std(rng_std);
        watch.stop();
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    sw.push_back(watch);
    rng_distribution_pval(chi2, ksad, pval);

    watch.reset();
    for (std::size_t i = 0; i != M; ++i) {
        watch.start();
        for (std::size_t j = 0; j != N; ++j)
            r[j] = dist_vsmc(rng_vsmc);
        watch.stop();
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    sw.push_back(watch);
    rng_distribution_pval(chi2, ksad, pval);

    watch.reset();
    for (std::size_t i = 0; i != M; ++i) {
        watch.start();
        vsmc::rng_rand(rng_vsmc, dist_vsmc, N, r.data());
        watch.stop();
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    sw.push_back(watch);
    rng_distribution_pval(chi2, ksad, pval);

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    watch.reset();
    for (std::size_t i = 0; i != M; ++i) {
        watch.start();
        vsmc::rng_rand(rng_mkl, dist_vsmc, N, r.data());
        watch.stop();
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    sw.push_back(watch);
    rng_distribution_pval(chi2, ksad, pval);

    int brng = vsmc::mkl_brng<vsmc::RNG>();
    vsmc::MKLStream stream(brng, 101);
    watch.reset();
    for (std::size_t i = 0; i != M; ++i) {
        watch.start();
        rng_distribution_stream(stream, dist_vsmc, N, r.data());
        watch.stop();
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    sw.push_back(watch);
    rng_distribution_pval(chi2, ksad, pval);
#endif // VSMC_HAS_MKL
}

template <typename RealType>
inline void rng_distribution_summary_pval(RealType pval)
{
    std::stringstream ss;
    if (pval < 50)
        ss << '*';
    ss << pval << '%';
    std::cout << std::right << std::setw(15) << ss.str();
}

template <typename RealType>
inline void rng_distribution_summary(std::size_t N, std::size_t M,
    const vsmc::Vector<std::string> &names,
    const std::array<vsmc::Vector<RealType>, 6> &pval,
    const vsmc::Vector<vsmc::StopWatch> &sw)
{
    std::size_t K = names.size();
    std::size_t R = sw.size() / K;
    int nwid = 30;
    int twid = 15;
    int Twid = static_cast<int>(R) * twid;
    std::size_t lwid = static_cast<std::size_t>(nwid + Twid);

    const vsmc::StopWatch *w = sw.data();
    const RealType *p0 = pval[0].data();
    const RealType *p1 = pval[1].data();
    const RealType *p2 = pval[2].data();
    const RealType *p3 = pval[3].data();
    const RealType *p4 = pval[4].data();
    const RealType *p5 = pval[5].data();
    for (std::size_t i = 0; i != K; ++i) {
        std::cout << std::string(lwid, '=') << std::endl;
        std::cout << std::left << std::setw(nwid) << names[i];
        std::cout << std::right << std::setw(twid) << "STD";
        std::cout << std::right << std::setw(twid) << "vSMC";
        std::cout << std::right << std::setw(twid) << "Batch";
#if VSMC_HAS_MKL
        std::cout << std::right << std::setw(twid) << "MKL";
        std::cout << std::right << std::setw(twid) << "MKL (vSMC)";
#endif
        std::cout << std::endl;
        std::cout << std::string(lwid, '-') << std::endl;

        std::cout << std::left << std::setw(nwid) << "cpE";
        for (std::size_t r = 0; r != R; ++r, ++w) {
            std::cout << std::setw(twid) << std::right << std::fixed
                      << w->cycles() / (N * M);
        }
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "One level test (2.5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p0++);
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "One level test (5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p1++);
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "One level test (10%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p2++);
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "Two level test (2.5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p3++);
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "Two level test (5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p4++);
        std::cout << std::endl;

        std::cout << std::left << std::setw(nwid) << "Two level test (10%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p5++);
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '-') << std::endl;
}

template <typename RealType, std::size_t K, typename vSMCDistType,
    typename STDDistType, typename ParamType>
inline void rng_distribution_test(std::size_t N, std::size_t M,
    const std::string &name, const ParamType &params)
{
    vsmc::Vector<std::string> names;
    std::array<vsmc::Vector<RealType>, 6> pval;
    vsmc::Vector<vsmc::StopWatch> sw;
    for (const auto &p : params) {
        std::array<RealType, K> param;
        for (std::size_t i = 0; i != p.size(); ++i)
            param[i] = static_cast<RealType>(p[i]);
        rng_distribution_test<RealType, vSMCDistType, STDDistType>(
            N, M, param, name, names, pval, sw);
    }
    rng_distribution_summary(N, M, names, pval, sw);
}

inline void rng_distribution_beta(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.5, 0.5}});
    params.push_back({{1.0, 1.0}});
    params.push_back({{1.0, 0.5}});
    params.push_back({{1.0, 1.5}});
    params.push_back({{0.5, 1.0}});
    params.push_back({{1.5, 1.0}});
    params.push_back({{1.5, 1.5}});
    params.push_back({{0.3, 0.3}});
    params.push_back({{0.9, 0.9}});
    params.push_back({{1.5, 0.5}});
    params.push_back({{0.5, 1.5}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, Beta, boost::random::beta_distribution);
}

inline void rng_distribution_cauchy(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Cauchy, std::cauchy_distribution);
}

inline void rng_distribution_chi_squared(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 1>> params;
    params.push_back({{0.2}});
    params.push_back({{1.0}});
    params.push_back({{1.5}});
    params.push_back({{2.0}});
    params.push_back({{3.0}});
    params.push_back({{30.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        1, ChiSquared, std::chi_squared_distribution);
}

inline void rng_distribution_exponential(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 1>> params;
    params.push_back({{1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        1, Exponential, std::exponential_distribution);
}

inline void rng_distribution_extreme_value(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, ExtremeValue, std::extreme_value_distribution);
}

inline void rng_distribution_fisher_f(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.2, 0.2}});
    params.push_back({{0.2, 1.0}});
    params.push_back({{0.2, 1.5}});
    params.push_back({{0.2, 2.0}});
    params.push_back({{0.2, 3.0}});
    params.push_back({{1.0, 0.2}});
    params.push_back({{1.0, 1.0}});
    params.push_back({{1.0, 1.5}});
    params.push_back({{1.0, 2.0}});
    params.push_back({{1.0, 3.0}});
    params.push_back({{2.0, 0.2}});
    params.push_back({{2.0, 1.0}});
    params.push_back({{2.0, 1.5}});
    params.push_back({{2.0, 2.0}});
    params.push_back({{2.0, 3.0}});
    params.push_back({{3.0, 0.2}});
    params.push_back({{3.0, 1.0}});
    params.push_back({{3.0, 1.5}});
    params.push_back({{3.0, 2.0}});
    params.push_back({{3.0, 3.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, FisherF, std::fisher_f_distribution);
}

inline void rng_distribution_gamma(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{1.0, 1.0}});
    params.push_back({{0.1, 1.0}});
    params.push_back({{0.5, 1.0}});
    params.push_back({{0.7, 1.0}});
    params.push_back({{0.9, 1.0}});
    params.push_back({{1.5, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Gamma, std::gamma_distribution);
}

inline void rng_distribution_laplace(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, Laplace, boost::random::laplace_distribution);
}

inline void rng_distribution_levy(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Levy, vsmc::LevyDistribution);
}

inline void rng_distribution_logistic(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, Logistic, vsmc::LogisticDistribution);
}

inline void rng_distribution_lognormal(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, Lognormal, std::lognormal_distribution);
}

inline void rng_distribution_normal(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{0.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Normal, std::normal_distribution);
}

inline void rng_distribution_pareto(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{1.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Pareto, vsmc::ParetoDistribution);
}

inline void rng_distribution_rayleigh(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 1>> params;
    params.push_back({{1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        1, Rayleigh, vsmc::RayleighDistribution);
}

inline void rng_distribution_student_t(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 1>> params;
    params.push_back({{0.2}});
    params.push_back({{1.0}});
    params.push_back({{1.5}});
    params.push_back({{2.0}});
    params.push_back({{3.0}});
    params.push_back({{30.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        1, StudentT, std::student_t_distribution);
}

inline void rng_distribution_u01(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 0>> params(1);
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(0, U01, std::uniform_real_distribution);
}

inline void rng_distribution_uniform_real(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{-0.5, 0.5}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(
        2, UniformReal, std::uniform_real_distribution);
}

inline void rng_distribution_weibull(std::size_t N, std::size_t M)
{
    vsmc::Vector<std::array<double, 2>> params;
    params.push_back({{1.0, 1.0}});
    VSMC_EXAMPLE_RNG_DISTRIBUTION_TEST(2, Weibull, std::weibull_distribution);
}

inline void rng_distribution_help()
{
    vsmc::Vector<std::string> distribution;
    distribution.push_back("All");
    distribution.push_back("Beta");
    distribution.push_back("Cauchy");
    distribution.push_back("ChiSquared");
    distribution.push_back("Exponential");
    distribution.push_back("ExtremeValue");
    distribution.push_back("FisherF");
    distribution.push_back("Gamma");
    distribution.push_back("Laplace");
    distribution.push_back("Levy");
    distribution.push_back("Logistic");
    distribution.push_back("Lognormal");
    distribution.push_back("Normal");
    distribution.push_back("Pareto");
    distribution.push_back("Rayleigh");
    distribution.push_back("StudentT");
    distribution.push_back("U01");
    distribution.push_back("UniformReal");
    distribution.push_back("Weibull");
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "Possible values for --distribution" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    for (std::size_t i = 0; i != distribution.size(); ++i) {
        std::cout << std::setw(20) << std::left << distribution[i];
        if ((i + 1) % 4 == 0 || i + 1 == distribution.size())
            std::cout << std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
}

inline void rng_distribution(std::size_t N, std::size_t M,
    const vsmc::Vector<std::string> &distribution)
{
    N = std::max(N, static_cast<std::size_t>(10000));
    M = std::max(M, static_cast<std::size_t>(10));
    bool all = std::find(distribution.begin(), distribution.end(),
                   std::string("All")) != distribution.end();

    VSMC_EXAMPLE_RNG_DISTRIBUTION(Beta, beta);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Cauchy, cauchy);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(ChiSquared, chi_squared);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Exponential, exponential);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(ExtremeValue, extreme_value);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(FisherF, fisher_f);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Gamma, gamma);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Laplace, laplace);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Levy, levy);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Logistic, logistic);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Lognormal, lognormal);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Normal, normal);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Pareto, pareto);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Rayleigh, rayleigh);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(StudentT, student_t);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(U01, u01);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(UniformReal, uniform_real);
    VSMC_EXAMPLE_RNG_DISTRIBUTION(Weibull, weibull);
}

#endif // VSMC_EXAMPLE_RNG_DISTRIBUTION_HPP
