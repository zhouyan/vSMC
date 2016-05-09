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

#define VSMC_DEFINE_EXAMPLE_RNG_DISTRIBUTION_TEST(test)                       \
    rng_distribution_test_##test<vsmc::ArcsineDistribution<RealType>>(        \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::BetaDistribution<RealType>>(           \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::CauchyDistribution<RealType>>(         \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::ChiSquaredDistribution<RealType>>(     \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::ExponentialDistribution<RealType>>(    \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::ExtremeValueDistribution<RealType>>(   \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::FisherFDistribution<RealType>>(        \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::GammaDistribution<RealType>>(          \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::LaplaceDistribution<RealType>>(        \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::LevyDistribution<RealType>>(           \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::LogisticDistribution<RealType>>(       \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::LognormalDistribution<RealType>>(      \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::NormalDistribution<RealType>>(         \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::ParetoDistribution<RealType>>(         \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::RayleighDistribution<RealType>>(       \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::StudentTDistribution<RealType>>(       \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::U01Distribution<RealType>>(            \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::UniformRealDistribution<RealType>>(    \
        N, M, nwid, twid);                                                    \
    rng_distribution_test_##test<vsmc::WeibullDistribution<RealType>>(        \
        N, M, nwid, twid);

template <typename RNGType>
class RNG01 : public RNGType
{
    public:
    using result_type = typename RNGType::result_type;

    result_type operator()()
    {
        result_type u = RNGType::operator()();
        result_type r = u % 3;
        if (r == 0)
            return 0;
        if (r == 1)
            return std::numeric_limits<result_type>::max();
        return u;
    }

    void operator()(std::size_t n, result_type *r)
    {
        for (std::size_t i = 0; i != n; ++i)
            r[i] = operator()();
    }
}; // class RNG01

template <std::size_t ParamNum>
class DistTraitBase
{
    public:
    DistTraitBase() = default;
    DistTraitBase(const DistTraitBase<ParamNum> &) = default;
    DistTraitBase<ParamNum> &operator=(
        const DistTraitBase<ParamNum> &) = default;
    virtual ~DistTraitBase() {}

    static const std::size_t param_num = ParamNum;

    virtual std::string dist_name() const = 0;

    template <typename RealType>
    std::string name(const std::array<RealType, ParamNum> &param) const
    {
        return name_dispatch(dist_name(), param);
    }

    protected:
    template <typename RealType, typename QuantileType>
    vsmc::Vector<RealType> partition_quantile(
        std::size_t n, QuantileType &&quantile) const
    {
        const std::size_t k = n < 2000 ? n / 100 : 20; // The number of cells
        const RealType p = static_cast<RealType>(1) / k;
        vsmc::Vector<RealType> partition(k - 1);
        for (std::size_t i = 0; i != k - 1; ++i)
            partition[i] = quantile(p * (i + 1));

        return partition;
    }

    template <typename RealType, typename BoostDistType>
    vsmc::Vector<RealType> partition_boost(
        std::size_t n, BoostDistType &&dist) const
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return boost::math::quantile(std::forward<BoostDistType>(dist), p);
        });
    }

    template <typename RealType>
    static void add_param(vsmc::Vector<std::array<RealType, 0>> &params)
    {
        std::array<RealType, 0> tmp;
        params.push_back(tmp);
    }

    template <typename RealType, typename ValueType1>
    static void add_param(
        vsmc::Vector<std::array<RealType, 1>> &param, ValueType1 p1)
    {
        std::array<RealType, 1> tmp;
        tmp[0] = static_cast<RealType>(p1);
        param.push_back(tmp);
    }

    template <typename RealType, typename ValueType1, typename ValueType2>
    static void add_param(vsmc::Vector<std::array<RealType, 2>> &param,
        ValueType1 p1, ValueType2 p2)
    {
        std::array<RealType, 2> tmp;
        tmp[0] = static_cast<RealType>(p1);
        tmp[1] = static_cast<RealType>(p2);
        param.push_back(tmp);
    }

    private:
    template <typename RealType>
    static std::string name_dispatch(
        const std::string &dist_name, const std::array<RealType, 0> &)
    {
        std::stringstream ss;
        ss << dist_name << "<" << rng_type_name<RealType>() << ">";

        return ss.str();
    }

    template <typename RealType>
    static std::string name_dispatch(
        const std::string &dist_name, const std::array<RealType, 1> &param)
    {
        std::stringstream ss;
        ss << dist_name << "<" << rng_type_name<RealType>() << ">(" << param[0]
           << ")";

        return ss.str();
    }

    template <typename RealType>
    static std::string name_dispatch(
        const std::string &dist_name, const std::array<RealType, 2> &param)
    {
        std::stringstream ss;
        ss << dist_name << "<" << rng_type_name<RealType>() << ">(" << param[0]
           << "," << param[1] << ")";

        return ss.str();
    }
};

template <typename DistType>
class DistTrait;

template <typename RealType>
class DistTrait<vsmc::ArcsineDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::ArcsineDistribution<RealType>;
    using std_type = dist_type;

    std::string dist_name() const { return "Arcsine"; }

    vsmc::Vector<RealType> partition(
        std::size_t n, const dist_type &dist) const
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            RealType r = std::sin(vsmc::const_pi_by2<RealType>() * p);
            return dist.a() + (dist.b() - dist.a()) * (r * r);
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::BetaDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::BetaDistribution<RealType>;
    using std_type = boost::random::beta_distribution<RealType>;

    std::string dist_name() const { return "Beta"; }

    vsmc::Vector<RealType> partition(
        std::size_t n, const dist_type &dist) const
    {
        return partition_boost<RealType>(
            n, boost::math::beta_distribution<RealType>(
                   dist.alpha(), dist.beta()));
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0.5, 0.5);
        add_param(params, 1, 1);
        add_param(params, 1, 0.5);
        add_param(params, 1, 1.5);
        add_param(params, 0.5, 1);
        add_param(params, 1.5, 1);
        add_param(params, 1.5, 1.5);
        add_param(params, 0.3, 0.3);
        add_param(params, 0.9, 0.9);
        add_param(params, 1.5, 0.5);
        add_param(params, 0.5, 1.5);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::CauchyDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::CauchyDistribution<RealType>;
    using std_type = std::cauchy_distribution<RealType>;

    std::string dist_name() const { return "Cauchy"; }

    vsmc::Vector<RealType> partition(
        std::size_t n, const dist_type &dist) const
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return dist.a() +
                dist.b() * std::tan(vsmc::const_pi<RealType>() *
                               (p - static_cast<RealType>(0.5)));
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::GammaDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::GammaDistribution<RealType>;
    using std_type = std::gamma_distribution<RealType>;

    std::string dist_name() const { return "Gamma"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(
            n, boost::math::gamma_distribution<RealType>(
                   dist.alpha(), dist.beta()));
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 1, 1);
        add_param(params, 0.1, 1);
        add_param(params, 0.5, 1);
        add_param(params, 0.7, 1);
        add_param(params, 0.9, 1);
        add_param(params, 1.5, 1);
        add_param(params, 15, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::ChiSquaredDistribution<RealType>>
    : public DistTraitBase<1>
{
    public:
    using dist_type = vsmc::ChiSquaredDistribution<RealType>;
    using std_type = std::chi_squared_distribution<RealType>;

    std::string dist_name() const { return "ChiSquared"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(
            n, boost::math::chi_squared_distribution<RealType>(dist.n()));
    }

    vsmc::Vector<std::array<RealType, 1>> params() const
    {
        DistTrait<vsmc::GammaDistribution<RealType>> gamma_traits;
        vsmc::Vector<std::array<RealType, 2>> pgamma = gamma_traits.params();
        vsmc::Vector<std::array<RealType, 1>> params;
        for (std::size_t i = 0; i != pgamma.size(); ++i)
            add_param(params, pgamma[i][0] * 2);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::ExponentialDistribution<RealType>>
    : public DistTraitBase<1>
{
    public:
    using dist_type = vsmc::ExponentialDistribution<RealType>;
    using std_type = std::exponential_distribution<RealType>;

    std::string dist_name() const { return "Exponential"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(
            n, [&](RealType p) { return -std::log(1 - p) / dist.lambda(); });
    }

    vsmc::Vector<std::array<RealType, 1>> params() const
    {
        vsmc::Vector<std::array<RealType, 1>> params;
        add_param(params, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::ExtremeValueDistribution<RealType>>
    : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::ExtremeValueDistribution<RealType>;
    using std_type = std::extreme_value_distribution<RealType>;

    std::string dist_name() const { return "ExtremeValue"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return dist.a() - dist.b() * std::log(-std::log(p));
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::FisherFDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::FisherFDistribution<RealType>;
    using std_type = std::fisher_f_distribution<RealType>;

    std::string dist_name() const { return "FisherF"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(n,
            boost::math::fisher_f_distribution<RealType>(dist.m(), dist.n()));
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        std::array<double, 5> df = {{1, 0.5, 1.5, 3, 30}};
        vsmc::Vector<std::array<RealType, 2>> params;
        for (std::size_t i = 0; i != df.size(); ++i)
            for (std::size_t j = 0; j != df.size(); ++j)
                add_param(params, df[i], df[j]);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::LaplaceDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::LaplaceDistribution<RealType>;
    using std_type = boost::random::laplace_distribution<RealType>;

    std::string dist_name() const { return "Laplace"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            RealType q = p - static_cast<RealType>(0.5);
            return q > 0 ? dist.a() - dist.b() * std::log(1 - 2 * q) :
                           dist.a() + dist.b() * std::log(1 + 2 * q);
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::LevyDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::LevyDistribution<RealType>;
    using std_type = dist_type;

    std::string dist_name() const { return "Levy"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        boost::math::normal_distribution<RealType> normal(0, 1);
        return partition_quantile<RealType>(n, [&](RealType p) {
            RealType q = boost::math::quantile(normal, 1 - p / 2);
            return dist.a() + dist.b() / (q * q);
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::LogisticDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::LogisticDistribution<RealType>;
    using std_type = dist_type;

    std::string dist_name() const { return "Logistic"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return dist.a() + dist.b() * std::log(p / (1 - p));
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::LognormalDistribution<RealType>>
    : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::LognormalDistribution<RealType>;
    using std_type = std::lognormal_distribution<RealType>;

    std::string dist_name() const { return "Lognormal"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(n,
            boost::math::lognormal_distribution<RealType>(dist.m(), dist.s()));
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::NormalDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::NormalDistribution<RealType>;
    using std_type = std::normal_distribution<RealType>;

    std::string dist_name() const { return "Normal"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(
            n, boost::math::normal_distribution<RealType>(
                   dist.mean(), dist.stddev()));
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 0, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::ParetoDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::ParetoDistribution<RealType>;
    using std_type = dist_type;

    std::string dist_name() const { return "Pareto"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return dist.b() / std::exp(std::log(1 - p) / dist.a());
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 1, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::RayleighDistribution<RealType>> : public DistTraitBase<1>
{
    public:
    using dist_type = vsmc::RayleighDistribution<RealType>;
    using std_type = dist_type;

    std::string dist_name() const { return "Rayleigh"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return std::sqrt(
                -2 * std::log(1 - p) * dist.sigma() * dist.sigma());
        });
    }

    vsmc::Vector<std::array<RealType, 1>> params() const
    {
        vsmc::Vector<std::array<RealType, 1>> params;
        add_param(params, 1);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::StudentTDistribution<RealType>> : public DistTraitBase<1>
{
    public:
    using dist_type = vsmc::StudentTDistribution<RealType>;
    using std_type = std::student_t_distribution<RealType>;

    std::string dist_name() const { return "StudentT"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_boost<RealType>(
            n, boost::math::students_t_distribution<RealType>(dist.n()));
    }

    vsmc::Vector<std::array<RealType, 1>> params() const
    {
        DistTrait<vsmc::ChiSquaredDistribution<RealType>> chi_traits;
        return chi_traits.params();
    }
};

template <typename RealType>
class DistTrait<vsmc::U01Distribution<RealType>> : public DistTraitBase<0>
{
    public:
    using dist_type = vsmc::U01Distribution<RealType>;
    using std_type = std::uniform_real_distribution<RealType>;

    std::string dist_name() const { return "U01"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &)
    {
        return partition_quantile<RealType>(n, [&](RealType p) { return p; });
    }

    vsmc::Vector<std::array<RealType, 0>> params() const
    {
        return vsmc::Vector<std::array<RealType, 0>>(1);
    }
};

template <typename RealType>
class DistTrait<vsmc::UniformRealDistribution<RealType>>
    : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::UniformRealDistribution<RealType>;
    using std_type = std::uniform_real_distribution<RealType>;

    std::string dist_name() const { return "UniformReal"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n,
            [&](RealType p) { return dist.a() + p * (dist.b() - dist.a()); });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, -0.5, 0.5);

        return params;
    }
};

template <typename RealType>
class DistTrait<vsmc::WeibullDistribution<RealType>> : public DistTraitBase<2>
{
    public:
    using dist_type = vsmc::WeibullDistribution<RealType>;
    using std_type = std::weibull_distribution<RealType>;

    std::string dist_name() const { return "Weibull"; }

    vsmc::Vector<RealType> partition(std::size_t n, const dist_type &dist)
    {
        return partition_quantile<RealType>(n, [&](RealType p) {
            return dist.b() * std::pow(-std::log(1 - p), 1 / dist.a());
        });
    }

    vsmc::Vector<std::array<RealType, 2>> params() const
    {
        vsmc::Vector<std::array<RealType, 2>> params;
        add_param(params, 1, 1);

        return params;
    }
};

template <typename DistType, typename RealType>
inline DistType rng_distribution_init(const std::array<RealType, 0> &)
{
    return DistType();
}

template <typename DistType, typename RealType>
inline DistType rng_distribution_init(const std::array<RealType, 1> &param)
{
    return DistType(param[0]);
}

template <typename DistType, typename RealType>
inline DistType rng_distribution_init(const std::array<RealType, 2> &param)
{
    return DistType(param[0], param[1]);
}

template <typename RealType, typename vSMCDistType>
inline RealType rng_distribution_chi2(
    std::size_t n, const RealType *r, const vSMCDistType &dist)
{
    vsmc::Vector<RealType> rval(r, r + n);
    std::sort(rval.begin(), rval.end());

    DistTrait<vSMCDistType> traits;
    vsmc::Vector<RealType> partition(traits.partition(n, dist));

    const std::size_t k = partition.size() + 1;
    vsmc::Vector<RealType> count(k);
    std::size_t j = 0;
    for (std::size_t i = 0; i != k - 1; ++i) {
        std::size_t c = 0;
        while (j != rval.size() && rval[j] <= partition[i]) {
            ++c;
            ++j;
        }
        count[i] = static_cast<RealType>(c);
    }
    count.back() = static_cast<RealType>(rval.size() - j);

    const RealType np = n * static_cast<RealType>(1) / k;
    RealType s = 0;
    for (std::size_t i = 0; i != k; ++i)
        s += (count[i] - np) * (count[i] - np) / np;

    boost::math::chi_squared_distribution<RealType> chi2(
        static_cast<RealType>(k - 1));

    return boost::math::cdf(chi2, s);
}

template <typename RealType, typename vSMCDistType>
inline RealType rng_distribution_ksad(
    std::size_t n, const RealType *r, const vSMCDistType &dist)
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

template <typename RealType>
inline void rng_distribution_summary_pval(RealType pval, int twid)
{
    std::stringstream ss;
    if (pval < 50)
        ss << "*";
    ss << pval << "%";
    std::cout << std::setw(twid) << std::right << ss.str();
}

template <typename RealType>
inline void rng_distribution_summary_pval(
    const vsmc::Vector<std::string> &names,
    const std::array<vsmc::Vector<RealType>, 6> &pval, int nwid, int twid)
{
    std::size_t D = names.size();
    std::size_t R = pval[0].size() / D;
    std::size_t lwid =
        static_cast<std::size_t>(nwid + twid * (3 + VSMC_HAS_MKL));

    const RealType *p0 = pval[0].data();
    const RealType *p1 = pval[1].data();
    const RealType *p2 = pval[2].data();
    const RealType *p3 = pval[3].data();
    const RealType *p4 = pval[4].data();
    const RealType *p5 = pval[5].data();
    for (std::size_t i = 0; i != D; ++i) {
        std::cout << std::string(lwid, '=') << std::endl;
        std::cout << std::setw(nwid) << std::left << names[i];
        std::cout << std::setw(twid) << std::right << "STD";
        std::cout << std::setw(twid) << std::right << "vSMC";
        std::cout << std::setw(twid) << std::right << "Batch";
#if VSMC_HAS_MKL
        std::cout << std::setw(twid) << std::right << "MKL";
#endif
        std::cout << std::endl;
        std::cout << std::string(lwid, '-') << std::endl;

        std::cout << std::setw(nwid) << std::left << "One level test (2.5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p0++, twid);
        std::cout << std::endl;

        std::cout << std::setw(nwid) << std::left << "One level test (5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p1++, twid);
        std::cout << std::endl;

        std::cout << std::setw(nwid) << std::left << "One level test (10%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p2++, twid);
        std::cout << std::endl;

        std::cout << std::setw(nwid) << std::left << "Two level test (2.5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p3++, twid);
        std::cout << std::endl;

        std::cout << std::setw(nwid) << std::left << "Two level test (5%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p4++, twid);
        std::cout << std::endl;

        std::cout << std::setw(nwid) << std::left << "Two level test (10%)";
        for (std::size_t r = 0; r != R; ++r)
            rng_distribution_summary_pval(*p5++, twid);
        std::cout << std::endl;
    }
    std::cout << std::string(lwid, '-') << std::endl;
}

template <typename vSMCDistType, typename RealType, std::size_t ParamNum>
inline void rng_distribution_test_pval(std::size_t N, std::size_t M,
    const std::array<RealType, ParamNum> &param,
    vsmc::Vector<std::string> &names,
    std::array<vsmc::Vector<RealType>, 6> &pval)
{
    using trait_type = DistTrait<vSMCDistType>;
    using STDDistType = typename trait_type::std_type;

    trait_type trait;
    names.push_back(trait.name(param));

    vsmc::RNG rng;
    vSMCDistType dist_vsmc(rng_distribution_init<vSMCDistType>(param));
    STDDistType dist_std(rng_distribution_init<STDDistType>(param));

    vsmc::Vector<RealType> r(N);
    vsmc::Vector<RealType> chi2(M);
    vsmc::Vector<RealType> ksad(M);

    for (std::size_t i = 0; i != M; ++i) {
        for (std::size_t j = 0; j != N; ++j)
            r[j] = dist_std(rng);
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    rng_distribution_pval(chi2, ksad, pval);

    for (std::size_t i = 0; i != M; ++i) {
        for (std::size_t j = 0; j != N; ++j)
            r[j] = dist_vsmc(rng);
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    rng_distribution_pval(chi2, ksad, pval);

    for (std::size_t i = 0; i != M; ++i) {
        vsmc::rand(rng, dist_vsmc, N, r.data());
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    rng_distribution_pval(chi2, ksad, pval);

#if VSMC_HAS_MKL
    vsmc::MKL_SFMT19937 rng_mkl;
    for (std::size_t i = 0; i != M; ++i) {
        vsmc::rand(rng_mkl, dist_vsmc, N, r.data());
        chi2[i] = rng_distribution_chi2(N, r.data(), dist_vsmc);
        ksad[i] = rng_distribution_ksad(N, r.data(), dist_vsmc);
    }
    rng_distribution_pval(chi2, ksad, pval);
#endif // VSMC_HAS_MKL
}

template <typename vSMCDistType>
inline void rng_distribution_test_pval(
    std::size_t N, std::size_t M, int nwid, int twid)
{
    using result_type = typename vSMCDistType::result_type;

    DistTrait<vSMCDistType> trait;
    auto params = trait.params();

    vsmc::Vector<std::string> names;
    std::array<vsmc::Vector<result_type>, 6> pval;
    for (const auto &param : params)
        rng_distribution_test_pval<vSMCDistType>(N, M, param, names, pval);
    rng_distribution_summary_pval(names, pval, nwid, twid);
}

template <typename RealType>
inline void rng_distribution_pval(
    std::size_t N, std::size_t M, int nwid, int twid)
{
    VSMC_DEFINE_EXAMPLE_RNG_DISTRIBUTION_TEST(pval);
}

inline void rng_distribution(std::size_t N, std::size_t M)
{
    N = std::max(N, static_cast<std::size_t>(10000));
    M = std::max(M, static_cast<std::size_t>(10));

    int nwid = 30;
    int twid = 12;

    rng_distribution_pval<float>(N, M, nwid, twid);
    rng_distribution_pval<double>(N, M, nwid, twid);
}

#endif // VSMC_EXAMPLE_RNG_DISTRIBUTION_HPP
