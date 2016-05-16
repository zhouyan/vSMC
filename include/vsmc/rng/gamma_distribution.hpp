//============================================================================
// vSMC/include/vsmc/rng/gamma_distribution.hpp
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

#ifndef VSMC_RNG_GAMMA_DISTRIBUTION_HPP
#define VSMC_RNG_GAMMA_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool gamma_distribution_check_param(RealType alpha, RealType beta)
{
    return alpha > 0 && beta > 0;
}

enum GammaDistributionAlgorithm {
    GammaDistributionAlgorithmT,
    GammaDistributionAlgorithmW,
    GammaDistributionAlgorithmN,
    GammaDistributionAlgorithmE
}; // enum GammaDistributionAlgorithm

template <typename RealType>
class GammaDistributionConstant
{
    public:
    GammaDistributionConstant(RealType alpha = 1, RealType beta = 1)
    {
        reset(alpha, beta);
    }

    void reset(RealType alpha, RealType)
    {
        if (alpha < static_cast<RealType>(0.6))
            algorithm = GammaDistributionAlgorithmT;
        else if (alpha < 1)
            algorithm = GammaDistributionAlgorithmW;
        else if (alpha > 1)
            algorithm = GammaDistributionAlgorithmN;
        else
            algorithm = GammaDistributionAlgorithmE;

        d = c = 0;
        switch (algorithm) {
            case GammaDistributionAlgorithmT:
                d = 1 - alpha;
                c = 1 / alpha;
                break;
            case GammaDistributionAlgorithmW:
                d = std::pow(alpha, alpha / (1 - alpha)) * (1 - alpha);
                c = 1 / alpha;
                break;
            case GammaDistributionAlgorithmN:
                d = alpha - const_one<RealType>() / 3;
                c = 1 / (3 * std::sqrt(d));
                break;
            case GammaDistributionAlgorithmE: break;
        }
    }

    RealType d;
    RealType c;
    GammaDistributionAlgorithm algorithm;
}; // class GammaDistributionConstant

} // namespace internal

/// \brief Gamma distribution
/// \ingroup Distribution
template <typename RealType>
class GammaDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Gamma, gamma, alpha, 1, beta, 1)

    public:
    result_type min() const { return 0; }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() { constant_.reset(alpha(), beta()); }

    private:
    internal::GammaDistributionConstant<RealType> constant_;

    bool is_equal(const distribution_type &other) const
    {
        if (!is_equal(constant_.d, other.constant_.d))
            return false;
        if (!is_equal(constant_.c, other.constant_.c))
            return false;
        if (!is_equal(constant_.algorithm, other.constant_.algorithm))
            return false;
        return true;
    }

    template <typename CharT, typename Traits>
    void ostream(std::basic_ostream<CharT, Traits> &) const
    {
    }

    template <typename CharT, typename Traits>
    void istream(std::basic_istream<CharT, Traits> &)
    {
        reset();
    }

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        if (param == param_)
            return generate(rng, param_, constant_);

        internal::GammaDistributionConstant<RealType> constant(
            param.alpha(), param.beta());

        return generate(rng, param, constant);
    }

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param,
        const internal::GammaDistributionConstant<RealType> &constant)
    {
        result_type r = 0;
        switch (constant.algorithm) {
            case internal::GammaDistributionAlgorithmT:
                r = generate_t(rng, param, constant);
                break;
            case internal::GammaDistributionAlgorithmW:
                r = generate_w(rng, param, constant);
                break;
            case internal::GammaDistributionAlgorithmN:
                r = generate_n(rng, param, constant);
                break;
            case internal::GammaDistributionAlgorithmE:
                r = generate_e(rng, param, constant);
                break;
        }

        return param.beta() * r;
    }

    template <typename RNGType>
    result_type generate_t(RNGType &rng, const param_type &param,
        const internal::GammaDistributionConstant<RealType> &constant)
    {
        U01OODistribution<RealType> u01;
        while (true) {
            result_type u = 1 - u01(rng);
            result_type e = -std::log(u01(rng));
            if (u > constant.d) {
                u = -std::log(constant.c * (1 - u));
                e += u;
                u = constant.d + param.alpha() * u;
            }
            result_type r = std::exp(constant.c * std::log(u));
            if (std::abs(r) < e)
                return r;
        }
    }

    template <typename RNGType>
    result_type generate_w(RNGType &rng, const param_type &,
        const internal::GammaDistributionConstant<RealType> &constant)
    {
        U01OODistribution<RealType> u01;
        result_type u = 0;
        result_type e = 0;
        result_type r = 0;
        do {
            u = -std::log(u01(rng));
            e = -std::log(u01(rng));
            r = std::exp(constant.c * std::log(u));
        } while (u + e < constant.d + r);

        return r;
    }

    template <typename RNGType>
    result_type generate_n(RNGType &rng, const param_type &,
        const internal::GammaDistributionConstant<RealType> &constant)
    {
        U01OODistribution<RealType> u01;
        NormalDistribution<RealType> rnorm(0, 1);
        while (true) {
            result_type u = u01(rng);
            result_type e = 0;
            result_type v = 0;
            result_type w = 0;
            do {
                w = rnorm(rng);
                v = 1 + constant.c * w;
            } while (v <= 0);
            v = v * v * v;

            e = 1 - static_cast<result_type>(0.0331) * (w * w) * (w * w);
            if (u < e)
                return constant.d * v;

            e = w * w / 2 + constant.d * (1 - v + std::log(v));
            if (std::log(u) < e)
                return constant.d * v;
        }
    }

    template <typename RNGType>
    result_type generate_e(RNGType &rng, const param_type &,
        const internal::GammaDistributionConstant<RealType> &)
    {
        U01OODistribution<RealType> u01;

        return -std::log(u01(rng));
    }
}; // class GammaDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline std::size_t gamma_distribution_impl_t(RNGType &rng, std::size_t n,
    RealType *r, RealType alpha, RealType beta,
    const GammaDistributionConstant<RealType> &constant)
{
    const RealType d = constant.d;
    const RealType c = constant.c;
    Array<RealType, K * 3> s;
    RealType *const u = s.data();
    RealType *const e = s.data() + n;
    RealType *const x = s.data() + n * 2;

    u01_oo_distribution(rng, n * 2, s.data());
    log(n, e, e);
    mul(n, static_cast<RealType>(-1), e, e);
    for (std::size_t i = 0; i != n; ++i) {
        if (u[i] > d) {
            u[i] = -std::log(c * (1 - u[i]));
            e[i] += u[i];
            u[i] = d + alpha * u[i];
        }
    }
    log(n, u, x);
    mul(n, c, x, x);
    exp(n, x, x);
    abs(n, x, u);
    mul(n, beta, x, x);

    std::size_t m = 0;
    for (std::size_t i = 0; i != n; ++i)
        if (u[i] < e[i])
            r[m++] = x[i];

    return m;
}

template <std::size_t K, typename RealType, typename RNGType>
inline std::size_t gamma_distribution_impl_w(RNGType &rng, std::size_t n,
    RealType *r, RealType, RealType beta,
    const GammaDistributionConstant<RealType> &constant)
{
    const RealType d = constant.d;
    const RealType c = constant.c;
    Array<RealType, K * 3> s;
    RealType *const u = s.data();
    RealType *const e = s.data() + n;
    RealType *const x = s.data() + n * 2;

    u01_oo_distribution(rng, n * 2, s.data());
    log(n * 2, s.data(), s.data());
    mul(n * 2, static_cast<RealType>(-1), s.data(), s.data());
    log(n, s.data(), x);
    mul(n, c, x, x);
    exp(n, x, x);
    add(n, u, e, u);
    add(n, d, x, e);
    mul(n, beta, x, x);

    std::size_t m = 0;
    for (std::size_t i = 0; i != n; ++i)
        if (u[i] > e[i])
            r[m++] = x[i];

    return m;
}

template <std::size_t K, typename RealType, typename RNGType>
inline std::size_t gamma_distribution_impl_n(RNGType &rng, std::size_t n,
    RealType *r, RealType, RealType beta,
    const GammaDistributionConstant<RealType> &constant)
{
    const RealType d = constant.d;
    const RealType c = constant.c;
    Array<RealType, K * 5> s;
    RealType *const u = s.data();
    RealType *const e = s.data() + n;
    RealType *const v = s.data() + n * 2;
    RealType *const w = s.data() + n * 3;
    RealType *const x = s.data() + n * 4;

    u01_oo_distribution(rng, n, u);
    normal_distribution(
        rng, n, w, const_zero<RealType>(), const_one<RealType>());
    fma(n, c, w, const_one<RealType>(), v);
    NormalDistribution<RealType> rnorm(0, 1);
    for (std::size_t i = 0; i != n; ++i) {
        if (v[i] <= 0) {
            do {
                w[i] = rnorm(rng);
                v[i] = 1 + c * w[i];
            } while (v[i] <= 0);
        }
    }
    sqr(n, v, e);
    mul(n, v, e, v);
    sqr(n, w, e);
    sqr(n, e, e);
    fma(n, -static_cast<RealType>(0.0331), e, const_one<RealType>(), e);
    mul(n, d * beta, v, x);

    std::size_t m = 0;
    for (std::size_t i = 0; i != n; ++i) {
        if (u[i] < e[i]) {
            r[m++] = x[i];
        } else {
            e[i] = w[i] * w[i] / 2 + d * (1 - v[i] + std::log(v[i]));
            if (std::log(u[i]) < e[i])
                r[m++] = x[i];
        }
    }

    return m;
}

template <std::size_t, typename RealType, typename RNGType>
inline std::size_t gamma_distribution_impl_e(RNGType &rng, std::size_t n,
    RealType *r, RealType, RealType beta,
    const GammaDistributionConstant<RealType> &)
{
    u01_oo_distribution(rng, n, r);
    log(n, r, r);
    mul(n, -beta, r, r);

    return n;
}

template <std::size_t K, typename RealType, typename RNGType>
inline std::size_t gamma_distribution_impl(RNGType &rng, std::size_t n,
    RealType *r, RealType alpha, RealType beta,
    const GammaDistributionConstant<RealType> &constant)
{
    switch (constant.algorithm) {
        case GammaDistributionAlgorithmT:
            return gamma_distribution_impl_t<K>(
                rng, n, r, alpha, beta, constant);
        case GammaDistributionAlgorithmW:
            return gamma_distribution_impl_w<K>(
                rng, n, r, alpha, beta, constant);
        case GammaDistributionAlgorithmN:
            return gamma_distribution_impl_n<K>(
                rng, n, r, alpha, beta, constant);
        case GammaDistributionAlgorithmE:
            return gamma_distribution_impl_e<K>(
                rng, n, r, alpha, beta, constant);
    }
    return 0;
}

} // namespace vsmc::internal

/// \brief Generating gamma random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void gamma_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType alpha, RealType beta)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**gamma_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = internal::BufferSize<RealType>::value;
    const internal::GammaDistributionConstant<RealType> constant(alpha);
    while (n > k) {
        std::size_t m = internal::gamma_distribution_impl<k>(
            rng, k, r, alpha, beta, constant);
        if (m == 0)
            break;
        n -= m;
        r += m;
    }
    std::size_t m =
        internal::gamma_distribution_impl<k>(rng, n, r, alpha, beta, constant);
    n -= m;
    r += m;
    if (n > 0) {
        GammaDistribution<RealType> dist(alpha, beta);
        for (std::size_t i = 0; i != n; ++i)
            r[i] = dist(rng);
    }
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(Gamma, gamma, alpha, beta)

} // namespace vsmc

#endif // VSMC_RNG_GAMMA_DISTRIBUTION_HPP
