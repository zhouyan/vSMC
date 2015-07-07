//============================================================================
// vSMC/include/vsmc/rng/beta_distribution.hpp
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

#ifndef VSMC_RNG_BETA_DISTRIBUTION_HPP
#define VSMC_RNG_BETA_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>
#include <vsmc/rng/normal_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_ALPHA_CHECK(alpha)    \
    VSMC_RUNTIME_ASSERT((alpha > 0), "**BetaDistribution** CONSTRUCTED "      \
                                     "WITH INVALID SHAPE PARAMETER VALUE")

#define VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_BETA_CHECK(beta)      \
    VSMC_RUNTIME_ASSERT((beta > 0), "**BetaDistribution** CONSTRUCTED "       \
                                    "WITH INVALID SHAPE PARAMETER VALUE")

#define VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_CHECK(alpha, beta)    \
    VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_ALPHA_CHECK(alpha);       \
    VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_BETA_CHECK(beta);

namespace vsmc
{

/// \brief beta distribution
/// \ingroup Distribution
template <typename RealType>
class BetaDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = BetaDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = BetaDistribution<RealType>;

        explicit param_type(result_type alpha = 1, result_type beta = 1)
            : alpha_(alpha), beta_(beta), d_(0), c_(0)
        {
            invariant();
        }

        result_type alpha() const { return alpha_; }
        result_type beta() const { return beta_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.alpha_, param2.alpha_))
                return false;
            if (!internal::is_equal(param1.beta_, param2.beta_))
                return false;
            return true;
        }

        friend bool operator!=(
            const param_type &param1, const param_type &param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.alpha_ << ' ';
            os << param.beta_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type alpha = 0;
            result_type beta = 0;
            is >> std::ws >> alpha;
            is >> std::ws >> beta;

            if (is.good()) {
                if (alpha > 0 && beta > 0)
                    param = param_type(alpha, beta);
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        result_type alpha_;
        result_type beta_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_CHECK(
                alpha_, beta_);
        }

        void reset() {}
    }; // class param_type

    explicit BetaDistribution(result_type alpha = 1, result_type beta = 1)
        : param_(alpha, beta)
    {
    }

    explicit BetaDistribution(const param_type &param) : param_(param) {}

    result_type alpha() const { return param_.alpha(); }
    result_type beta() const { return param_.beta(); }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        result_type r = 0;
        if (param_.alpha_ < 0.6)
            r = generate_t(rng);
        else if (param_.alpha_ < 1)
            r = generate_w(rng);
        else if (param_.alpha_ > 1)
            r = generate_n(rng);
        else
            r = generate_e(rng);

        return param_.beta_ * r;
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;

    template <typename RNGType>
    result_type generate_t(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        while (true) {
            result_type u = runif(rng);
            result_type e = -std::log(1 - runif(rng));
            if (u > param_.d_) {
                u = -std::log(param_.c_ * (1 - u));
                e += u;
                u = param_.d_ + param_.alpha_ * u;
            }
            result_type r = std::pow(u, param_.c_);
            if (std::abs(r) < e)
                return r;
        }
    }

    template <typename RNGType>
    result_type generate_w(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        result_type u = 0;
        result_type e = 0;
        result_type r = 0;
        do {
            u = -std::log(1 - runif(rng));
            e = -std::log(1 - runif(rng));
            r = std::pow(u, param_.c_);
        } while (u + e < param_.d_ + r);

        return r;
    }

    template <typename RNGType>
    result_type generate_n(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        NormalDistribution<RealType> rnorm(0, 1);
        while (true) {
            result_type u = 1 - runif(rng);
            result_type e = 0;
            result_type x = 0;
            result_type v = 0;
            do {
                x = rnorm(rng);
                v = 1 + param_.c_ * x;
            } while (v <= 0);
            v = v * v * v;

            e = 1 - static_cast<result_type>(0.0331) * (x * x) * (x * x);
            if (u < e)
                return param_.d_ * v;

            e = static_cast<result_type>(0.5) * x * x +
                param_.d_ * (1 - v + std::log(v));
            if (std::log(u) < e)
                return param_.d_ * v;
        }
    }

    template <typename RNGType>
    result_type generate_e(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        return -std::log(1 - runif(rng));
    }
}; // class BetaDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_u(
    RNGType &rng, std::size_t n, RealType *r, RealType, RealType, RealType *)
{
    u01_distribution(rng, n, r);
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_1b(RNGType &rng, std::size_t n, RealType *r,
    RealType, RealType beta, RealType *)
{
    u01_distribution(rng, n, r);
    pow(n, r, 1 / beta, r);
    sub(n, static_cast<RealType>(1), r, r);
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_a1(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType, RealType *)
{
    u01_distribution(rng, n, r);
    pow(n, r, 1 / alpha, r);
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_c(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
    const RealType t = alpha + beta;
    const RealType lambda = std::sqrt((2 * alpha * beta - t) / (t - 2));
    const RealType u = alpha + lambda;
    RealType *const u1 = s;
    RealType *const u2 = s + n;
    RealType *const v = s + n * 2;
    RealType *const y = s + n * 3;
    u01_distribution(rng, n * 2, s);
    sub(n, static_cast<RealType>(1), u2, v);
    div(n, u1, v, v);
    mul(n, 1 / lambda, v, v);
    exp(n, v, y);
    mul(n, alpha, y, y);
    add(n, beta, y, r);
    div(n, t, r, r);
    log(n, r, r);
    mul(n, t, r, r);
    fma(n, -const_ln_4<RealType>(), u, v, v);
    add(n, v, r, r);
    sqr(n, u1, u1);
    mul(n, u1, u2, v);

    BetaDistribution<RealType> dist(alpha, beta);
    for (std::size_t i = 0; i != n; ++i) {
        if (r[i] < v[i])
            r[i] = y[i] / (beta + y[i]);
        else
            r[i] = dist(rng);
    }
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_j(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
    const RealType a = 1 / alpha;
    const RealType b = 1 / beta;
    RealType *const u = s;
    RealType *const v = s + n;
    U01DistributionType<RNGType, RealType> runif;
    u01_distribution(rng, n * 2, s);
    pow(n, u, a, u);
    pow(n, v, b, v);
    add(n, u, v, r);
    div(n, u, r, r);

    BetaDistribution<RealType> dist(alpha, beta);
    for (std::size_t i = 0; i != n; ++i)
        if (u[i] + v[i] > 1)
            r[i] = dist(rng);
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_a(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
    const RealType K = static_cast<RealType>(0.852);
    const RealType C = static_cast<RealType>(0.956);
    bool is_1b = internal::is_equal(static_cast<RealType>(1), alpha);
    bool is_a1 = internal::is_equal(static_cast<RealType>(1), beta);
    bool is_11 = is_1b && is_a1;
    if (is_11)
        beta_distribution_impl_u(rng, n, r, alpha, beta, s);
    if (is_1b)
        beta_distribution_impl_1b(rng, n, r, alpha, beta, s);
    else if (is_a1)
        beta_distribution_impl_a1(rng, n, r, alpha, beta, s);
    else if (alpha < 1 && beta < 1)
        beta_distribution_impl_j(rng, n, r, alpha, beta, s);
    else if (alpha < 1 && beta > 1)
        beta_distribution_impl_j(rng, n, r, alpha, beta, s);
    else if (alpha > 1 && beta < 1)
        beta_distribution_impl_j(rng, n, r, alpha, beta, s);
    else if (alpha > 1 && beta > 1)
        beta_distribution_impl_j(rng, n, r, alpha, beta, s);
}

} // namespace vsmc::internal

/// \brief Generating beta random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void beta_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha = 0, RealType beta = 1)
{
    const std::size_t k = 500;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    RealType s[k * 2];
    for (std::size_t i = 0; i != m; ++i)
        internal::beta_distribution_impl(rng, k, r + i * k, alpha, beta, s);
    internal::beta_distribution_impl(rng, l, r + m * k, alpha, beta, s);
}

} // namespace vsmc

#endif // VSMC_RNG_BETA_DISTRIBUTION_HPP
