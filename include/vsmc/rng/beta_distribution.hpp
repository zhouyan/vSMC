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

namespace internal
{

enum BetaDistributionAlgorithm {
    BetaDistributionAlgorithm11,
    BetaDistributionAlgorithm1X,
    BetaDistributionAlgorithmX1,
    BetaDistributionAlgorithmC,
    BetaDistributionAlgorithmJ,
    BetaDistributionAlgorithmA1,
    BetaDistributionAlgorithmA2,
    BetaDistributionAlgorithmA3
}; // enum BetaDistributionAlgorithm

template <typename RealType>
BetaDistributionAlgorithm beta_distribution_algorithm(
    RealType alpha, RealType beta)
{
    const RealType K = static_cast<RealType>(0.852);
    const RealType C = static_cast<RealType>(-0.956);
    const RealType D = beta + K * alpha * alpha + C;
    if (is_equal<RealType>(alpha, 1) && is_equal<RealType>(beta, 1))
        return BetaDistributionAlgorithm11;
    else if (is_equal<RealType>(alpha, 1))
        return BetaDistributionAlgorithm1X;
    else if (is_equal<RealType>(beta, 1))
        return BetaDistributionAlgorithmX1;
    else if (alpha > 1 && beta > 1)
        return BetaDistributionAlgorithmC;
    else if (alpha < 1 && beta < 1 && D <= 0)
        return BetaDistributionAlgorithmJ;
    else if (alpha < 1 && beta < 1)
        return BetaDistributionAlgorithmA1;
    else if (alpha < 1 && beta > 1)
        return BetaDistributionAlgorithmA2;
    else if (alpha > 1 && beta < 1)
        return BetaDistributionAlgorithmA3;
}

} // namespace internal

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
            : alpha_(alpha)
            , beta_(beta)
            , a_(0)
            , b_(0)
            , algorithm_(internal::beta_distribution_algorithm(alpha_, beta_))
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
        result_type a_;
        result_type b_;
        result_type t_;
        result_type p_;
        internal::BetaDistributionAlgorithm algorithm_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_BETA_DISTRIBUTION_PARAM_CHECK(
                alpha_, beta_);
            switch (algorithm_) {
                case internal::BetaDistributionAlgorithm11: break;
                case internal::BetaDistributionAlgorithm1X:
                    b_ = 1 / beta_;
                    break;
                case internal::BetaDistributionAlgorithmX1:
                    a_ = 1 / alpha_;
                    break;
                case internal::BetaDistributionAlgorithmC:
                    a_ = alpha_ + beta_;
                    b_ = std::min(alpha_, beta_);
                    if (b_ > 1)
                        b_ = std::sqrt((2 * alpha_ * beta_ - a_) / (a_ - 2));
                    b_ = 1 / b_;
                    t_ = alpha_ + 1 / b_;
                    p_ = a_ * std::log(a_);
                    break;
                case internal::BetaDistributionAlgorithmJ:
                    a_ = 1 / alpha_;
                    b_ = 1 / beta_;
                    break;
                case internal::BetaDistributionAlgorithmA1:
                    a_ = 1 / alpha_;
                    b_ = 1 / beta_;
                    t_ = std::sqrt(alpha_ * (1 - alpha_));
                    t_ /= t_ + std::sqrt(beta_ * (1 - beta_));
                    p_ = beta_ * t_;
                    p_ /= p_ + alpha_ * (1 - t_);
                    break;
                case internal::BetaDistributionAlgorithmA2:
                    a_ = 1 / alpha_;
                    b_ = 1 / beta_;
                    t_ = 1 - alpha_;
                    t_ /= t_ + beta_;
                    p_ = beta_ * t_;
                    p_ /= p_ + alpha_ * std::pow(1 - t_, beta_);
                    break;
                case internal::BetaDistributionAlgorithmA3:
                    a_ = 1 / beta_;
                    b_ = 1 / alpha_;
                    t_ = 1 - beta_;
                    t_ /= t_ + alpha_;
                    p_ = alpha_ * t_;
                    p_ /= p_ + beta_ * std::pow(1 - t_, alpha_);
                    break;
            }
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
    result_type max() const { return 1; }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        switch (param_.algorithm_) {
            case internal::BetaDistributionAlgorithm11:
                return generate_11(rng);
                break;
            case internal::BetaDistributionAlgorithm1X:
                return generate_1x(rng);
                break;
            case internal::BetaDistributionAlgorithmX1:
                return generate_x1(rng);
                break;
            case internal::BetaDistributionAlgorithmC:
                return generate_c(rng);
                break;
            case internal::BetaDistributionAlgorithmJ:
                return generate_j(rng);
                break;
            case internal::BetaDistributionAlgorithmA1:
                return generate_a1(rng);
                break;
            case internal::BetaDistributionAlgorithmA2:
                return generate_a2(rng);
                break;
            case internal::BetaDistributionAlgorithmA3:
                return generate_a3(rng);
                break;
        }
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;

    template <typename RNGType>
    result_type generate_11(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;

        return runif(rng);
    }

    template <typename RNGType>
    result_type generate_1x(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;

        return 1 - std::pow(1 - runif(rng), param_.b_);
    }

    template <typename RNGType>
    result_type generate_x1(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;

        return std::pow(runif(rng), param_.a_);
    }

    template <typename RNGType>
    result_type generate_c(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        const result_type ln_4 = 2 * const_ln_2<result_type>();
        result_type y = 0;
        result_type z = 0;
        result_type left = 0;
        result_type right = 0;
        do {
            result_type u1 = runif(rng);
            result_type u2 = runif(rng);
            result_type v = param_.b_ * std::log(u1 / (1 - u1));
            y = param_.alpha_ * std::exp(v);
            z = param_.beta_ + y;
            left = param_.p_ - param_.a_ * std::log(z) + param_.t_ * v - ln_4;
            right = std::log(u1 * u1 * u2);
        } while (left < right);

        return y / z;
    }

    template <typename RNGType>
    result_type generate_j(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        result_type x = 0;
        result_type y = 0;
        do {
            x = std::pow(runif(rng), param_.a_);
            y = std::pow(runif(rng), param_.b_);
        } while (x + y > 1);

        return x / (x + y);
    }

    template <typename RNGType>
    result_type generate_a1(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        while (true) {
            result_type u = runif(rng);
            result_type e = -std::log(runif(rng));
            result_type x = 0;
            result_type v = 0;
            if (u < param_.p_) {
                x = param_.t_ * std::pow(u / param_.p_, param_.a_);
                v = (1 - param_.beta_) * std::log((1 - x) / (1 - param_.t_));
            } else {
                x = 1 -
                    (1 - param_.t_) *
                        std::pow((1 - u) / (1 - param_.p_), param_.b_);
                v = (1 - param_.a_) * std::log(x / param_.t_);
            }
            if (v < e)
                return x;
        }
    }

    template <typename RNGType>
    result_type generate_a2(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        while (true) {
            result_type u = runif(rng);
            result_type e = -std::log(runif(rng));
            result_type x = 0;
            result_type v = 0;
            if (u < param_.p_) {
                x = param_.t_ * std::pow(u / param_.p_, param_.a_);
                v = (1 - param_.beta_) * std::log(1 - x);
            } else {
                x = 1 -
                    (1 - param_.t_) *
                        std::pow((1 - u) / (1 - param_.p_), param_.b_);
                v = (1 - param_.alpha_) * std::log(x / param_.t_);
            }
            if (v < e)
                return x;
        }
    }

    template <typename RNGType>
    result_type generate_a3(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        while (true) {
            result_type u = runif(rng);
            result_type e = -std::log(runif(rng));
            result_type x = 0;
            result_type v = 0;
            if (u < param_.p_) {
                x = param_.t_ * std::pow(u / param_.p_, param_.a_);
                v = (1 - param_.alpha_) * std::log(1 - x);
            } else {
                x = 1 -
                    (1 - param_.t_) *
                        std::pow((1 - u) / (1 - param_.p_), param_.b_);
                v = (1 - param_.beta_) * std::log(x / param_.t_);
            }
            if (v < e)
                return 1 - x;
        }
    }
}; // class BetaDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_11(
    RNGType &rng, std::size_t n, RealType *r, RealType, RealType, RealType *)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_1x(RNGType &rng, std::size_t n, RealType *r,
    RealType, RealType beta, RealType *)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_x1(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType, RealType *)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_c(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_j(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_a1(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_a2(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl_a3(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
}

template <typename RealType, typename RNGType>
inline void beta_distribution_impl(RNGType &rng, std::size_t n, RealType *r,
    RealType alpha, RealType beta, RealType *s)
{
    switch (beta_distribution_algorithm(alpha, beta)) {
        case BetaDistributionAlgorithm11:
            beta_distribution_impl_11(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithm1X:
            beta_distribution_impl_1x(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmX1:
            beta_distribution_impl_a1(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmC:
            beta_distribution_impl_c(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmJ:
            beta_distribution_impl_j(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmA1:
            beta_distribution_impl_a1(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmA2:
            beta_distribution_impl_a2(rng, n, r, alpha, beta, s);
            break;
        case BetaDistributionAlgorithmA3:
            beta_distribution_impl_a3(rng, n, r, alpha, beta, s);
            break;
    }
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
