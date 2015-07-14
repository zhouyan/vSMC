//============================================================================
// vSMC/include/vsmc/rng/normal_distribution.hpp
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

#ifndef VSMC_RNG_NORMAL_DISTRIBUTION_HPP
#define VSMC_RNG_NORMAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_NORMAL_DISTRIBUTION_PARAM_CHECK(stddev)       \
    VSMC_RUNTIME_ASSERT((stddev > 0), "**NormalDistribution** CONSTRUCTED "   \
                                      "WITH INVALID STANDARD DEVIATION "      \
                                      "PARAMETER VALUE")

namespace vsmc
{

/// \brief Normal distribution
/// \ingroup Distribution
template <typename RealType>
class NormalDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = NormalDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = NormalDistribution<RealType>;

        explicit param_type(result_type mean = 0, result_type stddev = 1)
            : mean_(mean), stddev_(stddev), v_(0), saved_(false)
        {
            invariant();
        }

        result_type mean() const { return mean_; }
        result_type stddev() const { return stddev_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.mean_, param2.mean_))
                return false;
            if (!internal::is_equal(param1.stddev_, param2.stddev_))
                return false;
            if (!internal::is_equal(param1.v_, param2.v_))
                return false;
            if (param1.saved_ && !param2.saved)
                return false;
            if (!param1.saved_ && param2.saved)
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

            os << param.mean_ << ' ';
            os << param.stddev_ << ' ';
            os << param.v_ << ' ';
            os << param.saved_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type mean = 0;
            result_type stddev = 0;
            result_type v = 0;
            bool saved = false;
            is >> std::ws >> mean;
            is >> std::ws >> stddev;
            is >> std::ws >> v;
            is >> std::ws >> saved;

            if (is.good()) {
                if (stddev > 0) {
                    param = param_type(mean, stddev);
                    param.v_ = v;
                    param.saved_ = saved;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type mean_;
        result_type stddev_;
        result_type v_;
        bool saved_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_NORMAL_DISTRIBUTION_PARAM_CHECK(stddev_);
            saved_ = false;
        }

        void reset() { saved_ = false; }
    }; // class param_type

    explicit NormalDistribution(result_type mean = 0, result_type stddev = 1)
        : param_(mean, stddev)
    {
    }

    explicit NormalDistribution(const param_type &param) : param_(param) {}

    result_type mean() const { return param_.mean(); }
    result_type stddev() const { return param_.stddev(); }

    result_type min() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        if (param_.saved_) {
            param_.saved_ = false;
            return param_.v_;
        }

        U01OCDistribution<RealType> runif;
        result_type u = runif(rng);
        result_type v = runif(rng);
        result_type s = param_.stddev_ * std::sqrt(-2 * std::log(u));
        v *= const_pi_2<result_type>();
        u = std::cos(v);
        v = std::sin(v);
        param_.v_ = param_.mean_ + v * s;
        param_.saved_ = true;

        return param_.mean_ + u * s;
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        normal_distribution(rng, n, r, mean(), stddev());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class NormalDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void normal_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType mean, RealType stddev)
{
    RealType s[K / 2];
    const std::size_t nu = n / 2;
    RealType *const u1 = r;
    RealType *const u2 = r + nu;
    u01_oc_distribution(rng, n, r);
    log(nu, u1, s);
    mul(nu, static_cast<RealType>(-2), s, s);
    sqrt(nu, s, s);
    mul(nu, const_pi_2<RealType>(), u2, u2);
    sincos(nu, u2, u1, u2);
    mul(nu, stddev, s, s);
    fma(nu, s, u1, mean, u1);
    fma(nu, s, u2, mean, u2);
}

template <typename RealType>
inline void normal_distribution_pdf_impl(std::size_t n, const RealType *x,
    RealType *r, RealType mean, RealType stddev)
{
    sub(n, x, mean, r);
    mul(n, 1 / stddev, r, r);
    sqr(n, r, r);
    mul(n, static_cast<RealType>(-0.5), r, r);
    exp(n, r, r);
    mul(n, 1 / (stddev * const_sqrt_pi_2<RealType>()), r, r);
}

template <typename RealType>
inline void normal_distribution_cdf_impl(std::size_t n, const RealType *x,
    RealType *r, RealType mean, RealType stddev)
{
    sub(n, x, mean, r);
    mul(n, 1 / stddev, r, r);
    cdfnorm(n, r, r);
}

} // namespace vsmc::internal

/// \brief Generating normal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void normal_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType mean, RealType stddev)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::normal_distribution_impl<k>(rng, k, r, mean, stddev);
    internal::normal_distribution_impl<k>(rng, l, r, mean, stddev);
    if (n % 2 != 0) {
        U01OCDistribution<RealType> runif;
        RealType u = runif(rng);
        RealType v = runif(rng);
        r[l - 1] = mean +
            stddev * std::sqrt(-2 * std::log(u)) *
                std::cos(const_pi_2<RealType>() * v);
    }
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, NormalDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_NORMAL_DISTRIBUTION_HPP
