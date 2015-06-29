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

namespace internal
{

template <typename RealType, typename RNGType>
inline void normal_distribution_impl(RNGType &rng, std::size_t n, RealType *r,
    RealType mean, RealType stddev, RealType *s)
{
    u01_distribution(rng, n, r);
    const std::size_t nu = n / 2;
    RealType *const u1 = r;
    RealType *const u2 = r + nu;
    math::vLn(nu, u1, s);
    math::vMul(nu, static_cast<RealType>(-2), s, s);
    math::vSqrt(nu, s, s);
    math::vMul(nu, math::pi_2<RealType>(), u2, u2);
    math::vSinCos(nu, u2, u1, u2);
    math::vMul(nu, stddev, s, s);
    math::vFMA(nu, mean, s, u1, u1);
    math::vFMA(nu, mean, s, u2, u2);
}

} // namespace vsmc::internal

/// \brief Generating normal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void normal_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType mean = 0, RealType stddev = 1)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    RealType s[k / 2];
    for (std::size_t i = 0; i != m; ++i)
        internal::normal_distribution_impl(rng, k, r + i * k, mean, stddev, s);
    internal::normal_distribution_impl(rng, l, r + m * k, mean, stddev, s);
    if (n % 2 != 0) {
        U01DistributionType<RNGType, RealType> runif;
        RealType v1 = runif(rng);
        RealType v2 = runif(rng);
        r[n - 1] = mean +
            stddev * std::sqrt(-2 * std::log(v1)) *
                std::cos(math::pi_2<RealType>() * v2);
    }
}

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

    result_type mean() const { return param_.mean_; }
    result_type stddev() const { return param_.stddev_; }

    result_type min VSMC_MNE() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max VSMC_MNE() const
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

        result_type u = 0;
        result_type v = 0;
        generate_uv(rng, u, v, std::integral_constant<bool,
                                   internal::RNGBits<RNGType>::value >= 64>());
        result_type s = param_.stddev_ * std::sqrt(-2 * std::log(u));
        v = std::sin(math::pi_2<result_type>() * v);
        u = std::sqrt(1 - v * v);
        param_.v_ = param_.mean_ * v * s;
        param_.saved_ = true;

        return param_.mean_ + u * s;
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;

    template <typename RNGType>
    void generate_uv(
        RNGType &rng, result_type &u, result_type &v, std::false_type)
    {
        U01DistributionType<RNGType, RealType> runif;
        u = 1 - runif(rng);
        v = 1 - runif(rng);
    }

    template <typename RNGType>
    void generate_uv(
        RNGType &rng, result_type &u, result_type &v, std::true_type)
    {
        std::uint64_t r = static_cast<std::uint64_t>(rng());
        u = U01<std::uint32_t, RealType, Open, Closed>::eval(r);
        v = U01<std::uint32_t, RealType, Open, Closed>::eval(r >> 32);
    }
}; // class NormalDistribution

} // namespace vsmc

#endif // VSMC_RNG_NORMAL_DISTRIBUTION_HPP
