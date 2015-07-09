//============================================================================
// vSMC/include/vsmc/rng/stable_distribution.hpp
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

#ifndef VSMC_RNG_STABLE_DISTRIBUTION_HPP
#define VSMC_RNG_STABLE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(a)  \
    VSMC_RUNTIME_ASSERT((a > 0 && a <= 2),                                    \
        "**StableDistribution** CONSTRUCTED WITH INVALID "                    \
        "STABILITY PARAMETER VALUE")

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(a)   \
    VSMC_RUNTIME_ASSERT((a >= -1 && a <= 1),                                  \
        "**StableDistribution** CONSTRUCTED WITH INVALID "                    \
        "SKEWNESS PARAMETER VALUE")

#define VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(a)      \
    VSMC_RUNTIME_ASSERT((a > 0),                                              \
        "**StableDistribution** CONSTRUCTED WITH INVALID "                    \
        "SCALE PARAMETER VALUE")

namespace vsmc
{

namespace internal
{

template <typename RealType>
class StableDistributionConstant
{
    public:
    RealType zeta;
    RealType xi;
    RealType a;
    RealType b;
    RealType c;
    RealType d;
    RealType e;

    StableDistributionConstant(RealType stability, RealType skewness,
        RealType location, RealType scale)
    {
        if (is_equal<RealType>(stability, 1)) {
            xi = const_pi_by2<RealType>();
            d = 1 / xi;
            e = location +
                2 * const_pi_inv<RealType>() * skewness * scale *
                    std::log(scale);
        } else {
            zeta = -skewness * std::tan(const_pi_by2<RealType>() * stability);
            xi = 1 / stability * std::atan(-zeta);
            a = std::log(1 + zeta * zeta) / 2 / stability;
            b = stability * xi;
            c = 1 / stability;
            d = (1 - stability) / stability;
            e = 1 - stability;
        }
    }
}; // class StableDistributionConstant

} // namespace vsmc::internal

template <typename RealType, typename RNGType>
inline void stable_distribution(RNGType &, std::size_t, RealType *, RealType,
    RealType, RealType, RealType);

/// \brief Stable distribution
/// \ingroup Distribution
template <typename RealType = double>
class StableDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = StableDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = StableDistribution<RealType>;

        explicit param_type(result_type stability = 1,
            result_type skewness = 0, result_type location = 0,
            result_type scale = 1)
            : stability_(stability)
            , skewness_(skewness)
            , location_(location)
            , scale_(scale)
            , constant_(stability_, skewness_, location_, scale_)
        {
            invariant();
        }

        result_type stability() const { return stability_; }
        result_type skewness() const { return skewness_; }
        result_type location() const { return location_; }
        result_type scale() const { return scale_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.stability_, param2.stability_))
                return false;
            if (!internal::is_equal(param1.skewness_, param2.skewness_))
                return false;
            if (!internal::is_equal(param1.location_, param2.location_))
                return false;
            if (!internal::is_equal(param1.scale_, param2.scale_))
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

            os << param.stability_ << ' ';
            os << param.skewness_ << ' ';
            os << param.location_ << ' ';
            os << param.scale_ << ' ';

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type stability = 0;
            result_type skewness = 0;
            result_type location = 0;
            result_type scale = 0;
            is >> std::ws >> stability;
            is >> std::ws >> skewness;
            is >> std::ws >> location;
            is >> std::ws >> scale;

            if (is.good()) {
                if (stability > 0 && stability <= 2 && skewness >= -1 &&
                    skewness <= 1 && scale > 0) {
                    param = param_type(stability, skewness, location, scale);
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type stability_;
        result_type skewness_;
        result_type location_;
        result_type scale_;
        const internal::StableDistributionConstant<RealType> constant_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(
                stability_);
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(
                skewness_);
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(
                scale_);
        }

        void reset() {}
    }; // class param_type

    explicit StableDistribution(result_type stability = 1,
        result_type skewness = 0, result_type location = 0,
        result_type scale = 1)
        : param_(stability, skewness, location, scale)
    {
    }

    explicit StableDistribution(const param_type &param) : param_(param) {}

    result_type stability() const { return param_.stability(); }
    result_type skewness() const { return param_.skewness(); }
    result_type location() const { return param_.location(); }
    result_type scale() const { return param_.scale(); }

    result_type min() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        return internal::is_equal<result_type>(param_.stability_, 1) ?
            generate_1(rng) :
            generate_a(rng);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        stable_distribution(
            rng, n, r, stability(), skewness(), location(), scale());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;

    template <typename RNGType>
    result_type generate_1(RNGType &rng) const
    {
        U01OCDistribution<RealType> runif;
        result_type w = -std::log(runif(rng));
        result_type u =
            runif(rng) * const_pi<result_type>() - const_pi_by2<result_type>();
        result_type c = const_pi_by2<result_type>() + param_.skewness_ * u;
        result_type a = c * std::tan(u);
        result_type b =
            std::log(const_pi_by2<result_type>() * w * std::cos(u));
        c = std::log(c);
        result_type x = param_.constant_.d * (a - param_.skewness_ * (b - c));

        return param_.constant_.e + param_.scale_ * x;
    }

    template <typename RNGType>
    result_type generate_a(RNGType &rng) const
    {
        U01OCDistribution<RealType> runif;
        result_type w = -std::log(runif(rng));
        result_type u =
            runif(rng) * const_pi<result_type>() - const_pi_by2<result_type>();
        result_type b = std::sin(param_.constant_.b + param_.stability_ * u);
        result_type c = param_.constant_.c * std::log(std::cos(u));
        result_type d = param_.constant_.d *
            std::log(std::cos(param_.constant_.e * u - param_.constant_.b) /
                            w);
        result_type x = b * std::exp(param_.constant_.zeta - c + d);

        return param_.location_ + param_.scale_ * x;
    }
}; // class StableDistributionBase

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void stable_distribution_impl_1(RNGType &rng, std::size_t n,
    RealType *r, RealType, RealType skewness, RealType, RealType scale,
    const StableDistributionConstant<RealType> &constant)
{
    RealType s[K * 5];
    RealType *const w = s;
    RealType *const u = s + n;
    RealType *const a = s + n * 2;
    RealType *const b = s + n * 3;
    RealType *const c = s + n * 4;
    u01_oc_distribution(rng, n * 2, s);
    log(n, w, w);
    mul(n, static_cast<RealType>(-1), w, w);
    fma(n, -const_pi_by2<RealType>(), const_pi<RealType>(), u, u);
    fma(n, const_pi_by2<RealType>(), skewness, u, c);
    tan(n, u, a);
    mul(n, c, a, a);
    cos(n, u, b);
    mul(n, w, b, b);
    mul(n, const_pi_by2<RealType>(), b, b);
    log(n, b, b);
    log(n, c, c);
    sub(n, b, c, r);
    fma(n, a, -skewness, r, r);
    fma(n, constant.e, constant.d * scale, r, r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void stable_distribution_impl_a(RNGType &rng, std::size_t n,
    RealType *r, RealType stability, RealType, RealType location,
    RealType scale, const StableDistributionConstant<RealType> &constant)
{
    RealType s[K * 5];
    RealType *const w = s;
    RealType *const u = s + n;
    RealType *const b = s + n * 2;
    RealType *const c = s + n * 3;
    RealType *const d = s + n * 4;
    u01_oc_distribution(rng, n * 2, s);
    log(n, w, w);
    mul(n, static_cast<RealType>(-1), w, w);
    fma(n, -const_pi_by2<RealType>(), const_pi<RealType>(), u, u);
    fma(n, constant.b, stability, u, b);
    sin(n, b, b);
    cos(n, u, c);
    log(n, c, c);
    mul(n, constant.c, c, c);
    fma(n, -constant.b, constant.e, u, d);
    cos(n, d, d);
    div(n, d, w, d);
    log(n, d, d);
    mul(n, constant.d, d, d);
    sub(n, constant.zeta, c, r);
    add(n, d, r, r);
    exp(n, r, r);
    mul(n, b, r, r);
    fma(n, location, scale, r, r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void stable_distribution_impl(RNGType &rng, std::size_t n, RealType *r,
    RealType stability, RealType skewness, RealType location, RealType scale,
    const StableDistributionConstant<RealType> &constant)
{
    if (is_equal<RealType>(stability, 1)) {
        stable_distribution_impl_1<K>(
            rng, n, r, stability, skewness, location, scale, constant);
    } else {
        stable_distribution_impl_a<K>(
            rng, n, r, stability, skewness, location, scale, constant);
    }
}

} // namespace vsmc::internal

/// \brief Generating stable random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void stable_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType stability, RealType skewness, RealType location, RealType scale)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    const internal::StableDistributionConstant<RealType> constant(
        stability, skewness, location, scale);
    for (std::size_t i = 0; i != m; ++i) {
        internal::stable_distribution_impl<k>(
            rng, k, r + i * k, stability, skewness, location, scale, constant);
    }
    internal::stable_distribution_impl<k>(
        rng, l, r + m * k, stability, skewness, location, scale, constant);
}

} // namespace vsmc

#endif // VSMC_RNG_STABLE_DISTRIBUTION_HPP
