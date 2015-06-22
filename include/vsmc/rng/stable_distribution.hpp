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

/// \brief Generating stable random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
void stable_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType stability = 1, RealType skewness = 0, RealType location = 0,
    RealType scale = 1)
{
    U01DistributionType<RNGType, RealType> runif;

    if (stability < 1 || stability > 1) {
        Vector<RealType> w(n);
        Vector<RealType> u(n);
        Vector<RealType> b(n);
        Vector<RealType> c(n);
        Vector<RealType> d(n);
        RealType zeta =
            -skewness * std::tan(math::pi_by2<RealType>() * stability);
        RealType xi = 1 / stability * std::atan(-zeta);
        RealType a = 0.5 * std::log(1 + zeta * zeta) / stability;
        for (std::size_t i = 0; i != n; ++i)
            w[i] = -runif(rng);
        for (std::size_t i = 0; i != n; ++i)
            u[i] = runif(rng) - 0.5;
        math::vLog1p(n, w.data(), w.data());
        for (std::size_t i = 0; i != n; ++i)
            w[i] = -w[i];
        math::scal(n, math::pi<RealType>(), u.data(), 1);
        for (std::size_t i = 0; i != n; ++i)
            r[i] = u[i] + xi;
        math::scal(n, stability, r, 1);
        math::vSin(n, r, b.data());
        math::vCos(n, u.data(), c.data());
        math::vLn(n, c.data(), c.data());
        math::scal(n, 1 / stability, c.data(), 1);
        math::vSub(n, u.data(), r, d.data());
        math::vCos(n, d.data(), d.data());
        math::vDiv(n, d.data(), w.data(), d.data());
        math::vLn(n, d.data(), d.data());
        math::scal(n, (1 - stability) / stability, d.data(), 1);
        for (std::size_t i = 0; i != n; ++i)
            r[i] = a - c[i];
        math::vAdd(n, r, d.data(), r);
        math::vExp(n, r, r);
        math::vMul(n, b.data(), r, r);
        for (std::size_t i = 0; i != n; ++i)
            r[i] = location + scale * r[i];
    } else {
        Vector<RealType> w(n);
        Vector<RealType> u(n);
        Vector<RealType> a(n);
        Vector<RealType> b(n);
        Vector<RealType> c(n);
        for (std::size_t i = 0; i != n; ++i)
            w[i] = -runif(rng);
        for (std::size_t i = 0; i != n; ++i)
            u[i] = runif(rng) - 0.5;
        math::vLog1p(n, w.data(), w.data());
        math::scal(n, -math::pi_by2<RealType>(), w.data(), 1);
        math::scal(n, math::pi<RealType>(), u.data(), 1);
        math::vTan(n, u.data(), a.data());
        for (std::size_t i = 0; i != n; ++i)
            c[i] = skewness * u[i];
        for (std::size_t i = 0; i != n; ++i)
            r[i] = math::pi_by2<RealType>() + c[i];
        math::vMul(n, a.data(), r, a.data());
        math::vCos(n, u.data(), b.data());
        math::vMul(n, w.data(), b.data(), b.data());
        math::vLn(n, b.data(), b.data());
        math::vLn(n, r, c.data());
        math::vSub(n, b.data(), c.data(), r);
        math::scal(n, skewness, r, 1);
        math::vSub(n, a.data(), r, r);
        RealType offset = location +
            2 * math::pi_inv<RealType>() * skewness * scale * std::log(scale);
        RealType coeff = scale / math::pi_by2<RealType>();
        for (std::size_t i = 0; i != n; ++i)
            r[i] = offset + coeff * r[i];
    }
}

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
            os << param.zeta_ << ' ';
            os << param.xi_ << ' ';
            os << param.stability_1_;

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
            result_type zeta;
            result_type xi;
            bool stability_1;
            is >> std::ws >> stability;
            is >> std::ws >> skewness;
            is >> std::ws >> location;
            is >> std::ws >> scale;
            is >> std::ws >> zeta;
            is >> std::ws >> xi;
            is >> std::ws >> stability_1;

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
        result_type zeta_;
        result_type xi_;
        bool stability_1_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_STABILITY(
                stability_);
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SKEWNESS(
                skewness_);
            VSMC_RUNTIME_ASSERT_RNG_STABLE_DISTRIBUTION_PARAM_CHECK_SCALE(
                scale_);

            if (stability_ < 1 || stability_ > 1) {
                stability_1_ = false;
                zeta_ = -skewness_ *
                    std::tan(math::pi_by2<result_type>() * stability_);
                xi_ = 1 / stability_ * std::atan(-zeta_);
            } else {
                stability_1_ = true;
                zeta_ = -std::numeric_limits<result_type>::infinity();
                xi_ = math::pi_by2<result_type>();
            }
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

    result_type stability() const { return param_.stability_; }
    result_type skewness() const { return param_.skewness_; }
    result_type location() const { return param_.location_; }
    result_type scale() const { return param_.scale_; }

    result_type min VSMC_MNE() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max VSMC_MNE() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        if (param_.stability_1_)
            return trans_1(standard_1(rng));
        else
            return trans_a(standard_a(rng));
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;

    template <typename RNGType>
    result_type standard_1(RNGType &rng) const
    {
        U01DistributionType<RNGType, RealType> runif;
        result_type w = -std::log(1 - runif(rng));
        result_type u = (runif(rng) - 0.5) * math::pi<result_type>();
        result_type a =
            (math::pi_by2<result_type>() + param_.skewness_ * u) * std::tan(u);
        result_type b =
            std::log(math::pi_by2<result_type>() * w * std::cos(u));
        result_type c =
            std::log(math::pi_by2<result_type>() + param_.skewness_ * u);
        result_type x = (a - param_.skewness_ * (b - c)) / param_.xi_;

        return x;
    }

    template <typename RNGType>
    result_type standard_a(RNGType &rng) const
    {
        U01DistributionType<RNGType, RealType> runif;
        result_type w = -std::log(1 - runif(rng));
        result_type u = (runif(rng) - 0.5) * math::pi<result_type>();
        result_type a = 0.5 * std::log(1 + param_.zeta_ * param_.zeta_) /
            param_.stability_;
        result_type b = std::sin(param_.stability_ * (u + param_.xi_));
        result_type c = std::log(std::cos(u)) / param_.stability_;
        result_type d = (1 - param_.stability_) / param_.stability_ *
            std::log(std::cos(u - param_.stability_ * (u + param_.xi_)) / w);
        result_type x = b * std::exp(a - c + d);

        return x;
    }

    result_type trans_1(result_type x) const
    {
        return param_.scale_ * x + param_.location_ +
            2 * math::pi_inv<result_type>() * param_.skewness_ *
            param_.scale_ * std::log(param_.scale_);
    }

    result_type trans_a(result_type x) const
    {
        return param_.scale_ * x + param_.location_;
    }
}; // class StableDistributionBase

} // namespace vsmc

#endif // VSMC_RNG_STABLE_DISTRIBUTION_HPP
