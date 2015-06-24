//============================================================================
// vSMC/include/vsmc/rng/laplace_distribution.hpp
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

#ifndef VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
#define VSMC_RNG_LAPLACE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_LAPLACE_DISTRIBUTION_PARAM_CHECK(scale)       \
    VSMC_RUNTIME_ASSERT((scale > 0), "**LaplaceDistribution** CONSTRUCTED "   \
                                     "WITH INVALID SCALE PARAMETER VALUE")

namespace vsmc
{

namespace internal
{

template <typename RealType, typename RNGType>
inline void laplace_distribution_impl(RNGType &rng, std::size_t n, RealType *r,
    RealType location, RealType scale, RealType *s)
{
    u01_distribution(rng, n, r);
    for (std::size_t i = 0; i != n; ++i) {
        r[i] -= 0.5;
        if (r[i] > 0) {
            r[i] *= 2;
            s[i] = -scale;
        } else {
            r[i] *= -2;
            s[i] = scale;
        }
    }
    math::vLog1p(n, r, r);
    math::vMul(n, r, s, r);
    for (std::size_t i = 0; i != n; ++i)
        r[i] += location;
}

} // namespace vsmc::internal

/// \brief Generating laplace random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void laplace_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType location = 0, RealType scale = 1)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    Vector<RealType> s(k);
    for (std::size_t i = 0; i != m; ++i) {
        internal::laplace_distribution_impl(
            rng, k, r + i * k, location, scale, s.data());
    }
    internal::laplace_distribution_impl(
        rng, l, r + m * k, location, scale, s.data());
}

/// \brief Laplace distribution
/// \ingroup Distribution
template <typename RealType>
class LaplaceDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = LaplaceDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = LaplaceDistribution<RealType>;

        explicit param_type(result_type location = 0, result_type scale = 1)
            : location_(location), scale_(scale)
        {
            invariant();
        }

        result_type location() const { return location_; }
        result_type scale() const { return scale_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
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

            os << param.location_ << ' ';
            os << param.scale_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type location = 0;
            result_type scale = 0;
            is >> std::ws >> location;
            is >> std::ws >> scale;

            if (is.good()) {
                if (scale > 0)
                    param = param_type(location, scale);
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        result_type location_;
        result_type scale_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_LAPLACE_DISTRIBUTION_PARAM_CHECK(scale_);
        }

        void reset() {}
    }; // class param_type

    explicit LaplaceDistribution(
        result_type location = 0, result_type scale = 1)
        : param_(location, scale)
    {
    }

    explicit LaplaceDistribution(const param_type &param) : param_(param) {}

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
    result_type operator()(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;
        result_type u = runif(rng) - 0.5;

        return u > 0 ? param_.location_ - param_.scale_ * std::log1p(2 * u) :
                       param_.location_ + param_.scale_ * std::log1p(-2 * u);
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class LaplaceDistribution

} // namespace vsmc

#endif // VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
