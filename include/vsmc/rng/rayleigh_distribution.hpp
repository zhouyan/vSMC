//============================================================================
// vSMC/include/vsmc/rng/rayleigh_distribution.hpp
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

#ifndef VSMC_RNG_RAYLEIGH_DISTRIBUTION_HPP
#define VSMC_RNG_RAYLEIGH_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_RAYLEIGH_DISTRIBUTION_PARAM_CHECK(sigma)      \
    VSMC_RUNTIME_ASSERT((sigma > 0), "**RayleighDistribution** "              \
                                     "CONSTRUCTED WITH INVALID SCALE "        \
                                     "PARAMETER VALUE")

namespace vsmc
{

/// \brief Rayleigh distribution
/// \ingroup Distribution
template <typename RealType>
class RayleighDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = RayleighDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = RayleighDistribution<RealType>;

        explicit param_type(result_type sigma = 1) : sigma_(sigma)
        {
            invariant();
        }

        result_type sigma() const { return sigma_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            return internal::is_equal(param1.sigma_, param2.sigma_);
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

            os << param.sigma_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type sigma = 0;
            is >> std::ws >> sigma;

            if (is.good()) {
                if (sigma > 0)
                    param.sigma_ = sigma;
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        result_type sigma_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_RAYLEIGH_DISTRIBUTION_PARAM_CHECK(sigma_);
        }

        void reset() {}
    }; // class param_type

    explicit RayleighDistribution(result_type b = 1) : param_(b) {}

    explicit RayleighDistribution(const param_type &param) : param_(param) {}

    result_type sigma() const { return param_.sigma(); }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        U01OCDistribution<RealType> runif;

        return param_.sigma_ * std::sqrt(-2 * std::log(runif(rng)));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        rayleigh_distribution(rng, n, r, sigma());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class RayleighDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void rayleigh_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType sigma)
{
    u01_oc_distribution(rng, n, r);
    log(n, r, r);
    mul(n, -2 * sigma * sigma, r, r);
    sqrt(n, r, r);
}

} // namespace vsmc::internal

/// \brief Generating rayleigh random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void rayleigh_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType sigma)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::rayleigh_distribution_impl(rng, k, r + i * k, sigma);
    internal::rayleigh_distribution_impl(rng, l, r + m * k, sigma);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, RayleighDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_RAYLEIGH_DISTRIBUTION_HPP
