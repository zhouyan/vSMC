//============================================================================
// vSMC/include/vsmc/rng/exponential_distribution.hpp
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

#ifndef VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP
#define VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_EXPONENTIAL_DISTRIBUTION_PARAM_CHECK(coeff)   \
    VSMC_RUNTIME_ASSERT((coeff < 0), "**ExponentialDistribution** "           \
                                     "CONSTRUCTED WITH INVALID RATE "         \
                                     "PARAMETER VALUE")

namespace vsmc
{

template <typename RealType, typename RNGType>
inline void exponential_distribution(
    RNGType &, std::size_t, RealType *, RealType);

/// \brief Exponential distribution
/// \ingroup Distribution
template <typename RealType>
class ExponentialDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = ExponentialDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = ExponentialDistribution<RealType>;

        explicit param_type(result_type lambda = 1) : coeff_(-1 / lambda)
        {
            invariant();
        }

        result_type lambda() const { return -1 / coeff_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            return internal::is_equal(param1.coeff_, param2.coeff_);
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

            os << param.coeff_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type coeff = 0;
            is >> std::ws >> coeff;

            if (is.good()) {
                if (coeff < 0)
                    param.coeff_ = coeff;
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        result_type coeff_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_EXPONENTIAL_DISTRIBUTION_PARAM_CHECK(
                coeff_);
        }

        void reset() {}
    }; // class param_type

    explicit ExponentialDistribution(result_type lambda = 1) : param_(lambda)
    {
    }

    explicit ExponentialDistribution(const param_type &param) : param_(param)
    {
    }

    result_type lambda() const { return param_.lambda(); }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        U01OCDistribution<RealType> runif;

        return param_.coeff_ * std::log(runif(rng));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        exponential_distribution(rng, n, r, lambda());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class ExponentialDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void exponential_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType lambda)
{
    u01_oc_distribution(rng, n, r);
    log(n, r, r);
    mul(n, -1 / lambda, r, r);
}

} // namespace vsmc::internal

/// \brief Generating exponential random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void exponential_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType lambda)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::exponential_distribution_impl(rng, k, r + i * k, lambda);
    internal::exponential_distribution_impl(rng, l, r + m * k, lambda);
}

} // namespace vsmc

#endif // VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP
