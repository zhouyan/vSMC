//============================================================================
// vSMC/include/vsmc/rng/pareto_distribution.hpp
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

#ifndef VSMC_RNG_PARETO_DISTRIBUTION_HPP
#define VSMC_RNG_PARETO_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/exponential_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_PARETO_DISTRIBUTION_PARAM_CHECK(b)            \
    VSMC_RUNTIME_ASSERT((b > 0), "**ParetoDistribution** CONSTRUCTED "        \
                                 "WITH INVALID SCALE PARAMETER VALUE")

namespace vsmc
{

/// \brief Pareto distribution
/// \ingroup Distribution
template <typename RealType>
class ParetoDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = ParetoDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = ParetoDistribution<RealType>;

        explicit param_type(result_type a = 1, result_type b = 1)
            : b_(b), exponential_(a)
        {
            invariant();
        }

        result_type a() const { return exponential_.lambda(); }
        result_type b() const { return b_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.b_, param2.b_))
                return false;
            if (param1.exponential_ != param2.exponential_)
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

            os << param.b_ << ' ';
            os << param.exponential_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type b = 0;
            ExponentialDistribution<RealType> exponential;
            is >> std::ws >> b;
            is >> std::ws >> exponential;

            if (is.good()) {
                if (b > 0) {
                    param.b_ = b;
                    param.exponential_ = exponential;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type b_;
        ExponentialDistribution<RealType> exponential_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_PARETO_DISTRIBUTION_PARAM_CHECK(b_);
        }

        void reset() {}
    }; // class param_type

    explicit ParetoDistribution(result_type a = 0, result_type b = 1)
        : param_(a, b)
    {
    }

    explicit ParetoDistribution(const param_type &param) : param_(param) {}

    result_type a() const { return param_.a(); }
    result_type b() const { return param_.b(); }

    result_type min() const { return a(); }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return param_.b_ * std::exp(param_.exponential_(rng));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        pareto_distribution(rng, n, r, a(), b());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class ParetoDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void pareto_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    exponential_distribution(rng, n, r, a);
    exp(n, r, r);
    mul(n, b, r, r);
}

} // namespace vsmc::internal

/// \brief Generating pareto random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void pareto_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::pareto_distribution_impl<k>(rng, k, r + i * k, a, b);
    internal::pareto_distribution_impl<k>(rng, l, r + m * k, a, b);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, ParetoDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_PARETO_DISTRIBUTION_HPP
