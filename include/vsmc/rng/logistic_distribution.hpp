//============================================================================
// vSMC/include/vsmc/rng/logistic_distribution.hpp
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

#ifndef VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP
#define VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_LOGISTIC_DISTRIBUTION_PARAM_CHECK(b)          \
    VSMC_RUNTIME_ASSERT((b > 0), "**LogisticDistribution** CONSTRUCTED "      \
                                 "WITH INVALID SCALE PARAMETER VALUE")

namespace vsmc
{

/// \brief Logistic distribution
/// \ingroup Distribution
template <typename RealType>
class LogisticDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = LogisticDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = LogisticDistribution<RealType>;

        explicit param_type(result_type a = 0, result_type b = 1)
            : a_(a), b_(b)
        {
            invariant();
        }

        result_type a() const { return a_; }
        result_type b() const { return b_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.a_, param2.a_))
                return false;
            if (!internal::is_equal(param1.b_, param2.b_))
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

            os << param.a_ << ' ';
            os << param.b_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type a = 0;
            result_type b = 0;
            is >> std::ws >> a;
            is >> std::ws >> b;

            if (is.good()) {
                if (b > 0)
                    param = param_type(a, b);
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        result_type a_;
        result_type b_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_LOGISTIC_DISTRIBUTION_PARAM_CHECK(b_);
        }

        void reset() {}
    }; // class param_type

    explicit LogisticDistribution(result_type a = 0, result_type b = 1)
        : param_(a, b)
    {
    }

    explicit LogisticDistribution(const param_type &param) : param_(param) {}

    result_type a() const { return param_.a(); }
    result_type b() const { return param_.b(); }

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
        U01OODistribution<RealType> runif;
        result_type u = runif(rng);

        return param_.a_ + param_.b_ * std::log(u / (1 - u));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        logistic_distribution(rng, n, r, a(), b());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class LogisticDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void logistic_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    RealType s[K];
    u01_oo_distribution(rng, n, r);
    sub(n, static_cast<RealType>(1), r, s);
    div(n, r, s, r);
    log(n, r, r);
    fma(n, a, b, r, r);
}

} // namespace vsmc::internal

/// \brief Generating logistic random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void logistic_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::logistic_distribution_impl<k>(rng, k, r + i * k, a, b);
    internal::logistic_distribution_impl<k>(rng, l, r + m * k, a, b);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, LogisticDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP
