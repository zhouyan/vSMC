//============================================================================
// vSMC/include/vsmc/rng/uniform_real_distribution.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a, b)   \
    VSMC_RUNTIME_ASSERT((a <= b), "**UniformRealDistribution** CONSTRUCTED "  \
                                  "WITH INVALID MINIMUM AND MAXIMUM "         \
                                  "PARAMTER VALUES")

namespace vsmc
{

/// \brief Uniform real distribution with open/closed variants
/// \ingroup Distribution
///
/// \tparam RealType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
template <typename RealType, typename Left, typename Right>
class UniformRealLRDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = UniformRealLRDistribution<RealType, Left, Right>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type =
            UniformRealLRDistribution<RealType, Left, Right>;

        param_type(result_type a, result_type b) : a_(a), b_(b)
        {
            invariant();
        }

        result_type a() const { return a_; }
        result_type b() const { return b_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!is_equal(param1.a_, param2.a_))
                return false;
            if (!is_equal(param1.b_, param2.b_))
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

            result_type a = 1;
            result_type b = 0;
            is >> std::ws >> a;
            is >> std::ws >> b;

            if (is.good()) {
                if (a <= b)
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
            VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(
                a_, b_);
        }

        void reset() {}
    }; // class param_type

    explicit UniformRealLRDistribution(result_type a = 0, result_type b = 1)
        : param_(a, b)
    {
    }

    explicit UniformRealLRDistribution(const param_type &param) : param_(param)
    {
    }

    result_type a() const { return param_.a(); }
    result_type b() const { return param_.b(); }

    result_type min VSMC_MNE() const { return a(); }
    result_type max VSMC_MNE() const { return b(); }

    void reset() {}

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return operator()(rng, param_);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const param_type &param)
    {
        U01LRDistribution<RealType, Left, Right> u01;

        return param.a() + (param.b() - param.a()) * u01(rng);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        return operator()(rng, n, r, param_);
    }

    template <typename RNGType>
    void operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &param)
    {
        uniform_real_lr_distribution<RealType, Left, Right>(
            rng, n, r, param.a(), param.b());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class UniformRealLRDistribution

/// \brief Uniform real distribution on cloed-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using UniformRealCCDistribution =
    UniformRealLRDistribution<RealType, Closed, Closed>;

/// \brief Uniform real distribution on cloed-open interval
/// \ingroup Distribution
template <typename RealType = double>
using UniformRealOODistribution =
    UniformRealLRDistribution<RealType, Open, Open>;

/// \brief Uniform real distribution on open-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using UniformRealCODistribution =
    UniformRealLRDistribution<RealType, Closed, Open>;

/// \brief Uniform real distribution on open-open interval
/// \ingroup Distribution
template <typename RealType = double>
using UniformRealOCDistribution =
    UniformRealLRDistribution<RealType, Open, Closed>;

/// \brief Uniform real distribution
template <typename RealType = double>
using UniformRealDistribution = UniformRealCODistribution<RealType>;

namespace internal
{

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void uniform_real_lr_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    u01_lr_distribution<RealType, Left, Right>(rng, n, r);
    fma(n, (b - a), r, a, r);
}

} // namespace vsmc::internal

/// \brief Generate uniform real random variates with open/closed variants
/// \ingroup Distribution
template <typename RealType, typename Left, typename Right, typename RNGType>
inline void uniform_real_lr_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i) {
        internal::uniform_real_lr_distribution_impl<RealType, Left, Right>(
            rng, k, r + i * k, a, b);
    }
    internal::uniform_real_lr_distribution_impl<RealType, Left, Right>(
        rng, l, r + m * k, a, b);
}

/// \brief Generate uniform real random variates on closed-closed interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void uniform_real_cc_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    uniform_real_lr_distribution<RealType, Closed, Closed>(rng, n, r, a, b);
}

/// \brief Generate uniform real random variates on closed-open interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void uniform_real_co_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    uniform_real_lr_distribution<RealType, Closed, Open>(rng, n, r, a, b);
}

/// \brief Generate uniform real random variates on open-closed interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void uniform_real_oc_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    uniform_real_lr_distribution<RealType, Open, Closed>(rng, n, r, a, b);
}

/// \brief Generate uniform real random variates on open-open interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void uniform_real_oo_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    uniform_real_lr_distribution<RealType, Open, Open>(rng, n, r, a, b);
}

/// \brief Generate uniform real random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void uniform_real_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    uniform_real_co_distribution(rng, n, r, a, b);
}

template <typename RealType, typename RNGType, typename Left, typename Right>
inline void rng_rand(RNGType &rng,
    UniformRealLRDistribution<RealType, Left, Right> &dist, std::size_t n,
    RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
