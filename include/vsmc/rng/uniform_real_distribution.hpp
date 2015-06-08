//============================================================================
// vSMC/include/vsmc/rng/uniform_real_distribution.hpp
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

#ifndef VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a, b)   \
    VSMC_RUNTIME_ASSERT((a <= b),                                             \
        "**UniformRealDistribution** CONSTRUCTED WITH INVALID "               \
        "MINIMUM AND MAXIMUM PARAMTER VALUES")

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(eng_min)    \
    VSMC_RUNTIME_ASSERT((eng_min == 0),                                       \
        "**UniformRealDistribution::operator()** "                            \
        "ENGINE MEMBER FUNCTION min() RETURN A VALUE OTHER THAN ZERO")

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX(eng_max)    \
    VSMC_RUNTIME_ASSERT(                                                      \
        (eng_max == uint32_t_max_ || eng_max == uint64_t_max_),               \
        "**UniformRealDistribution::operator()** "                            \
        "ENGINE MEMBER FUNCTION max() RETURN A VALUE OTHER THAN "             \
        "THE MAXIMUM OF std::uint32_t OR std::uint64_t")

namespace vsmc
{

namespace internal
{

template <std::uint64_t, std::uint64_t>
class UniformRealDistributionIntType;

template <>
class UniformRealDistributionIntType<0,
    static_cast<std::uint64_t>(VSMC_MAX_UINT(std::uint32_t))>
{
    public:
    using type = std::uint32_t;
}; // class UniformRealDistributionIntType

template <>
class UniformRealDistributionIntType<0, VSMC_MAX_UINT(std::uint64_t)>
{
    public:
    using type = std::uint64_t;
}; // class UniformRealDistributionIntType

} // namespace vsmc::interal

/// \brief Uniform real distribution with variants open/closed variants
/// \ingroup Distribution
///
/// \details
/// This distribution is almost identical to C++11
/// `std::uniform_real_distribution`. But it differs in two important aspects
/// - It allows the interval to be either open or closed on both sides.
/// - It requires that the uniform random number generator to produce integers
///   on the full range of either `std::uint32_t` or `std::uint64_t`.
/// \tparam RealType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
template <typename RealType = double, typename Left = Closed,
    typename Right = Open>
class UniformRealDistribution
{
    public:
    using result_type = RealType;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type =
            UniformRealDistribution<RealType, Left, Right>;

        explicit param_type(result_type a = 0, result_type b = 1)
            : a_(a), b_(b)
        {
        }

        result_type a() const { return a_; }
        result_type b() const { return b_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.a_ < param2.a_ || param1.a_ > param2.a_)
                return false;
            if (param1.b_ < param2.b_ || param1.b_ > param2.b_)
                return false;
            return true;
        }

        friend bool operator!=(
            const param_type param1, const param_type param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.a_ << ' ' << param.b_;

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
                if (a <= b) {
                    param.a_ = a;
                    param.b_ = b;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type a_;
        result_type b_;
    }; // class param_type

    explicit UniformRealDistribution(result_type a = 0, result_type b = 1)
        : a_(a), b_(b)
    {
        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a_, b_);
    }

    explicit UniformRealDistribution(const param_type &param)
        : a_(param.a()), b_(param.b())
    {
        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a_, b_);
    }

    param_type param() const { return param_type(a_, b_); }

    void param(const param_type &param)
    {
        a_ = param.a();
        b_ = param.b();
    }

    void reset() const {}

    result_type a() const { return a_; }
    result_type b() const { return b_; }
    result_type min VSMC_MNE() const { return a_; }
    result_type max VSMC_MNE() const { return b_; }

    /// \brief Generate uniform random variates
    ///
    /// \tparam Eng Requirement:
    /// ~~~{.cpp}
    /// Eng::min() == 0 && (
    /// Eng::max() == std::numeric_limits<std::uint32_t>::max() ||
    /// Eng::max() == std::numeric_limits<std::uint64_t>::max()
    /// )
    /// ~~~
    /// and both `min` and `max` are `constexpr`
    template <typename Eng>
    result_type operator()(Eng &eng) const
    {
        return U01<Left, Right,
                   typename internal::UniformRealDistributionIntType<
                       Eng::min VSMC_MNE(), Eng::max VSMC_MNE()>::type,
                   RealType>::uint2fp(eng()) *
            (b_ - a_) +
            a_;
    }

    friend bool operator==(
        const UniformRealDistribution<RealType, Left, Right> &runif1,
        const UniformRealDistribution<RealType, Left, Right> &runif2)
    {
        if (runif1.a_ < runif2.a_ || runif1.a_ > runif1.a_)
            return false;
        if (runif1.b_ < runif2.b_ || runif1.b_ > runif1.b_)
            return false;
        return true;
    }

    friend bool operator!=(
        const UniformRealDistribution<RealType, Left, Right> &runif1,
        const UniformRealDistribution<RealType, Left, Right> &runif2)
    {
        return !(runif1 == runif2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const UniformRealDistribution<RealType, Left, Right> &runif)
    {
        if (!os.good())
            return os;

        os << runif.a_ << ' ' << runif.b_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        UniformRealDistribution<RealType, Left, Right> &runif)
    {
        if (!is.good())
            return is;

        result_type a = 1;
        result_type b = 0;
        is >> std::ws >> a;
        is >> std::ws >> b;
        if (is.good()) {
            if (a <= b) {
                runif.a_ = a;
                runif.b_ = b;
            } else {
                is.setstate(std::ios_base::failbit);
            }
        }

        return is;
    }

    private:
    result_type a_;
    result_type b_;
}; // class UniformRealDistribution

template <typename RealType = double>
using UniformRealOpenOpenDistribution =
    UniformRealDistribution<RealType, Open, Open>;

template <typename RealType = double>
using UniformRealOpenClosedDistribution =
    UniformRealDistribution<RealType, Open, Closed>;

template <typename RealType = double>
using UniformRealClosedOpenDistribution =
    UniformRealDistribution<RealType, Closed, Open>;

template <typename RealType = double>
using UniformRealClosedClosedDistribution =
    UniformRealDistribution<RealType, Closed, Closed>;

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
