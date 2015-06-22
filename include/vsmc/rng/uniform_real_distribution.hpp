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
    VSMC_RUNTIME_ASSERT((a <= b), "**UniformRealDistribution** CONSTRUCTED "  \
                                  "WITH INVALID MINIMUM AND MAXIMUM "         \
                                  "PARAMTER VALUES")

namespace vsmc
{

namespace internal
{

template <int Bits, bool = Bits >= 32, bool = Bits >= 64>
class UniformRealDistributionIntTypeTraitImpl;

template <int Bits>
class UniformRealDistributionIntTypeTraitImpl<Bits, true, false>
{
    public:
    using type = std::uint32_t;
}; // class UniformRealDistribuitonIntTypeTraitImpl

template <int Bits>
class UniformRealDistributionIntTypeTraitImpl<Bits, true, true>
{
    public:
    using type = std::uint64_t;
}; // class UniformRealDistribuitonIntTypeTraitImpl

template <typename RNGType>
class UniformRealDistributionIntTypeTrait
    : public UniformRealDistributionIntTypeTraitImpl<RNGBits<RNGType>::value>
{
}; // class UniformRealTypeDistributionIntTypeTrait

template <typename RNGType>
using UniformRealDistributionIntType =
    typename UniformRealDistributionIntTypeTrait<RNGType>::type;

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
    using distribution_type = UniformRealDistribution<RealType, Left, Right>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type =
            UniformRealDistribution<RealType, Left, Right>;

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
            if (!is_equal(param1.a_, param2.a_))
                return false;
            if (!is_equal(param1.b_, param2.b_))
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

    explicit UniformRealDistribution(result_type a = 0, result_type b = 1)
        : param_(a, b)
    {
    }

    explicit UniformRealDistribution(const param_type &param) : param_(param)
    {
    }

    result_type a() const { return param_.a_; }
    result_type b() const { return param_.b_; }

    result_type min VSMC_MNE() const { return param_.a_; }
    result_type max VSMC_MNE() const { return param_.b_; }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        using int_type = internal::UniformRealDistributionIntType<RNGType>;
        return U01<Left, Right, int_type, RealType>::uint2fp(
                   static_cast<int_type>(rng())) *
            (param_.b_ - param_.a_) +
            param_.a_;
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class UniformRealDistribution

namespace internal
{

template <typename RNGType, typename RealType,
    bool = RNGBits<RNGType>::value >= 32>
class UniformRealDistributionTypeTraitImpl
{
    public:
    using type = UniformRealDistribution<RealType, Closed, Open>;
}; // class UniformrealDistributionTypeTraitImpl

template <typename RNGType, typename RealType>
class UniformRealDistributionTypeTraitImpl<RNGType, RealType, false>
{
    public:
    using type = std::uniform_real_distribution<RealType>;
}; // class UniformrealDistributionTypeTraitImpl

} // namespace vsmc::internal

template <typename RNGType, typename RealType = double>
class UniformRealDistributionTypeTrait
{
    public:
    using type =
        typename internal::UniformRealDistributionTypeTraitImpl<RNGType,
            RealType>::type;
}; // class UniformRealDistributionTypeTrait

template <typename RNGType, typename RealType = double>
using UniformRealDistributionType =
    typename UniformRealDistributionTypeTrait<RNGType, RealType>::type;

template <typename RealType = double>
using UniformRealCCDistribution =
    UniformRealDistribution<RealType, Closed, Closed>;

template <typename RealType = double>
using UniformRealOODistribution =
    UniformRealDistribution<RealType, Open, Open>;

template <typename RealType = double>
using UniformRealCODistribution =
    UniformRealDistribution<RealType, Closed, Open>;

template <typename RealType = double>
using UniformRealOCDistribution =
    UniformRealDistribution<RealType, Open, Closed>;

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
