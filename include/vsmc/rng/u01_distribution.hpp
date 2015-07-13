//============================================================================
// vSMC/include/vsmc/rng/u01_distribution.hpp
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

#ifndef VSMC_RNG_U01_DISTRIBUTION_HPP
#define VSMC_RNG_U01_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.hpp>

namespace vsmc
{

namespace internal
{

template <typename UIntType, int Bits>
class U01LRDistributionBits
{
    public:
    template <typename RNGType>
    static UIntType eval(RNGType &rng)
    {
        return eval(rng,
            std::integral_constant<bool, RNGMinBits<RNGType>::value == 0>(),
            std::integral_constant<bool, RNGBits<RNGType>::value >= Bits>());
    }

    private:
    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::true_type, std::true_type)
    {
        return static_cast<UIntType>(rng());
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::false_type, std::true_type)
    {
        return static_cast<UIntType>(rng() >> RNGMinBits<RNGType>::value);
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::true_type, std::false_type)
    {
        return static_cast<UIntType>(
            patch<0, RNGBits<RNGType>::value, RNGMinBits<RNGType>::value>(
                rng, std::true_type()));
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::false_type, std::false_type)
    {
        return eval(rng, std::true_type(), std::false_type());
    }

    template <int, int, int, typename RNGType>
    static UIntType patch(RNGType &, std::false_type)
    {
        return 0;
    }

    template <int N, int B, int R, typename RNGType>
    static UIntType patch(RNGType &rng, std::true_type)
    {
        return static_cast<UIntType>((rng() >> R) << (B * N)) +
            patch<N + 1, B, R>(
                   rng, std::integral_constant<bool, (N * B + B) < Bits>());
    }
}; // class U01DistributionBits

} // namespace vsmc::internal

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution(RNGType &, std::size_t, RealType *);

/// \brief Standard uniform distribution with open/closed variants
/// \ingroup Distribution
///
/// \tparam RealType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
template <typename RealType = double, typename Left = Closed,
    typename Right = Open>
class U01LRDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = U01LRDistribution<RealType, Left, Right>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = U01LRDistribution<RealType, Left, Right>;

        param_type() { invariant(); }

        friend bool operator==(const param_type &, const param_type &)
        {
            return true;
        }

        friend bool operator!=(const param_type &, const param_type &)
        {
            return false;
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &)
        {
            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &)
        {
            return is;
        }

        private:
        friend distribution_type;

        void invariant() {}

        void reset() {}
    }; // class param_type

    U01LRDistribution() {}

    explicit U01LRDistribution(const param_type &param) : param_(param) {}

    result_type min() const { return 0; }
    result_type max() const { return 1; }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        using uint_type =
            typename std::conditional<internal::RNGBits<RNGType>::value >= 64,
                std::uint64_t, std::uint32_t>::type;
        using bits_type =
            typename std::conditional<internal::RNGBits<RNGType>::value >= 64,
                std::integral_constant<int, 64>,
                std::integral_constant<int, 32>>::type;
        using flt_type = typename std::conditional<
            std::is_same<result_type, float>::value ||
                std::is_same<result_type, double>::value,
            result_type, double>::type;

        return U01<uint_type, flt_type, Left, Right>::eval(
            internal::U01LRDistributionBits<uint_type, bits_type::value>::eval(
                rng));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        u01_lr_distribution<RealType, Left, Right>(rng, n, r);
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class U01LRDistribution

/// \brief Standard uniform distribution on cloed-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using U01CCDistribution = U01LRDistribution<RealType, Closed, Closed>;

/// \brief Standard uniform distribution on cloed-open interval
/// \ingroup Distribution
template <typename RealType = double>
using U01OODistribution = U01LRDistribution<RealType, Open, Open>;

/// \brief Standard uniform distribution on open-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using U01CODistribution = U01LRDistribution<RealType, Closed, Open>;

/// \brief Standard uniform distribution on open-open interval
/// \ingroup Distribution
template <typename RealType = double>
using U01OCDistribution = U01LRDistribution<RealType, Open, Closed>;

/// \brief Standard uniform distribution
/// \ingroup Distribution
template <typename RealType = double>
using U01Distribution = U01CODistribution<RealType>;

namespace internal
{

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::false_type, std::false_type)
{
    U01LRDistribution<RealType, Left, Right> runif;
    for (std::size_t i = 0; i != n; ++i)
        r[i] = runif(rng);
}

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution_impl_u32(
    RNGType &rng, std::size_t n, RealType *r, typename RNGType::result_type *s)
{
    rng_rand(rng, n, s);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = U01<std::uint32_t, RealType, Left, Right>::eval(s[i]);
}

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution_impl_u64(
    RNGType &rng, std::size_t n, RealType *r, typename RNGType::result_type *s)
{
    rng_rand(rng, n / 2, s);
    std::uint32_t *u = reinterpret_cast<std::uint32_t *>(s);
    if (n % 2 != 0)
        u[n - 1] = static_cast<std::uint32_t>(rng());
    for (std::size_t i = 0; i != n; ++i)
        r[i] = U01<std::uint32_t, RealType, Left, Right>::eval(u[i]);
}

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::true_type, std::false_type)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    typename RNGType::result_type s[k];
    for (std::size_t i = 0; i != m; ++i) {
        u01_lr_distribution_impl_u32<RealType, Left, Right>(
            rng, k, r + i * k, s);
    }
    u01_lr_distribution_impl_u32<RealType, Left, Right>(rng, l, r + m * k, s);
}

template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::true_type, std::true_type)
{
    const std::size_t k = 2000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    typename RNGType::result_type s[k / 2];
    for (std::size_t i = 0; i != m; ++i) {
        u01_lr_distribution_impl_u64<RealType, Left, Right>(
            rng, k, r + i * k, s);
    }
    u01_lr_distribution_impl_u64<RealType, Left, Right>(rng, l, r + m * k, s);
}

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates with open/closed variants
/// \ingroup Distribution
template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    internal::u01_lr_distribution_impl<RealType, Left, Right>(rng, n, r,
        std::integral_constant < bool,
        internal::RNGMinBits<RNGType>::value == 0 &&
            internal::RNGMaxBits<RNGType>::value >= 32 > (),
        std::integral_constant < bool,
        internal::RNGMinBits<RNGType>::value == 0 &&
            internal::RNGMaxBits<RNGType>::value >= 64 > ());
}

/// \brief Generate standard uniform random variates on closed-closed interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_cc_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    u01_lr_distribution<RealType, Closed, Closed>(rng, n, r);
}

/// \brief Generate standard uniform random variates on closed-open interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_co_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    u01_lr_distribution<RealType, Closed, Open>(rng, n, r);
}

/// \brief Generate standard uniform random variates on open-closed interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_oc_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    u01_lr_distribution<RealType, Open, Closed>(rng, n, r);
}

/// \brief Generate standard uniform random variates on open-open interval
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_oo_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    u01_lr_distribution<RealType, Open, Open>(rng, n, r);
}

/// \brief Generate standard uniform random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    u01_co_distribution(rng, n, r);
}

template <typename RealType, typename RNGType, typename Left, typename Right>
inline void rng_rand(RNGType &rng, U01LRDistribution<RealType, Left, Right> &,
    std::size_t n, RealType *r)
{
    u01_lr_distribution<RealType, Left, Right>(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
