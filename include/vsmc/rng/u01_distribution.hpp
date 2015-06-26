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

template <typename RealType, typename RNGType>
inline void u01_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::false_type, std::false_type)
{
    std::uniform_real_distribution<RealType> runif(0, 1);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = runif(rng);
}

template <typename RealType, typename RNGType>
inline void u01_distribution_impl_u32(
    RNGType &rng, std::size_t n, RealType *r, typename RNGType::result_type *s)
{
    rng_rand(rng, n, s);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = U01<std::uint32_t, RealType, Closed, Open>::eval(s[i]);
}

template <typename RealType, typename RNGType>
inline void u01_distribution_impl_u64(
    RNGType &rng, std::size_t n, RealType *r, typename RNGType::result_type *s)
{
    rng_rand(rng, n / 2, s);
    std::uint32_t *u = reinterpret_cast<std::uint32_t *>(s);
    if (n % 2 != 0)
        u[n - 1] = static_cast<std::uint32_t>(rng());
    for (std::size_t i = 0; i != n; ++i)
        r[i] = U01<std::uint64_t, RealType, Closed, Open>::eval(u[i]);
}

template <typename RealType, typename RNGType>
inline void u01_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::true_type, std::false_type)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    typename RNGType::result_type s[k];
    for (std::size_t i = 0; i != m; ++i)
        u01_distribution_impl_u32(rng, k, r + i * k, s);
    u01_distribution_impl_u32(rng, l, r + m * k, s);
}

template <typename RealType, typename RNGType>
inline void u01_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, std::true_type, std::true_type)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    typename RNGType::result_type s[k / 2 + 1];
    for (std::size_t i = 0; i != m; ++i)
        u01_distribution_impl_u64(rng, k, r + i * k, s);
    u01_distribution_impl_u64(rng, l, r + m * k, s);
}

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates
template <typename RealType, typename RNGType>
inline void u01_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    internal::u01_distribution_impl(
        rng, n, r, std::integral_constant<bool,
                       internal::RNGBits<RNGType>::value >= 32>(),
        std::integral_constant<bool,
            internal::RNGBits<RNGType>::value >= 64>());
}

namespace internal
{

template <int Bits, bool = Bits >= 32, bool = Bits >= 64>
class U01DistributionIntTypeTraitImpl;

template <int Bits>
class U01DistributionIntTypeTraitImpl<Bits, true, false>
{
    public:
    using type = std::uint32_t;
}; // class U01DistribuitonIntTypeTraitImpl

template <int Bits>
class U01DistributionIntTypeTraitImpl<Bits, true, true>
{
    public:
    using type = std::uint64_t;
}; // class U01DistribuitonIntTypeTraitImpl

template <typename RNGType>
class U01DistributionIntTypeTrait
    : public U01DistributionIntTypeTraitImpl<RNGBits<RNGType>::value>
{
}; // class U01TypeDistributionIntTypeTrait

template <typename RNGType>
using U01DistributionIntType =
    typename U01DistributionIntTypeTrait<RNGType>::type;

} // namespace vsmc::internal

/// \brief Standard uniform distribution with open/closed variants
/// \ingroup Distribution
///
/// \tparam RealType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
template <typename RealType = double, typename Left = Closed,
    typename Right = Open>
class U01Distribution
{
    public:
    using result_type = RealType;
    using distribution_type = U01Distribution<RealType, Left, Right>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = U01Distribution<RealType, Left, Right>;

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

    U01Distribution() {}

    explicit U01Distribution(const param_type &param) : param_(param) {}

    result_type min VSMC_MNE() const { return 0; }
    result_type max VSMC_MNE() const { return 1; }

    template <typename RNGType>
    result_type operator()(RNGType &rng) const
    {
        using uint_type = internal::U01DistributionIntType<RNGType>;

        return U01<uint_type, RealType, Left, Right>::eval(
            static_cast<uint_type>(rng()));
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class U01Distribution

namespace internal
{

template <typename RNGType, typename RealType,
    bool = RNGBits<RNGType>::value >= 32>
class U01DistributionTypeTraitImpl
{
    public:
    using type = U01Distribution<RealType, Closed, Open>;
}; // class UniformrealDistributionTypeTraitImpl

template <typename RNGType, typename RealType>
class U01DistributionTypeTraitImpl<RNGType, RealType, false>
{
    public:
    class type : public std::uniform_real_distribution<RealType>
    {
        using base = std::uniform_real_distribution<RealType>;

        public:
        using param_type = typename base::param_type;

        type() : base(0, 1) {}

        type(const param_type &) : base(0, 1) {}

        param_type param() const { return base::param(); }
        void param(const param_type &) {}
    }; // class type
};     // class UniformrealDistributionTypeTraitImpl

} // namespace vsmc::internal

/// \brief Standard uniform distribution type trait
/// \ingroup Distribution
template <typename RNGType, typename RealType = double>
class U01DistributionTypeTrait
{
    public:
    using type = typename internal::U01DistributionTypeTraitImpl<RNGType,
        RealType>::type;
}; // class U01DistributionTypeTrait

/// \brief Standard uniform distribution type
/// \ingroup Distribution
template <typename RNGType, typename RealType = double>
using U01DistributionType =
    typename U01DistributionTypeTrait<RNGType, RealType>::type;

/// \brief Standard uniform distribution on cloed-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using U01CCDistribution = U01Distribution<RealType, Closed, Closed>;

/// \brief Standard uniform distribution on cloed-open interval
/// \ingroup Distribution
template <typename RealType = double>
using U01OODistribution = U01Distribution<RealType, Open, Open>;

/// \brief Standard uniform distribution on open-closed interval
/// \ingroup Distribution
template <typename RealType = double>
using U01CODistribution = U01Distribution<RealType, Closed, Open>;

/// \brief Standard uniform distribution on open-open interval
/// \ingroup Distribution
template <typename RealType = double>
using U01OCDistribution = U01Distribution<RealType, Open, Closed>;

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
