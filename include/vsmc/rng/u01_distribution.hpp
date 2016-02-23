//============================================================================
// vSMC/include/vsmc/rng/u01_distribution.hpp
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

#ifndef VSMC_RNG_U01_DISTRIBUTION_HPP
#define VSMC_RNG_U01_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.hpp>
#include <vsmc/rng/uniform_bits_distribution.hpp>

namespace vsmc
{

/// \brief Standard uniform distribution with open/closed variants
/// \ingroup Distribution
///
/// \tparam RealType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
template <typename RealType, typename Left, typename Right>
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
    }; // class param_type

    U01LRDistribution() {}
    explicit U01LRDistribution(const param_type &) {}

    result_type min VSMC_MNE() const { return 0; }
    result_type max VSMC_MNE() const { return 1; }

    void reset() {}

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        using uint_type =
            typename std::conditional<internal::RNGBits<RNGType>::value >= 64,
                std::uint64_t, std::uint32_t>::type;
        using flt_type = typename std::conditional<
            std::is_same<result_type, float>::value ||
                std::is_same<result_type, double>::value,
            result_type, double>::type;

        UniformBitsDistribution<uint_type> rbits;

        return U01<uint_type, flt_type, Left, Right>::eval(rbits(rng));
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const param_type &)
    {
        return operator()(rng);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        u01_lr_distribution<RealType, Left, Right>(rng, n, r);
    }

    template <typename RNGType>
    void operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &)
    {
        u01_lr_distribution<RealType, Left, Right>(rng, n, r);
    }

    friend bool operator==(const U01LRDistribution<RealType, Left, Right> &,
        const U01LRDistribution<RealType, Left, Right> &)
    {
        return true;
    }

    friend bool operator!=(const U01LRDistribution<RealType, Left, Right> &,
        const U01LRDistribution<RealType, Left, Right> &)
    {
        return false;
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const U01LRDistribution<RealType, Left, Right> &)
    {
        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        U01LRDistribution<RealType, Left, Right> &)
    {
        return is;
    }
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

template <std::size_t K, typename RealType, typename Left, typename Right,
    typename RNGType>
inline void u01_lr_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    uint32_t s[K];
    uniform_bits_distribution(rng, n, s);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = U01<std::uint32_t, RealType, Left, Right>::eval(s[i]);
}

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates with open/closed variants
/// \ingroup Distribution
template <typename RealType, typename Left, typename Right, typename RNGType>
inline void u01_lr_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k) {
        internal::u01_lr_distribution_impl<k, RealType, Left, Right>(
            rng, k, r);
    }
    internal::u01_lr_distribution_impl<k, RealType, Left, Right>(rng, l, r);
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
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
