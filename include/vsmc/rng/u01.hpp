//============================================================================
// vSMC/include/vsmc/rng/u01.hpp
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

#ifndef VSMC_RNG_U01_HPP
#define VSMC_RNG_U01_HPP

#include <vsmc/rng/internal/common.hpp>

namespace vsmc
{

namespace internal
{

template <int P,
    int Q = (std::numeric_limits<unsigned long long>::digits <
                        std::numeric_limits<long double>::digits ?
                    std::numeric_limits<unsigned long long>::digits :
                    std::numeric_limits<long double>::digits) -
        1,
    bool = (Q < P)>
class U01Pow2L
{
    public:
    static constexpr long double value =
        static_cast<long double>(1ULL << Q) * U01Pow2L<P - Q>::value;
}; // class U01Pow2L

template <int P, int Q>
class U01Pow2L<P, Q, false>
{
    public:
    static constexpr long double value = static_cast<long double>(1ULL << P);
}; // class U01Pow2L

template <int P>
class U01Pow2InvL
{
    public:
    static constexpr long double value = 1.0L / U01Pow2L<P>::value;
}; // class U01Pow2InvL

template <typename RealType, int P>
class U01Pow2
{
    public:
    static constexpr RealType value =
        static_cast<RealType>(U01Pow2L<P>::value);
}; // class U01Pow2

template <typename RealType, int P>
class U01Pow2Inv
{
    public:
    static constexpr RealType value =
        static_cast<RealType>(U01Pow2InvL<P>::value);
}; // class U01Pow2Inv

template <typename, typename, typename, typename>
class U01Impl;

template <typename UIntType, typename RealType>
class U01Impl<UIntType, RealType, Closed, Closed>
{
    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = W - 1 < M ? W - 1 : M;
    static constexpr int V = P + 1;
    static constexpr int L = V < W ? 1 : 0;
    static constexpr int R = V < W ? W - 1 - V : 0;

    public:
    static RealType eval(UIntType u) noexcept
    {
        return trans((u << L) >> (R + L),
                   std::integral_constant<bool, (V < W)>()) *
            U01Pow2Inv<RealType, P + 1>::value;
    }

    static void eval(std::size_t n, const UIntType *u, RealType *r) noexcept
    {
        for (std::size_t i = 0; i != n; ++i) {
            r[i] = trans((u[i] << L) >> (R + L),
                std::integral_constant<bool, (V < W)>());
        }
        mul(n, U01Pow2Inv<RealType, P + 1>::value, r, r);
    }

    private:
    static RealType trans(UIntType u, std::true_type) noexcept
    {
        return static_cast<RealType>((u & 1) + u);
    }

    static RealType trans(UIntType u, std::false_type) noexcept
    {
        return static_cast<RealType>(u & 1) + static_cast<RealType>(u);
    }
}; // class U01Impl

template <typename UIntType, typename RealType>
class U01Impl<UIntType, RealType, Closed, Open>
{
    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = W < M ? W : M;
    static constexpr int R = W - P;

    public:
    static RealType eval(UIntType u) noexcept
    {
        return static_cast<RealType>(u >> R) * U01Pow2Inv<RealType, P>::value;
    }

    static void eval(std::size_t n, const UIntType *u, RealType *r) noexcept
    {
        for (std::size_t i = 0; i != n; ++i)
            r[i] = static_cast<RealType>(u[i] >> R);
        mul(n, U01Pow2Inv<RealType, P>::value, r, r);
    }
}; // class U01Impl

template <typename UIntType, typename RealType>
class U01Impl<UIntType, RealType, Open, Closed>
{
    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = W < M ? W : M;
    static constexpr int R = W - P;

    public:
    static RealType eval(UIntType u) noexcept
    {
        return static_cast<RealType>(u >> R) * U01Pow2Inv<RealType, P>::value +
            U01Pow2Inv<RealType, P>::value;
    }

    static void eval(std::size_t n, const UIntType *u, RealType *r) noexcept
    {
        for (std::size_t i = 0; i != n; ++i)
            r[i] = static_cast<RealType>(u[i] >> R);
        fma(n, U01Pow2Inv<RealType, P>::value, r,
            U01Pow2Inv<RealType, P>::value, r);
    }
}; // class U01Impl

template <typename UIntType, typename RealType>
class U01Impl<UIntType, RealType, Open, Open>
{
    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = W + 1 < M ? W + 1 : M;
    static constexpr int R = W + 1 - P;

    public:
    static RealType eval(UIntType u) noexcept
    {
        return static_cast<RealType>(u >> R) *
            U01Pow2Inv<RealType, P - 1>::value +
            U01Pow2Inv<RealType, P>::value;
    }

    static void eval(std::size_t n, const UIntType *u, RealType *r) noexcept
    {
        for (std::size_t i = 0; i != n; ++i)
            r[i] = static_cast<RealType>(u[i] >> R);
        fma(n, U01Pow2Inv<RealType, P - 1>::value, r,
            U01Pow2Inv<RealType, P>::value, r);
    }
}; // class U01Impl

} // namespace vsmc::internal

/// \brief Convert uniform unsigned integers to floating points within [0, 1]
/// \ingroup RNG
///
/// \details
/// Let \f$W\f$ be the number of digits of unsigned integer type `UIntType`.
/// Let \f$M\f$ be the number of significant digits of floating point type
/// `RealType`. Assuming the input is a uniform random number on the set
/// \f$\{0,1,\dots,2^W - 1\f$, the output is uniform over the interval
/// \f$[0,1]\f$ or one of its (half-)open interval variant. The exact output
/// depend on the template parameter `Lower` and `Upper`.
template <typename UIntType, typename RealType, typename Lower, typename Upper>
inline RealType u01(UIntType u) noexcept
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**u01** used with UIntType other than unsigned integer "
        "types");

    static_assert(std::is_floating_point<RealType>::value,
        "**u01** used with RealType other than floating point "
        "types");

    return internal::U01Impl<UIntType, RealType, Lower, Upper>::eval(u);
}

/// \brief Convert uniform unsigned integers to floating points within [0, 1]
/// \ingroup RNG
template <typename UIntType, typename RealType, typename Lower, typename Upper>
inline void u01(std::size_t n, const UIntType *u, RealType *r) noexcept
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**u01** used with UIntType other than unsigned integer "
        "types");

    static_assert(std::is_floating_point<RealType>::value,
        "**u01** used with RealType other than floating point "
        "types");

    internal::U01Impl<UIntType, RealType, Lower, Upper>::eval(n, u, r);
}

/// \brief Convert uniform unsigned integers to floating points on [0, 1]
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline RealType u01_cc(UIntType u) noexcept
{
    return u01<UIntType, RealType, Closed, Closed>(u);
}

/// \brief Convert uniform unsigned integers to floating points on [0, 1)
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline RealType u01_co(UIntType u) noexcept
{
    return u01<UIntType, RealType, Closed, Open>(u);
}

/// \brief Convert uniform unsigned integers to floating points on (0, 1]
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline RealType u01_oc(UIntType u) noexcept
{
    return u01<UIntType, RealType, Open, Closed>(u);
}

/// \brief Convert uniform unsigned integers to floating points on (0, 1)
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline RealType u01_oo(UIntType u) noexcept
{
    return u01<UIntType, RealType, Open, Open>(u);
}

/// \brief Convert uniform unsigned integers to floating points on [0, 1]
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline void u01_cc(std::size_t n, const UIntType *u, RealType *r) noexcept
{
    u01<UIntType, RealType, Closed, Closed>(n, u, r);
}

/// \brief Convert uniform unsigned integers to floating points on [0, 1)
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline void u01_co(std::size_t n, const UIntType *u, RealType *r) noexcept
{
    u01<UIntType, RealType, Closed, Open>(n, u, r);
}

/// \brief Convert uniform unsigned integers to floating points on (0, 1]
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline void u01_oc(std::size_t n, const UIntType *u, RealType *r) noexcept
{
    u01<UIntType, RealType, Open, Closed>(n, u, r);
}

/// \brief Convert uniform unsigned integers to floating points on (0, 1)
/// \ingroup RNG
template <typename UIntType, typename RealType>
inline void u01_oo(std::size_t n, const UIntType *u, RealType *r) noexcept
{
    u01<UIntType, RealType, Open, Open>(n, u, r);
}

} // namespace vsmc

#endif //  VSMC_RNG_U01_HPP
