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
class U01ImplPow2L
{
    public:
    static constexpr long double value =
        static_cast<long double>(1ULL << Q) * U01ImplPow2L<P - Q>::value;
}; // class U01ImplPow2L

template <int P, int Q>
class U01ImplPow2L<P, Q, false>
{
    public:
    static constexpr long double value = static_cast<long double>(1ULL << P);
}; // class U01ImplPow2L

template <int P>
class U01ImplPow2InvL
{
    public:
    static constexpr long double value = 1.0L / U01ImplPow2L<P>::value;
}; // class U01ImplPow2InvL

template <typename RealType, int P>
class U01ImplPow2Inv
{
    public:
    static constexpr RealType value =
        static_cast<RealType>(U01ImplPow2InvL<P>::value);
}; // class U01ImplPow2Inv

template <typename, typename, typename, typename>
class U01FixedPointImpl;

template <typename UIntType, typename RealType>
class U01FixedPointImpl<UIntType, RealType, Closed, Closed>
{
    public:
    static RealType eval(UIntType u)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int m = std::numeric_limits<RealType>::digits;
        static constexpr int p = w - 1 < m ? w - 1 : m;
        static constexpr int v = p + 1;
        static constexpr int l = v < w ? 1 : 0;
        static constexpr int r = v < w ? w - 1 - v : 0;

        return trans((u << l) >> (r + l),
                   std::integral_constant<bool, (v < w)>()) *
            U01ImplPow2Inv<RealType, p + 1>::value;
    }

    private:
    static RealType trans(UIntType u, std::true_type)
    {
        return static_cast<RealType>((u & 1) + u);
    }

    static RealType trans(UIntType u, std::false_type)
    {
        return static_cast<RealType>(u & 1) + static_cast<RealType>(u);
    }
}; // class U01FixedPointImpl

template <typename UIntType, typename RealType>
class U01FixedPointImpl<UIntType, RealType, Closed, Open>
{
    public:
    static RealType eval(UIntType u)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int m = std::numeric_limits<RealType>::digits;
        static constexpr int p = w < m ? w : m;
        static constexpr int r = w - p;

        return static_cast<RealType>(u >> r) *
            U01ImplPow2Inv<RealType, p>::value;
    }
}; // class U01FixedPointImpl

template <typename UIntType, typename RealType>
class U01FixedPointImpl<UIntType, RealType, Open, Closed>
{
    public:
    static RealType eval(UIntType u)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int m = std::numeric_limits<RealType>::digits;
        static constexpr int p = w < m ? w : m;
        static constexpr int r = w - p;

        return static_cast<RealType>(u >> r) *
            U01ImplPow2Inv<RealType, p>::value +
            U01ImplPow2Inv<RealType, p>::value;
    }
}; // class U01FixedPointImpl

template <typename UIntType, typename RealType>
class U01FixedPointImpl<UIntType, RealType, Open, Open>
{
    public:
    static RealType eval(UIntType u)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int m = std::numeric_limits<RealType>::digits;
        static constexpr int p = w + 1 < m ? w + 1 : m;
        static constexpr int r = w + 1 - p;

        return static_cast<RealType>(u >> r) *
            U01ImplPow2Inv<RealType, p - 1>::value +
            U01ImplPow2Inv<RealType, p>::value;
    }
}; // class U01FixedPointImpl

} // namespace vsmc::internal

template <typename UIntType, typename RealType, typename Left, typename Right>
class U01FixedPoint
    : public internal::U01FixedPointImpl<UIntType, RealType, Left, Right>
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**U01FixedPoint** USED WITH UIntType OTHER THAN UNSIGNED INTEGER "
        "TYPES");
    static_assert(std::is_floating_point<RealType>::value,
        "**U01FixedPoint** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");
}; // class U01FixedPoint

template <typename UIntType, typename RealType>
class U01
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**U01** USED WITH UIntType OTHER THAN UNSIGNED INTEGER TYPES");
    static_assert(std::is_floating_point<RealType>::value,
        "**U01** USED WITH RealType OTHER THAN FLOATING POINT TYPES");

    public:
    static RealType eval(UIntType u)
    {
        static constexpr int p = std::numeric_limits<UIntType>::digits;

        return static_cast<RealType>(u) *
            internal::U01ImplPow2Inv<RealType, p>::value +
            internal::U01ImplPow2Inv<RealType, p + 1>::value;
    }
}; // class U01

} // namespace vsmc

#endif //  VSMC_RNG_U01_HPP
