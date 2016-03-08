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

template <int Bits,
    int RBits = (std::numeric_limits<unsigned long long>::digits <
                            std::numeric_limits<long double>::digits ?
                        std::numeric_limits<unsigned long long>::digits :
                        std::numeric_limits<long double>::digits) -
        1,
    bool = (RBits < Bits)>
class U01ImplPow2BitsL
{
    public:
    static constexpr long double value =
        static_cast<long double>(1ULL << RBits) *
        U01ImplPow2BitsL<Bits - RBits>::value;
}; // class U01ImplPow2BitsL

template <int Bits, int RBits>
class U01ImplPow2BitsL<Bits, RBits, false>
{
    public:
    static constexpr long double value =
        static_cast<long double>(1ULL << Bits);
}; // class U01ImplPow2BitsL

template <int Bits>
class U01ImplPow2BitsInvL
{
    public:
    static constexpr long double value = 1.0L / U01ImplPow2BitsL<Bits>::value;
}; // class U01ImplPow2BitsInvL

template <typename RealType, int Bits>
class U01ImplPow2BitsInv
{
    public:
    static constexpr RealType value =
        static_cast<RealType>(U01ImplPow2BitsInvL<Bits>::value);
}; // class U01ImplPow2BitsInv

template <typename UIntType, typename RealType, typename, typename, int UBits,
    int FBits, bool = UBits <= FBits>
class U01Impl;

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Closed, Closed, UBits, FBits, false>
{
    public:
    static RealType eval(UIntType u)
    {
        u >>= UBits - FBits;

        return static_cast<RealType>((u & 1) + u) *
            U01ImplPow2BitsInv<RealType, FBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Closed, Open, UBits, FBits, false>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u >> (UBits - FBits)) *
            U01ImplPow2BitsInv<RealType, FBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Open, Closed, UBits, FBits, false>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u >> (UBits - FBits)) *
            U01ImplPow2BitsInv<RealType, FBits>::value +
            U01ImplPow2BitsInv<RealType, FBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Open, Open, UBits, FBits, false>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u >> (UBits - (FBits - 1))) *
            U01ImplPow2BitsInv<RealType, FBits - 1>::value +
            U01ImplPow2BitsInv<RealType, FBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Closed, Closed, UBits, FBits, true>
{
    public:
    static RealType eval(UIntType u)
    {
        return (static_cast<RealType>(u & 1) + u) *
            U01ImplPow2BitsInv<RealType, UBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Closed, Open, UBits, FBits, true>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u) *
            U01ImplPow2BitsInv<RealType, UBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Open, Closed, UBits, FBits, true>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u) *
            U01ImplPow2BitsInv<RealType, UBits>::value +
            U01ImplPow2BitsInv<RealType, UBits>::value;
    }
}; // class U01Impl

template <typename UIntType, typename RealType, int UBits, int FBits>
class U01Impl<UIntType, RealType, Open, Open, UBits, FBits, true>
{
    public:
    static RealType eval(UIntType u)
    {
        return static_cast<RealType>(u) *
            U01ImplPow2BitsInv<RealType, UBits>::value +
            U01ImplPow2BitsInv<RealType, UBits + 1>::value;
    }
}; // class U01Impl

} // namespace vsmc::internal

template <typename UIntType, typename RealType, typename Left, typename Right>
class U01 : public internal::U01Impl<UIntType, RealType, Left, Right,
                std::numeric_limits<UIntType>::digits,
                std::numeric_limits<RealType>::digits>
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**U01** USED WITH UIntType OTHER THAN UNSIGNED INTEGER TYPES");
    static_assert(std::is_floating_point<RealType>::value,
        "**U01** USED WITH RealType OTHER THAN FLOATING POINT TYPES");
}; // class U01

} // namespace vsmc

#endif //  VSMC_RNG_U01_HPP
