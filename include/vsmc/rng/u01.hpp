//============================================================================
// vSMC/include/vsmc/rng/u01.hpp
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

#ifndef VSMC_RNG_U01_HPP
#define VSMC_RNG_U01_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rngc/u01.h>
#include <vsmc/utility/simd.hpp>

#define VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01_IMPL(                            \
    UBits, FBits, RealType, Left, Right, left, right)                         \
    template <>                                                               \
    class U01Impl<sizeof(std::uint##UBits##_t), sizeof(RealType), Left,       \
        Right>                                                                \
    {                                                                         \
        public:                                                               \
        template <typename UIntType>                                          \
        static RealType eval(UIntType u)                                      \
        {                                                                     \
            return ::vsmc_u01_##left##_##right##_u##UBits##_f##FBits(         \
                static_cast<uint##UBits##_t>(u));                             \
        }                                                                     \
    }; // class U01Impl

#define VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01(UBits, FBits, RealType)          \
    VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01_IMPL(                                \
        UBits, FBits, RealType, Closed, Closed, closed, closed)               \
    VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01_IMPL(                                \
        UBits, FBits, RealType, Closed, Open, closed, open)                   \
    VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01_IMPL(                                \
        UBits, FBits, RealType, Open, Closed, open, closed)                   \
    VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01_IMPL(                                \
        UBits, FBits, RealType, Open, Open, open, open)

namespace vsmc
{

/// \brief Parameter type for open interval
/// \ingroup Distribution
class Open;

/// \brief Parameter type for closed interval
/// \ingroup Distribution
class Closed;

namespace internal
{

template <std::size_t, std::size_t, typename, typename>
class U01Impl;

VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01(32, 32, float)
VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01(32, 64, double)
VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01(64, 32, float)
VSMC_DEFINE_RNG_U01_DISTRIBUTION_U01(64, 64, double)

} // namespace vsmc::internal

template <typename UIntType, typename RealType = double,
    typename Left = Closed, typename Right = Open>
class U01
    : public internal::U01Impl<sizeof(UIntType), sizeof(RealType), Left, Right>
{
}; // class U01

} // namespace vsmc

#endif //  VSMC_RNG_U01_HPP
