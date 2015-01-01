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
#include <vsmc/rng/u01.h>

#define VSMC_DEFINE_RNG_U01(FPType, Left, Right, left, right, UBits, FBits) \
template <> struct U01<Left, Right, uint##UBits##_t, FPType>                 \
{                                                                            \
    FPType operator() (uint##UBits##_t u) const                              \
    {return ::u01_##left##_##right##_##UBits##_##FBits(u);}                  \
                                                                             \
    static FPType uint2fp (uint##UBits##_t u)                                \
    {return ::u01_##left##_##right##_##UBits##_##FBits(u);}                  \
};

namespace vsmc {

/// \brief Parameter type for open interval
/// \ingroup U01
struct Open {};

/// \brief Parameter type for closed interval
/// \ingroup U01
struct Closed {};

template <typename, typename, typename, typename> struct U01;

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Closed, closed, closed, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Open, closed, open, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Closed, open, closed, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Open, open, open, 32, 24)

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Closed, closed, closed, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Open, closed, open, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Closed, open, closed, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Open, open, open, 32, 53)

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Closed, closed, closed, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Open, closed, open, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Closed, open, closed, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Open, open, open, 64, 24)

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Closed, closed, closed, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Open, closed, open, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Closed, open, closed, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Open, open, open, 64, 53)

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
