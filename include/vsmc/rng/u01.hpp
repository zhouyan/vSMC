//============================================================================
// include/vsmc/rng/u01.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
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
