//============================================================================
// include/vsmc/rng/intrin.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTERNAL_INTRIN_HPP
#define VSMC_INTERNAL_INTRIN_HPP

#include <vsmc/internal/config.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#elif VSMC_HAS_INTRINSIC_FUNCTION
#include <x86intrin.h>
#else
#error x86 intrinsic function is not supported
#endif

#endif // VSMC_INTERNAL_INTRIN_HPP
