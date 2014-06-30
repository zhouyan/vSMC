//============================================================================
// include/vsmc/cxx11/functional.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_CXX11_FUNCTIONAL_HPP
#define VSMC_CXX11_FUNCTIONAL_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <functional>

namespace vsmc { namespace cxx11 {

using std::function;

} } // namespace vsmc::cxx11

#else // VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <boost/function.hpp>

namespace vsmc { namespace cxx11 {

using boost::function;

} } // namespace vsmc::cxx11

#endif // VSMC_HAS_CXX11LIB_FUNCTIONAL

#endif // VSMC_CXX11_FUNCTIONAL_HPP
