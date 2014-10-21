//============================================================================
// include/vsmc/internal/config.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/compiler.hpp>

#ifndef VSMC_NO_STATIC_ASSERT
#define VSMC_NO_STATIC_ASSERT 0
#endif

#ifndef VSMC_NO_RUNTIME_ASSERT
#ifndef NDEBUG
#define VSMC_NO_RUNTIME_ASSERT 0
#else
#define VSMC_NO_RUNTIME_ASSERT 1
#endif
#endif

#ifndef VSMC_NO_RUNTIME_WARNING
#ifndef NDEBUG
#define VSMC_NO_RUNTIME_WARNING 0
#else
#define VSMC_NO_RUNTIME_WARNING 1
#endif
#endif

/// \brief Turn vSMC runtime assertions into exceptions
/// \ingroup Config
#ifndef VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT_AS_EXCEPTION 0
#endif

/// \brief Turn vSMC runtime warnings into exceptions
/// \ingroup Config
#ifndef VSMC_RUNTIME_WARNING_AS_EXCEPTION
#define VSMC_RUNTIME_WARNING_AS_EXCEPTION 0
#endif

#ifndef VSMC_USE_CXX11LIB_FUTURE
#define VSMC_USE_CXX11LIB_FUTURE VSMC_HAS_CXX11LIB_FUTURE
#endif

#ifndef VSMC_HAS_MPI
#define VSMC_HAS_MPI 0
#endif

#ifndef VSMC_HAS_OPENCL
#define VSMC_HAS_OPENCL 0
#endif

#ifndef VSMC_HAS_MKL
#define VSMC_HAS_MKL 0
#endif

#ifndef VSMC_USE_MKL_CBLAS
#define VSMC_USE_MKL_CBLAS VSMC_HAS_MKL
#endif

#ifndef VSMC_USE_MKL_VML
#define VSMC_USE_MKL_VML VSMC_HAS_MKL
#endif

#ifndef VSMC_USE_MKL_VSL
#define VSMC_USE_MKL_VSL VSMC_HAS_MKL
#endif

#ifndef VSMC_HAS_VECLIB
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_4)
#define VSMC_HAS_VECLIB 1
#else
#define VSMC_HAS_VECLIB 0
#endif
#endif

#ifndef VSMC_USE_VECLIB_CBLAS
#define VSMC_USE_VECLIB_CBLAS VSMC_HAS_VECLIB
#endif

#ifndef VSMC_USE_VECLIB_VFORCE
#define VSMC_USE_VECLIB_VFORCE VSMC_HAS_VECLIB
#endif

#ifndef VSMC_HAS_CILK
#if defined(__INTEL_COMPILER) && __INTEL_COMPILER >= 1210
#define VSMC_HAS_CILK 1
#else
#define VSMC_HAS_CILK 0
#endif
#endif

#ifndef VSMC_HAS_GCD
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_6)
#define VSMC_HAS_GCD 1
#else
#define VSMC_HAS_GCD 0
#endif
#endif

#ifndef VSMC_HAS_GCD_LION
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
#define VSMC_HAS_GCD_LION 1
#else
#define VSMC_HAS_GCD_LION 0
#endif
#endif

#ifndef VSMC_HAS_OMP
#ifdef _OPENMP
#define VSMC_HAS_OMP 1
#else
#define VSMC_HAS_OMP 0
#endif
#endif

#ifndef VSMC_HAS_PPL
#if defined(_MSC_VER) && _MSC_VER >= 1600
#define VSMC_HAS_PPL 1
#else
#define VSMC_HAS_PPL 0
#endif
#endif

#ifndef VSMC_HAS_TBB
#define VSMC_HAS_TBB 0
#endif

#ifndef VSMC_HAS_HDF5
#define VSMC_HAS_HDF5 0
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
