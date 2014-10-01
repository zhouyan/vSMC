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

#ifndef VSMC_USE_AES_NI
#define VSMC_USE_AES_NI 0
#endif

#ifndef VSMC_USE_RD_RAND
#define VSMC_USE_RD_RAND 0
#endif

#ifndef VSMC_USE_RD_SEED
#define VSMC_USE_RD_SEED 0
#endif

#ifndef VSMC_USE_CXX11LIB_FUTURE
#define VSMC_USE_CXX11LIB_FUTURE VSMC_HAS_CXX11LIB_FUTURE
#endif

#ifndef VSMC_USE_MPI
#define VSMC_USE_MPI 0
#endif

#ifndef VSMC_USE_OPENCL
#define VSMC_USE_OPENCL 0
#endif

#ifndef VSMC_USE_MKL
#define VSMC_USE_MKL 0
#endif

#ifndef VSMC_USE_CILK
#if defined(__INTEL_COMPILER) && __INTEL_COMPILER >= 1210
#define VSMC_USE_CILK 1
#else
#define VSMC_USE_CILK 0
#endif
#endif

#ifndef VSMC_USE_GCD
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_6)
#define VSMC_USE_GCD 1
#else
#define VSMC_USE_GCD 0
#endif
#endif

#ifndef VSMC_USE_GCD_LION
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
#define VSMC_USE_GCD_LION 1
#else
#define VSMC_USE_GCD_LION 0
#endif
#endif

#ifndef VSMC_USE_OMP
#ifdef _OPENMP
#define VSMC_USE_OMP 1
#else
#define VSMC_USE_OMP 0
#endif
#endif

#ifndef VSMC_USE_PPL
#if defined(_MSC_VER) && _MSC_VER >= 1600
#define VSMC_USE_PPL 1
#else
#define VSMC_USE_PPL 0
#endif
#endif

#ifndef VSMC_USE_TBB
#define VSMC_USE_TBB 0
#endif

#ifndef VSMC_USE_HDF5
#define VSMC_USE_HDF5 0
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
