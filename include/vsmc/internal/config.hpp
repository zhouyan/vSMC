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

// Parallelization features

#ifndef VSMC_HAS_CILK
#define VSMC_HAS_CILK 0
#endif

#ifndef VSMC_HAS_GCD
#define VSMC_HAS_GCD 0
#endif

#ifndef VSMC_HAS_GCD_LION
#define VSMC_HAS_GCD_LION 0
#endif

#ifndef VSMC_HAS_OMP
#define VSMC_HAS_OMP 0
#endif

#ifndef VSMC_HAS_PPL
#define VSMC_HAS_PPL 0
#endif

#ifndef VSMC_HAS_TBB
#define VSMC_HAS_TBB 0
#endif

#ifndef VSMC_HAS_MPI
#define VSMC_HAS_MPI 0
#endif

#ifndef VSMC_HAS_OPENCL
#define VSMC_HAS_OPENCL 0
#endif

// Optional libraries

#ifndef VSMC_HAS_HDF5
#define VSMC_HAS_HDF5 0
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
#define VSMC_HAS_VECLIB 0
#endif

#ifndef VSMC_USE_VECLIB_CBLAS
#define VSMC_USE_VECLIB_CBLAS VSMC_HAS_VECLIB
#endif

#ifndef VSMC_USE_VECLIB_VFORCE
#define VSMC_USE_VECLIB_VFORCE VSMC_HAS_VECLIB
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
