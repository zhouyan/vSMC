#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/compiler.hpp>

/// \brief Disable vSMC static assertions (fatal error)
/// \ingroup Config
#ifndef VSMC_NO_STATIC_ASSERT
#define VSMC_NO_STATIC_ASSERT 0
#endif

/// \brief Disable vSMC runtime assertions (fatal error)
/// \ingroup Config
#ifndef VSMC_NO_RUNTIME_ASSERT
#ifndef NDEBUG
#define VSMC_NO_RUNTIME_ASSERT 0
#else
#define VSMC_NO_RUNTIME_ASSERT 1
#endif
#endif

/// \brief Disable vSMC runtime warnings (nonfatal error)
/// \ingroup Config
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

/// \brief Use AES-NI for random number generating
/// \ingroup Config
#ifndef VSMC_USE_AES_NI
#define VSMC_USE_AES_NI 0
#endif

/// \brief Use MPI
/// \ingroup ThirdParty
#ifndef VSMC_USE_MPI
#define VSMC_USE_MPI 0
#endif

/// \brief Use OpenCL
/// \ingroup ThirdParty
#ifndef VSMC_USE_OPENCL
#define VSMC_USE_OPENCL 0
#endif

/// \brief Use MKL
/// \ingroup ThirdParty
#ifndef VSMC_USE_MKL
#define VSMC_USE_MKL 0
#endif

/// \brief Use Intel Cilk Plus
/// \ingroup ThirdParty
#ifndef VSMC_USE_CILK
#if defined(__INTEL_COMPILER) && __INTEL_COMPILER >= 1210
#define VSMC_USE_CILK 1
#else
#define VSMC_USE_CILK 0
#endif
#endif

/// \brief Use Apple GCD
/// \ingroup ThirdParty
#ifndef VSMC_USE_GCD
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_6)
#define VSMC_USE_GCD 1
#else
#define VSMC_USE_GCD 0
#endif
#endif

/// \brief USE OpenMP
/// \ingroup ThirdParty
#ifndef VSMC_USE_OMP
#ifdef _OPENMP
#define VSMC_USE_OMP 1
#else
#define VSMC_USE_OMP 0
#endif
#endif

/// \brief Use Microsoft PPL
/// \ingroup ThirdParty
#ifndef VSMC_USE_PPL
#if defined(_MSC_VER) && _MSC_VER >= 1600
#define VSMC_USE_PPL 1
#else
#define VSMC_USE_PPL 0
#endif
#endif

/// \brief Use Intel TBB
/// \ingroup ThirdParty
#ifndef VSMC_USE_TBB
#define VSMC_USE_TBB 0
#endif

/// \brief Use HDF5
/// \ingroup ThirdParty
#ifndef VSMC_USE_HDF5
#define VSMC_USE_HDF5 0
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
