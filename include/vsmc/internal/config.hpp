#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/compiler.hpp>

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// #if defined(__clang__)
// #ifndef TBB_USE_CAPTURED_EXCEPTION
// #define TBB_USE_CAPTURED_EXCEPTION 1
// #endif
// #endif

#if  VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_CONSTEXPR constexpr
#else
#define VSMC_CONSTEXPR
#endif

#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_EXPLICIT_OPERATOR explicit
#else
#define VSMC_EXPLICIT_OPERATOR
#endif

#if VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_NOEXCEPT noexcept
#else
#define VSMC_NOEXCEPT
#endif

#if VSMC_HAS_CXX11_NULLPTR && VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_NULLPTR nullptr
#else
#define VSMC_NULLPTR 0
#endif

/// \brief Turn vSMC runtime assertions into exceptions
/// \ingroup Config
#ifndef VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT_AS_EXCEPTION 0
#endif

/// \brief Use Random123 for random number generating
/// \ingroup Config
#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
#endif

/// \brief Use Intel MKL for BLAS level 2, level 3 and other computations
/// \ingroup Config
#ifndef VSMC_USE_MKL
#define VSMC_USE_MKL 0
#endif

/// \brief Use Apple vecLib for BLAS level 2 and level 3 computations
/// \ingroup Config
#ifndef VSMC_USE_VECLIB
#define VSMC_USE_VECLIB 0
#endif

/// \brief Use generic CBLAS for BLAS level 2 and level 3 computations
/// \ingroup Config
/// \note User has to include CLBAS header first
/// \note Use `VSMC_GENERIC_CBLAS_INT` to define the CBLAS integer type
#ifndef VSMC_USE_GENERIC_CBLAS
#define VSMC_USE_GENERIC_CBLAS 0
#endif

/// \brief Use the Armadillo library for BLAS level 2 and level 3 computations
/// \ingroup Config
#ifndef VSMC_USE_ARMADILLO
#define VSMC_USE_ARMADILLO 0
#endif

/// \brief Use the Eigen library for BLAS level 2 and level 3 computations
/// \ingroup Config
#ifndef VSMC_USE_EIGEN
#define VSMC_USE_EIGEN 0
#endif

/// \brief Use native timing library if `VSMC_HAS_CXX11LIB_CHRONO` is zero
/// \ingroup Config
#ifndef VSMC_HAS_NATIVE_TIME_LIBRARY
#define VSMC_HAS_NATIVE_TIME_LIBRARY 1
#endif

/// \brief The fallback StopWatch type
/// \ingroup Config
/// \note The class defined by `VSMC_STOP_WATCH_TYPE` need to be defined before
/// including the `<vsmc/utility/stop_watch.hpp>` header. It shall provide the
/// same interface as DummyStopWatch. This is only used when both
/// `VSMC_HAS_CXX11LIB_CHRONO` and `VSMC_HAS_NATIVE_TIME_LIBRARY` are zero, in
/// which case StopWatch is a typedef of this macro.
#ifndef VSMC_STOP_WATCH_TYPE
#define VSMC_STOP_WATCH_TYPE DummyStopWatch
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
