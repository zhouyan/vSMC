#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#include <vsmc/internal/compiler.hpp>

/// \brief C++11 long long interge type
/// \ingroup Config
#ifndef VSMC_HAS_LONG_LONG
#define VSMC_HAS_LONG_LONG 0
#endif

/// \brief Turn vSMC runtime assertions into exceptions
/// \ingroup Config
#ifndef VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT_AS_EXCEPTION 0
#endif

/// \brief Turn vSMC runtime warning into exceptions
/// \ingroup Config
#ifndef VSMC_RUNTIME_WARNING_AS_EXCEPTION
#define VSMC_RUNTIME_WARNING_AS_EXCEPTION 0
#endif

/// \brief Use Random123 for random number generating
/// \ingroup Config
#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
#endif

/// \brief Default RNG type for resampling
/// \ingroup Config
#ifndef VSMC_DEFAULT_RESAMPLE_RNG_TYPE
#define VSMC_DEFAULT_RESAMPLE_RNG_TYPE vsmc::cxx11::mt19937
#endif

/// \brief Default RNG set type
/// \ingroup Config
#ifndef VSMC_DEFAULT_RNG_SET_TYPE
#if VSMC_USE_RANDOM123
#define VSMC_DEFAULT_RNG_SET_TYPE \
    vsmc::RngSet<r123::Engine<r123::Philox2x64>, vsmc::VectorRng>
#else
#define VSMC_DEFAULT_RNG_SET_TYPE \
    vsmc::RngSet<vsmc::cxx11::mt19937, vsmc::VectorRng>
#endif
#endif

/// \brief Enable <vsmc/rng/u01.h> etc., double precision when used with vSMC
#if defined(VSMC_FP_TYPE_IS_FLOAT) && VSMC_FP_TYPE_IS_FLOAT
#define VSMC_FP_TYPE_IS_DOUBLE 0
#else
#define VSMC_FP_TYPE_IS_DOUBLE 1
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
///
/// \details
/// User has to include CLBAS header first
///
/// Use `VSMC_GENERIC_CBLAS_INT` to define the CBLAS integer type
#ifndef VSMC_USE_GENERIC_CBLAS
#define VSMC_USE_GENERIC_CBLAS 0
#endif

/// \brief Use native timing library if `VSMC_HAS_CXX11LIB_CHRONO` test fail
/// \ingroup Config
#ifndef VSMC_HAS_NATIVE_TIME_LIBRARY
#define VSMC_HAS_NATIVE_TIME_LIBRARY 1
#endif

/// \brief The fallback StopWatch type
/// \ingroup Config
///
/// \details
/// The class defined by `VSMC_STOP_WATCH_TYPE` need to be defined before
/// including the `<vsmc/utility/stop_watch.hpp>` header. It shall provide the
/// same interface as DummyStopWatch. This is only used when both
/// `VSMC_HAS_CXX11LIB_CHRONO` and `VSMC_HAS_NATIVE_TIME_LIBRARY` are zero, in
/// which case StopWatch is a typedef of this macro.
#ifndef VSMC_STOP_WATCH_TYPE
#define VSMC_STOP_WATCH_TYPE DummyStopWatch
#endif

/// \brief Default type of Seed
/// \ingroup Config
#ifndef VSMC_SEED_TYPE
#define VSMC_SEED_TYPE vsmc::SeedGenerator<NullType>
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
