#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/version.hpp>
#include <vsmc/internal/compiler.hpp>

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#define VSMC_PREVENT_MIN_MAX

// C++11 Libraries

#ifndef VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_HAS_CXX11LIB_CHRONO 0
#endif

#ifndef VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_HAS_CXX11LIB_FUNCTIONAL 0
#endif

#ifndef VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 0
#endif

#ifndef VSMC_HAS_CXX11LIB_THREAD
#define VSMC_HAS_CXX11LIB_THREAD 0
#endif

#ifndef VSMC_HAS_CXX11LIB_TYPE_TRAITS
#define VSMC_HAS_CXX11LIB_TYPE_TRAITS 0
#endif

// Thread support

#if VSMC_HAS_CXX11LIB_THREAD
#define VSMC_HAS_LIB_THREAD 1
#endif

#ifndef VSMC_HAS_LIB_THREAD
#define VSMC_HAS_LIB_THREAD 0
#endif

// size_type

#ifndef VSMC_SIZE_TYPE
#define VSMC_SIZE_TYPE std::ptrdiff_t
#endif

// RNG types

#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
#endif

#ifndef VSMC_RNG_SEED
#define VSMC_RNG_SEED 0
#endif

#ifndef VSMC_SEQRNG_TYPE
#define VSMC_SEQRNG_TYPE vsmc::cxx11::mt19937
#endif

#ifndef VSMC_CBRNG_TYPE
#define VSMC_CBRNG_TYPE r123::Threefry4x64
#endif

#if VSMC_USE_RANDOM123
#define VSMC_PRLRNG_TYPE vsmc::cxx11::Engine<VSMC_CBRNG_TYPE>
#else
#define VSMC_PRLRNG_TYPE vsmc::cxx11::mt19937
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
