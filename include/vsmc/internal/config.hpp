#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/version.hpp>
#include <vsmc/internal/compiler.hpp>

// nullptr

#ifndef VSMC_NULLPTR
#if VSMC_HAS_CXX11_NULLPTR && VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_NULLPTR nullptr
#else
#define VSMC_NULLPTR 0
#endif
#endif

// cstdint

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// size_type

#ifndef VSMC_SIZE_TYPE
#define VSMC_SIZE_TYPE std::ptrdiff_t
#endif

// RNG types

#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
#endif

#ifndef VSMC_SEED_TYPE
#define VSMC_SEED_TYPE vsmc::Seed
#endif

#ifndef VSMC_RNG_SEED
#define VSMC_RNG_SEED 0
#endif

#ifndef VSMC_CBRNG_TYPE
#define VSMC_CBRNG_TYPE r123::Threefry4x64
#endif

#ifndef VSMC_SEQRNG_TYPE
#define VSMC_SEQRNG_TYPE vsmc::cxx11::mt19937
#endif

#if VSMC_USE_RANDOM123
#define VSMC_PRLRNG_TYPE r123::Engine<VSMC_CBRNG_TYPE>
#else
#define VSMC_PRLRNG_TYPE vsmc::cxx11::mt19937
#endif

// Optional features

#ifndef VSMC_USE_CILK
#define VSMC_USE_CILK 0
#endif

#ifndef VSMC_USE_CL
#define VSMC_USE_CL 0
#endif

#ifndef VSMC_USE_OMP
#define VSMC_USE_OMP 0
#endif

#ifndef VSMC_USE_TBB
#define VSMC_USE_TBB 0
#endif

#ifndef VSMC_USE_MULTITHREAD
#define VSMC_USE_MULTITHREAD 0
#endif

#if VSMC_USE_MULTITHREAD
#define VSMC_USE_MUTEX 1
#endif

// C++11 Libraries from the standard library

#ifndef VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_HAS_CXX11LIB_CHRONO 0
#endif

#ifndef VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_HAS_CXX11LIB_FUNCTIONAL 0
#endif

#ifndef VSMC_HAS_CXX11LIB_MUTEX
#define VSMC_HAS_CXX11LIB_MUTEX 0
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

// If the type_traits implementation is complete

#ifndef VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE
#define VSMC_HAS_LIB_TYPE_TRAITS_COMPLETE 0
#endif

// If the following C++11 (or Boost) libraries are present

#if VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_USE_CHRONO 1
#endif

#if VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_USE_FUNCTIONAL 1
#endif

#if VSMC_HAS_CXX11LIB_MUTEX
#define VSMC_USE_MUTEX 1
#endif

#if VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_USE_RANDOM 1
#endif

#if VSMC_HAS_CXX11LIB_THREAD
#define VSMC_USE_THREAD 1
#endif

#if VSMC_HAS_CXX11LIB_TYPE_TRAITS
#define VSMC_USE_TYPE_TRAITS 1
#endif

#ifndef VSMC_USE_CHRONO
#define VSMC_USE_CHRONO 0
#endif

#ifndef VSMC_USE_FUNCTIONAL
#define VSMC_USE_FUNCTIONAL 1
#endif

#ifndef VSMC_USE_MUTEX
#define VSMC_USE_MUTEX 0
#endif

#ifndef VSMC_USE_RANDOM
#define VSMC_USE_RANDOM 1
#endif

#ifndef VSMC_USE_THREAD
#define VSMC_USE_THREAD 0
#endif

#ifndef VSMC_USE_TYPE_TRAITS
#define VSMC_USE_TYPE_TRAITS 1
#endif

#endif // VSMC_INTERNAL_CONFIG_HPP
