#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/compiler.h>
#include <vsmc/internal/version.hpp>

// cstdint

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// size_type

#ifndef VSMC_SIZE_TYPE
#define VSMC_SIZE_TYPE std::size_t
#endif

// OpenCL Device type

#ifndef VSMC_CL_DEFAULT_ID
#define VSMC_CL_DEFAULT_ID vsmc::opencl::Default
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

#if VSMC_USE_RANDOM123
#define VSMC_PRLRNG_TYPE r123::Engine<VSMC_CBRNG_TYPE>
#else
#define VSMC_PRLRNG_TYPE vsmc::cxx11::mt19937
#endif

// Optional features

#ifndef VSMC_USE_CILK
#define VSMC_USE_CILK 0
#endif

#ifndef VSMC_USE_GCD
#define VSMC_USE_GCD 0
#endif

#ifndef VSMC_USE_OMP
#define VSMC_USE_OMP 0
#endif

#ifndef VSMC_USE_PPL
#define VSMC_USE_PPL 0
#endif

#ifndef VSMC_USE_STD
#define VSMC_USE_STD 0
#endif

#ifndef VSMC_USE_TBB
#define VSMC_USE_TBB 0
#endif

#ifndef VSMC_USE_CL
#define VSMC_USE_CL 0
#endif

#ifndef VSMC_USE_MKL
#define VSMC_USE_MKL 0
#endif

#ifndef VSMC_USE_VECLIB
#define VSMC_USE_VECLIB 0
#endif

#ifndef VSMC_USE_GENERIC_CBLAS
#define VSMC_USE_GENERIC_CBLAS 0
#endif

#ifndef VSMC_USE_ARMADILLO
#define VSMC_USE_ARMADILLO 0
#endif

#ifndef VSMC_USE_EIGEN
#define VSMC_USE_EIGEN 0
#endif

// C++11 Language features

#ifndef VSMC_HAS_CXX11_CONSTEXPR
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

#endif // VSMC_INTERNAL_CONFIG_HPP
