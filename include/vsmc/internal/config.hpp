#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/version.hpp>
#include <vsmc/internal/compiler.hpp>

// cstdint

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// Assertion

#ifndef VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT_AS_EXCEPTION 0
#endif

// OpenCL Device type

#ifndef VSMC_CL_DEFAULT_ID
#define VSMC_CL_DEFAULT_ID vsmc::CLDefault
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

// Linear algebra

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
