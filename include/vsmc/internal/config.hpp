#ifndef VSMC_INTERNAL_CONFIG_HPP
#define VSMC_INTERNAL_CONFIG_HPP

#include <vsmc/internal/version.hpp>
#include <vsmc/internal/compiler.hpp>

// cstdint

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// TBB exception

#include <cstddef>
#if defined(__clang__)
#if defined(_LIBCPP_VERSION)  && !defined(__APPLE__)
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // libc++
#if defined(__GLIBCXX__) && (__GLIBCXX__ < 20100429)
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // libstdc++
#endif // __clang__

// Assertion

#ifndef VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT_AS_EXCEPTION 0
#endif

// RNG types

#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
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
