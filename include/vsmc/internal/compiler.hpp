//============================================================================
// vSMC/include/vsmc/internal/compiler.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTERNAL_COMPILER_HPP
#define VSMC_INTERNAL_COMPILER_HPP

#include <cstddef>

#if defined(__APPLE__) || defined(__MACOSX)
#define VSMC_MACOSX
#include <Availability.h>
#define VSMC_MAC_10_0  __MAC_10_0
#define VSMC_MAC_10_1  __MAC_10_1
#define VSMC_MAC_10_2  __MAC_10_2
#define VSMC_MAC_10_3  __MAC_10_3
#define VSMC_MAC_10_4  __MAC_10_4
#define VSMC_MAC_10_5  __MAC_10_5
#define VSMC_MAC_10_6  __MAC_10_6
#define VSMC_MAC_10_7  __MAC_10_7
#define VSMC_MAC_10_8  __MAC_10_8
#define VSMC_MAC_10_9  __MAC_10_9
#define VSMC_MAC_10_10 __MAC_10_10
#define VSMC_MAC_VERSION __MAC_OS_X_VERSION_MIN_REQUIRED
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver) VSMC_MAC_VERSION >= ver
#else
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver) 0
#endif

#ifdef VSMC_MACOSX
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_5)
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif
#else // VSMC_MACOSX
#include <stdlib.h>
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif // _POSIX_C_SOURCE >= 200112L
#if defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600
#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 1
#endif
#endif // _XOPEN_SOURCE >= 600
#endif // VSMC_MACOSX

#ifndef VSMC_HAS_POSIX
#define VSMC_HAS_POSIX 0
#endif

#if defined(__INTEL_COMPILER)
#define VSMC_INTEL
#include <vsmc/internal/compiler/intel.hpp>
#elif defined(__clang__)
#define VSMC_CLANG
#include <vsmc/internal/compiler/clang.hpp>
#elif defined(__GNUC__)
#define VSMC_GCC
#include <vsmc/internal/compiler/gcc.hpp>
#elif defined(_MSC_VER)
#define VSMC_MSVC
#include <vsmc/internal/compiler/msvc.hpp>
#endif

// C++11 language features

#ifndef VSMC_HAS_CXX11_LONG_LONG
#define VSMC_HAS_CXX11_LONG_LONG 0
#endif

#ifndef VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE
#define VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE 0
#endif

#ifndef VSMC_HAS_CXX11_ALIAS_TEMPLATES
#define VSMC_HAS_CXX11_ALIAS_TEMPLATES 0
#endif

#ifndef VSMC_HAS_CXX11_ALIGNAS
#define VSMC_HAS_CXX11_ALIGNAS 0
#endif

#ifndef VSMC_HAS_CXX11_ATTRIBUTES
#define VSMC_HAS_CXX11_ATTRIBUTES 0
#endif

#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 0
#endif

#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 0
#endif

#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 0
#endif

#ifndef VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS 0
#endif

#ifndef VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
#define VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS 0
#endif

#ifndef VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS
#define VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS 0
#endif

#ifndef VSMC_HAS_CXX11_DELETED_FUNCTIONS
#define VSMC_HAS_CXX11_DELETED_FUNCTIONS 0
#endif

#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 0
#endif

#ifndef VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS
#define VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS 0
#endif

#ifndef VSMC_HAS_CXX11_IMPLICIT_MOVES
#define VSMC_HAS_CXX11_IMPLICIT_MOVES 0
#endif

#ifndef VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS
#define VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS 0
#endif

#ifndef VSMC_HAS_CXX11_INLINE_NAMESPACES
#define VSMC_HAS_CXX11_INLINE_NAMESPACES 0
#endif

#ifndef VSMC_HAS_CXX11_LAMBDAS
#define VSMC_HAS_CXX11_LAMBDAS 0
#endif

#ifndef VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS 0
#endif

#ifndef VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_HAS_CXX11_NOEXCEPT 0
#endif

#ifndef VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT
#define VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT 0
#endif

#ifndef VSMC_HAS_CXX11_NULLPTR
#define VSMC_HAS_CXX11_NULLPTR 0
#endif

#ifndef VSMC_HAS_CXX11_OVERRIDE_CONTROL
#define VSMC_HAS_CXX11_OVERRIDE_CONTROL 0
#endif

#ifndef VSMC_HAS_CXX11_RANGE_FOR
#define VSMC_HAS_CXX11_RANGE_FOR 0
#endif

#ifndef VSMC_HAS_CXX11_RAW_STRING_LITERALS
#define VSMC_HAS_CXX11_RAW_STRING_LITERALS 0
#endif

#ifndef VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS
#define VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS 0
#endif

#ifndef VSMC_HAS_CXX11_RVALUE_REFERENCES
#define VSMC_HAS_CXX11_RVALUE_REFERENCES 0
#endif

#ifndef VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_HAS_CXX11_STATIC_ASSERT 0
#endif

#ifndef VSMC_HAS_CXX11_STRONG_ENUMS
#define VSMC_HAS_CXX11_STRONG_ENUMS 0
#endif

#ifndef VSMC_HAS_CXX11_THREAD_LOCAL
#define VSMC_HAS_CXX11_THREAD_LOCAL 0
#endif

#ifndef VSMC_HAS_CXX11_TRAILING_RETURN
#define VSMC_HAS_CXX11_TRAILING_RETURN 0
#endif

#ifndef VSMC_HAS_CXX11_UNICODE_LITERALS
#define VSMC_HAS_CXX11_UNICODE_LITERALS 0
#endif

#ifndef VSMC_HAS_CXX11_UNRESTRICTED_UNIONS
#define VSMC_HAS_CXX11_UNRESTRICTED_UNIONS 0
#endif

#ifndef VSMC_HAS_CXX11_USER_LITERALS
#define VSMC_HAS_CXX11_USER_LITERALS 0
#endif

#ifndef VSMC_HAS_CXX11_VARIADIC_TEMPLATES
#define VSMC_HAS_CXX11_VARIADIC_TEMPLATES 0
#endif

// C++11 library features

#ifndef VSMC_HAS_CXX11LIB_ALGORITHM
#define VSMC_HAS_CXX11LIB_ALGORITHM 0
#endif

#ifndef VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_HAS_CXX11LIB_CHRONO 0
#endif

#ifndef VSMC_HAS_CXX11LIB_CMATH
#define VSMC_HAS_CXX11LIB_CMATH 0
#endif

#ifndef VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_HAS_CXX11LIB_FUNCTIONAL 0
#endif

#ifndef VSMC_HAS_CXX11LIB_FUTURE
#define VSMC_HAS_CXX11LIB_FUTURE 0
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

#ifndef VSMC_HAS_CXX11LIB_TUPLE
#define VSMC_HAS_CXX11LIB_TUPLE 0
#endif

#ifndef VSMC_USE_CXX11LIB_FUTURE
#define VSMC_USE_CXX11LIB_FUTURE VSMC_HAS_CXX11LIB_FUTURE
#endif

// C++14 language features

#ifndef VSMC_HAS_CXX14_BINARY_LITERALS
#define VSMC_HAS_CXX14_BINARY_LITERALS 0
#endif

#ifndef VSMC_HAS_CXX14_CONTEXTUAL_CONVERSIONS
#define VSMC_HAS_CXX14_CONTEXTUAL_CONVERSIONS 0
#endif

#ifndef VSMC_HAS_CXX14_DECLTYPE_AUTO
#define VSMC_HAS_CXX14_DECLTYPE_AUTO 0
#endif

#ifndef VSMC_HAS_CXX14_DIGIT_SEPERATORS
#define VSMC_HAS_CXX14_DIGIT_SEPERATORS 0
#endif

#ifndef VSMC_HAS_CXX14_AGGREGATE_NSDMI
#define VSMC_HAS_CXX14_AGGREGATE_NSDMI 0
#endif

#ifndef VSMC_HAS_CXX14_INIT_CAPTURES
#define VSMC_HAS_CXX14_INIT_CAPTURES 0
#endif

#ifndef VSMC_HAS_CXX14_GENERIC_LAMBDAS
#define VSMC_HAS_CXX14_GENERIC_LAMBDAS 0
#endif

#ifndef VSMC_HAS_CXX14_RELAXED_CONSTEXPR
#define VSMC_HAS_CXX14_RELAXED_CONSTEXPR 0
#endif

#ifndef VSMC_HAS_CXX14_RETURN_TYPE_DEDUCTION
#define VSMC_HAS_CXX14_RETURN_TYPE_DEDUCTION 0
#endif

#ifndef VSMC_HAS_CXX14_RUNTIME_SIZED_ARRAYS
#define VSMC_HAS_CXX14_RUNTIME_SIZED_ARRAYS 0
#endif

#ifndef VSMC_HAS_CXX14_VARIABLE_TEMPLATES
#define VSMC_HAS_CXX14_VARIABLE_TEMPLATES 0
#endif

// C99 library features

#ifndef VSMC_HAS_C99LIB_MATH
#define VSMC_HAS_C99LIB_MATH 0
#endif

// Compiler features

#ifndef VSMC_HAS_WARNING
#define VSMC_HAS_WARNING 0
#endif

#ifndef VSMC_STRONG_INLINE
#define VSMC_STRONG_INLINE inline
#endif

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#ifndef __has_feature
#define __has_feature(x) 0
#endif

#ifndef __has_extension
#define __has_extension(x) 0
#endif

#ifndef __has_cpp_attribute
#define __has_cpp_attribute(x) 0
#endif

#ifndef __has_attribute
#define __has_attribute(x) 0
#endif

#ifndef __has_include
#define __has_include(x) 0
#endif

#ifndef __has_include_next
#define __has_include_next(x) 0
#endif

// Target specific features

#ifndef VSMC_INT64
#define VSMC_INT64 long long
#endif

#ifndef VSMC_HAS_INT128
#define VSMC_HAS_INT128 0
#endif

#ifndef VSMC_HAS_AES_NI
#define VSMC_HAS_AES_NI 0
#endif

#ifndef VSMC_HAS_RDRAND
#define VSMC_HAS_RDRAND 0
#endif

#ifndef VSMC_HAS_AVX2
#ifdef __AVX2__
#define VSMC_HAS_AVX2 1
#else
#define VSMC_HAS_AVX2 0
#endif
#endif

#ifndef VSMC_HAS_AVX
#ifdef __AVX__
#define VSMC_HAS_AVX 1
#else
#define VSMC_HAS_AVX VSMC_HAS_AVX2
#endif
#endif

#ifndef VSMC_HAS_SSE4_2
#ifdef __SSE4_2__
#define VSMC_HAS_SSE4_2 1
#else
#define VSMC_HAS_SSE4_2 VSMC_HAS_AVX
#endif
#endif

#ifndef VSMC_HAS_SSE4_1
#ifdef __SSE4_1__
#define VSMC_HAS_SSE4_1 1
#else
#define VSMC_HAS_SSE4_1 AVX_HAS_SSE4_2
#endif
#endif

#ifndef VSMC_HAS_SSSE3
#ifdef __SSSE3__
#define VSMC_HAS_SSSE3 1
#else
#define VSMC_HAS_SSSE3 VSMC_HAS_SSE4_1
#endif
#endif

#ifndef VSMC_HAS_SSE3
#ifdef __SSE3__
#define VSMC_HAS_SSE3 1
#else
#define VSMC_HAS_SSE3 VSMC_HAS_SSSE3
#endif
#endif

#ifndef VSMC_HAS_SSE2
#ifdef __SSE2__
#define VSMC_HAS_SSE2 1
#else
#define VSMC_HAS_SSE2 VSMC_HAS_SSE3
#endif
#endif

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
#ifndef VSMC_HAS_GCD_LION
#define VSMC_HAS_GCD_LION 1
#endif
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP
