//============================================================================
// vsmc/internal/compiler.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTERNAL_COMPILER_HPP
#define VSMC_INTERNAL_COMPILER_HPP

#include <cstddef>

#if defined(__x86_64__) || defined(_M_AMD64) || defined (_M_X64)
#ifndef VSMC_X86_64
#define VSMC_X86_64 1
#endif
#endif

#ifndef VSMC_X86_64
#define VSMC_X86_64 0
#endif

#if VSMC_X86_64 || defined(__i386__) || defined(_M_I386) || defined(_WIN32)
#ifndef VSMC_X86
#define VSMC_X86 1
#endif
#endif

#ifndef VSMC_X86
#define VSMC_X86 0
#endif

#if defined(__APPLE__) || defined(__MACOSX)
#include <Availability.h>
#endif

#if defined(__MAC_OS_X_VERSION_MIN_REQUIRED) && defined(__MAC_10_0)
#define VSMC_MAC_10_0 __MAC_10_0
#define VSMC_MAC_10_1 __MAC_10_1
#define VSMC_MAC_10_2 __MAC_10_2
#define VSMC_MAC_10_3 __MAC_10_3
#define VSMC_MAC_10_4 __MAC_10_4
#define VSMC_MAC_10_5 __MAC_10_5
#define VSMC_MAC_10_6 __MAC_10_6
#define VSMC_MAC_10_7 __MAC_10_7
#define VSMC_MAC_10_8 __MAC_10_8
#define VSMC_MAC_10_9 __MAC_10_9
#define VSMC_MAC_VERSION __MAC_OS_X_VERSION_MIN_REQUIRED
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver) VSMC_MAC_VERSION >= ver
#else
#define VSMC_MAC_VERSION_MIN_REQUIRED(ver) 0
#endif

#if defined(__INTEL_COMPILER)
#include <vsmc/internal/compiler/intel.hpp>
#elif defined(__clang__)
#include <vsmc/internal/compiler/clang.hpp>
#elif defined(__OPEN64__)
#include <vsmc/internal/compiler/open64.hpp>
#elif defined(__SUNPRO_CC)
#include <vsmc/internal/compiler/sunpro.hpp>
#elif defined(__GNUC__)
#include <vsmc/internal/compiler/gcc.hpp>
#elif defined(_MSC_VER)
#include <vsmc/internal/compiler/msvc.hpp>
#endif

#ifdef VSMC_DOXYGEN
#undef VSMC_HAS_CXX11_LONG_LONG
#undef VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE
#undef VSMC_HAS_CXX11_ALIAS_TEMPLATES
#undef VSMC_HAS_CXX11_ALIGNAS
#undef VSMC_HAS_CXX11_ATTRIBUTES
#undef VSMC_HAS_CXX11_AUTO_TYPE
#undef VSMC_HAS_CXX11_CONSTEXPR
#undef VSMC_HAS_CXX11_DECLTYPE
#undef VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS
#undef VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
#undef VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS
#undef VSMC_HAS_CXX11_DELETED_FUNCTIONS
#undef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#undef VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS
#undef VSMC_HAS_CXX11_IMPLICIT_MOVES
#undef VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS
#undef VSMC_HAS_CXX11_INLINE_NAMESPACES
#undef VSMC_HAS_CXX11_LAMBDAS
#undef VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS
#undef VSMC_HAS_CXX11_NOEXCEPT
#undef VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT
#undef VSMC_HAS_CXX11_NULLPTR
#undef VSMC_HAS_CXX11_OVERRIDE_CONTROL
#undef VSMC_HAS_CXX11_RANGE_FOR
#undef VSMC_HAS_CXX11_RAW_STRING_LITERALS
#undef VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS
#undef VSMC_HAS_CXX11_RVALUE_REFERENCES
#undef VSMC_HAS_CXX11_STATIC_ASSERT
#undef VSMC_HAS_CXX11_STRONG_ENUMS
#undef VSMC_HAS_CXX11_THREAD_LOCAL
#undef VSMC_HAS_CXX11_TRAILING_RETURN
#undef VSMC_HAS_CXX11_UNICODE_LITERALS
#undef VSMC_HAS_CXX11_UNRESTRICTED_UNIONS
#undef VSMC_HAS_CXX11_USER_LITERALS
#undef VSMC_HAS_CXX11_VARIADIC_TEMPLATES
#undef VSMC_HAS_CXX11LIB_CHRONO
#undef VSMC_HAS_CXX11LIB_CMATH
#undef VSMC_HAS_CXX11LIB_FUNCTIONAL
#undef VSMC_HAS_CXX11LIB_FUTURE
#undef VSMC_HAS_CXX11LIB_MUTEX
#undef VSMC_HAS_CXX11LIB_RANDOM
#undef VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
#undef VSMC_HAS_CXX11LIB_THREAD
#undef VSMC_HAS_CXX11LIB_TUPLE
#undef VSMC_HAS_C99LIB_MATH
#undef VSMC_HAS_INT128
#undef VSMC_HAS_INLINE_ASSEMBLY
#undef VSMC_HAS_INTRINSIC_FUNCTION
#undef VSMC_HAS_INTRINSIC_INT64
#undef VSMC_HAS_OPENCL_DOUBLE
#undef __MAC_OS_X_VERSION_MIN_REQUIRED
#endif

//  C++11 language features

/// \brief C++11 long long
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_LONG_LONG
#define VSMC_HAS_CXX11_LONG_LONG 0
#endif

/// \brief C++11 SFINAE includes access control
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE
#define VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE 0
#endif

/// \brief C++11 alias templates
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_ALIAS_TEMPLATES
#define VSMC_HAS_CXX11_ALIAS_TEMPLATES 0
#endif

/// \brief C++11 alignas
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_ALIGNAS
#define VSMC_HAS_CXX11_ALIGNAS 0
#endif

/// \brief C++11 attributes
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_ATTRIBUTES
#define VSMC_HAS_CXX11_ATTRIBUTES 0
#endif

/// \brief C++11 auto
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 0
#endif

/// \brief C++11 constexpr
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 0
#endif

/// \brief C++11 decltype
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 0
#endif

/// \brief C++11 default function template arguments
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS 0
#endif

/// \brief C++11 default functions
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
#define VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS 0
#endif

/// \brief C++11 delegating constructors
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS
#define VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS 0
#endif

/// \brief C++11 delete functions
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_DELETED_FUNCTIONS
#define VSMC_HAS_CXX11_DELETED_FUNCTIONS 0
#endif

/// \brief C++11 explicit conversion operators
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 0
#endif

/// \brief C++11 generalized initializers
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS
#define VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS 0
#endif

/// \brief C++11 implicit move constructors and assignment operators
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_IMPLICIT_MOVES
#define VSMC_HAS_CXX11_IMPLICIT_MOVES 0
#endif

/// \brief C++11 inheriting constructors
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS
#define VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS 0
#endif

/// \brief C++11 inline namespaces
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_INLINE_NAMESPACES
#define VSMC_HAS_CXX11_INLINE_NAMESPACES 0
#endif

/// \brief C++11 lambda expressions
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_LAMBDAS
#define VSMC_HAS_CXX11_LAMBDAS 0
#endif

/// \brief C++11 local and unnamed types as template arguments
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS 0
#endif

/// \brief C++11 noexcept
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_HAS_CXX11_NOEXCEPT 0
#endif

/// \brief C++11 non-static member initialization
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT
#define VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT 0
#endif

/// \brief C++11 nullptr
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_NULLPTR
#define VSMC_HAS_CXX11_NULLPTR 0
#endif

/// \brief C++11 override control
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_OVERRIDE_CONTROL
#define VSMC_HAS_CXX11_OVERRIDE_CONTROL 0
#endif

/// \brief C++11 range based for
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_RANGE_FOR
#define VSMC_HAS_CXX11_RANGE_FOR 0
#endif

/// \brief C++11 raw string literals
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_RAW_STRING_LITERALS
#define VSMC_HAS_CXX11_RAW_STRING_LITERALS 0
#endif

/// \brief C++11 reference qualified functions
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS
#define VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS 0
#endif

/// \brief C++11 rvalue references
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_RVALUE_REFERENCES
#define VSMC_HAS_CXX11_RVALUE_REFERENCES 0
#endif

/// \brief C++11 static_assert
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_HAS_CXX11_STATIC_ASSERT 0
#endif

/// \brief C++11 strong typed enums
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_STRONG_ENUMS
#define VSMC_HAS_CXX11_STRONG_ENUMS 0
#endif

/// \brief C++11 thread_local
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_THREAD_LOCAL
#define VSMC_HAS_CXX11_THREAD_LOCAL 0
#endif

/// \brief C++11 trailing return type function declaration
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_TRAILING_RETURN
#define VSMC_HAS_CXX11_TRAILING_RETURN 0
#endif

/// \brief C++11 Unicode literals
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_UNICODE_LITERALS
#define VSMC_HAS_CXX11_UNICODE_LITERALS 0
#endif

/// \brief C++11 Unrestricted unions
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_UNRESTRICTED_UNIONS
#define VSMC_HAS_CXX11_UNRESTRICTED_UNIONS 0
#endif

/// \brief C++11 User defined literals
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_USER_LITERALS
#define VSMC_HAS_CXX11_USER_LITERALS 0
#endif

/// \brief C++11 Variadic templates
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11_VARIADIC_TEMPLATES
#define VSMC_HAS_CXX11_VARIADIC_TEMPLATES 0
#endif

//  C++11 library features

/// \brief C++11 `<chrono>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_HAS_CXX11LIB_CHRONO 0
#endif

/// \brief C++11 `<cmath>` (special functions)
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_CMATH
#define VSMC_HAS_CXX11LIB_CMATH 0
#endif

/// \brief C++11 `<functional>` (std::function)
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_HAS_CXX11LIB_FUNCTIONAL 0
#endif

/// \brief C++11 `<future>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_FUTURE
#define VSMC_HAS_CXX11LIB_FUTURE 0
#endif

/// \brief C++11 `<mutex>`
#ifndef VSMC_HAS_CXX11LIB_MUTEX
#define VSMC_HAS_CXX11LIB_MUTEX 0
#endif

/// \brief C++11 `<random>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 0
#endif

/// \brief C++11 `<random>` RNG support constexpr min/max
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
#define VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX 0
#endif

/// \brief C++11 `<thread>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_THREAD
#define VSMC_HAS_CXX11LIB_THREAD 0
#endif

/// \brief C++11 `<tuple>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_TUPLE
#define VSMC_HAS_CXX11LIB_TUPLE 0
#endif

// C99 library features

/// \brief C99 `<math.h>` (special functions)
/// \ingroup Compiler
#ifndef VSMC_HAS_C99LIB_MATH
#define VSMC_HAS_C99LIB_MATH 0
#endif

// Target specific features

/// \brief 128-bits integers
/// \ingroup Compiler
#ifndef VSMC_HAS_INT128
#define VSMC_HAS_INT128 0
#endif

/// \brief Inline assembly
/// \ingroup Compiler
#ifndef VSMC_HAS_INLINE_ASSEMBLY
#define VSMC_HAS_INLINE_ASSEMBLY 0
#endif

/// \brief Intrinsic functions
/// \ingroup Compiler
#ifndef VSMC_HAS_INTRINSIC_FUNCTION
#define VSMC_HAS_INTRINSIC_FUNCTION 0
#endif

/// \brief Intrinsic functions defines `__int64`
/// \ingroup Compiler
#ifndef VSMC_HAS_INTRINSIC_INT64
#define VSMC_HAS_INTRINSIC_INT64 0
#endif

/// \brief Intrinsic functions does not define `__int64` and use `long long` as
/// 64-bits integers
/// \ingroup Compiler
#ifndef VSMC_HAS_INTRINSIC_INT64_LONG_LONG
#define VSMC_HAS_INTRINSIC_INT64_LONG_LONG 1
#endif

/// \brief Enable <vsmc/rng/u01.h> etc., double precision when used with vSMC
/// \ingroup Config
#ifndef VSMC_HAS_OPENCL_DOUBLE
#define VSMC_HAS_OPENCL_DOUBLE 1
#endif

#if VSMC_HAS_INTRINSIC_INT64
#define VSMC_INTRINSIC_INT64 __int64
#elif VSMC_HAS_INTRINSIC_INT64_LONG_LONG
#define VSMC_INTRINSIC_INT64 long long
#else
#define VSMC_INTRINSIC_INT64 long
#endif

#if VSMC_HAS_INTRINSIC_FUNCTION

#ifdef __MMX__
#ifndef VSMC_MMX
#define VSMC_MMX 1
#endif
#endif

#ifdef __SSE__
#ifndef VSMC_SSE
#define VSMC_SSE 1
#endif
#endif

#ifdef __SSE2__
#ifndef VSMC_SSE2
#define VSMC_SSE2 1
#endif
#endif

#ifdef __SSE3__
#ifndef VSMC_SSE3
#define VSMC_SSE3 1
#endif
#endif

#ifdef __SSSE3__
#ifndef VSMC_SSSE3
#define VSMC_SSSE3 1
#endif
#endif

#ifdef __SSE4_1__
#ifndef VSMC_SSE4_1
#define VSMC_SSE4_1 1
#endif
#endif

#ifdef __SSE4_2__
#ifndef VSMC_SSE4_2
#define VSMC_SSE4_2 1
#endif
#endif

#ifdef __AVX__
#ifndef VSMC_AVX
#define VSMC_AVX 1
#endif
#endif

#ifdef __AVX2__
#ifndef VSMC_AVX2
#define VSMC_AVX2 1
#endif
#endif

#endif // VSMC_HAS_INTRINSIC_FUNCTION

#ifndef VSMC_MMX
#define VSMC_MMX 0
#endif

#ifndef VSMC_SSE
#define VSMC_SSE 0
#endif

#ifndef VSMC_SSE2
#define VSMC_SSE2 0
#endif

#ifndef VSMC_SSE3
#define VSMC_SSE3 0
#endif

#ifndef VSMC_SSSE3
#define VSMC_SSSE3 0
#endif

#ifndef VSMC_SSE4_1
#define VSMC_SSE4_1 0
#endif

#ifndef VSMC_SSE4_2
#define VSMC_SSE4_2 0
#endif

#ifndef VSMC_AVX
#define VSMC_AVX 0
#endif

#ifndef VSMC_AVX2
#define VSMC_AVX2 0
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP
