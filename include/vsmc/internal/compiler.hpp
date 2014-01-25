#ifndef VSMC_INTERNAL_COMPILER_HPP
#define VSMC_INTERNAL_COMPILER_HPP

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

//  C++11 language features

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

/// \brief C++11 `<random>`
/// \ingroup Compiler
#ifndef VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 0
#endif

#endif // VSMC_INTERNAL_COMPILER_HPP
