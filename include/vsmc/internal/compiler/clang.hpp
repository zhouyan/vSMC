#ifndef VSMC_INTERNAL_COMPILER_CLANG_HPP
#define VSMC_INTERNAL_COMPILER_CLANG_HPP

#if __cplusplus >= 201103L

// C++11 language features

#if __has_feature(cxx_access_control_sfinae)
#ifndef VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE
#define VSMC_HAS_CXX11_ACCESS_CONTROL_SFINAE 1
#endif
#endif

#if __has_feature(cxx_alias_templates)
#ifndef VSMC_HAS_CXX11_ALIAS_TEMPLATES
#define VSMC_HAS_CXX11_ALIAS_TEMPLATES 1
#endif
#endif

#if __has_feature(cxx_alignas)
#ifndef VSMC_HAS_CXX11_ALIGNAS
#define VSMC_HAS_CXX11_ALIGNAS 1
#endif
#endif

#if __has_feature(cxx_attributes)
#ifndef VSMC_HAS_CXX11_ATTRIBUTES
#define VSMC_HAS_CXX11_ATTRIBUTES 1
#endif
#endif

#if __has_feature(cxx_auto_type)
#ifndef VSMC_HAS_CXX11_AUTO_TYPE
#define VSMC_HAS_CXX11_AUTO_TYPE 1
#endif
#endif

#if __has_feature(cxx_constexpr)
#ifndef VSMC_HAS_CXX11_CONSTEXPR
#define VSMC_HAS_CXX11_CONSTEXPR 1
#endif
#endif

#if __has_feature(cxx_decltype)
#ifndef VSMC_HAS_CXX11_DECLTYPE
#define VSMC_HAS_CXX11_DECLTYPE 1
#endif
#endif

#if __has_feature(cxx_default_function_template_args)
#ifndef VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_DEFAULT_FUNCTION_TEMPLATE_ARGS 1
#endif
#endif

#if __has_feature(cxx_defaulted_functions)
#ifndef VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
#define VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS 1
#endif
#endif

#if __has_feature(cxx_delegating_constructors)
#ifndef VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS
#define VSMC_HAS_CXX11_DELEGATING_CONSTRUCTORS 1
#endif
#endif

#if __has_feature(cxx_deleted_functions)
#ifndef VSMC_HAS_CXX11_DELETED_FUNCTIONS
#define VSMC_HAS_CXX11_DELETED_FUNCTIONS 1
#endif
#endif

#if __has_feature(cxx_explicit_conversions)
#ifndef VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS 1
#endif
#endif

#if __has_feature(cxx_generalized_initializers)
#ifndef VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS
#define VSMC_HAS_CXX11_GENERALIZED_INITIALIZERS 1
#endif
#endif

#if __has_feature(cxx_implicit_moves)
#ifndef VSMC_HAS_CXX11_IMPLICIT_MOVES
#define VSMC_HAS_CXX11_IMPLICIT_MOVES 1
#endif
#endif

#if __has_feature(cxx_inheriting_constructors)
#ifndef VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS
#define VSMC_HAS_CXX11_INHERITING_CONSTRUCTORS 1
#endif
#endif

#if __has_feature(cxx_inline_namespaces)
#ifndef VSMC_HAS_CXX11_INLINE_NAMESPACES
#define VSMC_HAS_CXX11_INLINE_NAMESPACES 1
#endif
#endif

#if __has_feature(cxx_lambdas)
#ifndef VSMC_HAS_CXX11_LAMBDAS
#define VSMC_HAS_CXX11_LAMBDAS 1
#endif
#endif

#if __has_feature(cxx_local_type_template_args)
#ifndef VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS
#define VSMC_HAS_CXX11_LOCAL_TYPE_TEMPLATE_ARGS 1
#endif
#endif

#if __has_feature(cxx_noexcept)
#ifndef VSMC_HAS_CXX11_NOEXCEPT
#define VSMC_HAS_CXX11_NOEXCEPT 1
#endif
#endif

#if __has_feature(cxx_nonstatic_member_init)
#ifndef VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT
#define VSMC_HAS_CXX11_NONSTATIC_MEMBER_INIT 1
#endif
#endif

#if __has_feature(cxx_nullptr)
#ifndef VSMC_HAS_CXX11_NULLPTR
#define VSMC_HAS_CXX11_NULLPTR 1
#endif
#endif

#if __has_feature(cxx_override_control)
#ifndef VSMC_HAS_CXX11_OVERRIDE_CONTROL
#define VSMC_HAS_CXX11_OVERRIDE_CONTROL 1
#endif
#endif

#if __has_feature(cxx_range_for)
#ifndef VSMC_HAS_CXX11_RANGE_FOR
#define VSMC_HAS_CXX11_RANGE_FOR 1
#endif
#endif

#if __has_feature(cxx_raw_string_literals)
#ifndef VSMC_HAS_CXX11_RAW_STRING_LITERALS
#define VSMC_HAS_CXX11_RAW_STRING_LITERALS 1
#endif
#endif

#if __has_feature(cxx_reference_qualified_functions)
#ifndef VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS
#define VSMC_HAS_CXX11_REFERENCE_QUALIFIED_FUNCTIONS 1
#endif
#endif

#if __has_feature(cxx_rvalue_references)
#ifndef VSMC_HAS_CXX11_RVALUE_REFERENCES
#define VSMC_HAS_CXX11_RVALUE_REFERENCES 1
#endif
#endif

#if __has_feature(cxx_static_assert)
#ifndef VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_HAS_CXX11_STATIC_ASSERT 1
#endif
#endif

#if __has_feature(cxx_strong_enums)
#ifndef VSMC_HAS_CXX11_STRONG_ENUMS
#define VSMC_HAS_CXX11_STRONG_ENUMS 1
#endif
#endif

#if __has_feature(cxx_thread_local)
#ifndef VSMC_HAS_CXX11_THREAD_LOCAL
#define VSMC_HAS_CXX11_THREAD_LOCAL 1
#endif
#endif

#if __has_feature(cxx_trailing_return)
#ifndef VSMC_HAS_CXX11_TRAILING_RETURN
#define VSMC_HAS_CXX11_TRAILING_RETURN 1
#endif
#endif

#if __has_feature(cxx_unicode_literals)
#ifndef VSMC_HAS_CXX11_UNICODE_LITERALS
#define VSMC_HAS_CXX11_UNICODE_LITERALS 1
#endif
#endif

#if __has_feature(cxx_unrestricted_unions)
#ifndef VSMC_HAS_CXX11_UNRESTRICTED_UNIONS
#define VSMC_HAS_CXX11_UNRESTRICTED_UNIONS 1
#endif
#endif

#if __has_feature(cxx_user_literals)
#ifndef VSMC_HAS_CXX11_USER_LITERALS
#define VSMC_HAS_CXX11_USER_LITERALS 1
#endif
#endif

#if __has_feature(cxx_variadic_templates)
#ifndef VSMC_HAS_CXX11_VARIADIC_TEMPLATES
#define VSMC_HAS_CXX11_VARIADIC_TEMPLATES 1
#endif
#endif

// C++11 library features

#if defined(_LIBCPP_VERSION)

#define VSMC_LIBCPP_NONEXIST 1000000UL

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_HAS_CXX11LIB_CHRONO 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_CMATH
#define VSMC_HAS_CXX11LIB_CMATH 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_HAS_CXX11LIB_FUNCTIONAL 1
#endif
#endif

#if _LIBCPP_VERSION >= VSMC_LIBCPP_NONEXIST
#ifndef VSMC_HAS_CXX11LIB_FUTURE
#define VSMC_HAS_CXX11LIB_FUTURE 0
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_MUTEX
#define VSMC_HAS_CXX11LIB_MUTEX 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_RANDOM
#define VSMC_HAS_CXX11LIB_RANDOM 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
#define VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_THREAD
#define VSMC_HAS_CXX11LIB_THREAD 1
#endif
#endif

#if _LIBCPP_VERSION >= 1101
#ifndef VSMC_HAS_CXX11LIB_TUPLE
#define VSMC_HAS_CXX11LIB_TUPLE 1
#endif
#endif

#endif // defined(_LIBCPP_VERSION)

#endif // __cplusplus >= 201103L

// C99 library features

#ifndef VSMC_HAS_C99LIB_MATH
#define VSMC_HAS_C99LIB_MATH 1
#endif

#endif // VSMC_INTERNAL_COMPILER_CLANG_HPP
