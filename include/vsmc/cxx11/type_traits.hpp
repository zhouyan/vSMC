#ifndef VSMC_CXX11_TYPE_TRAITS_HPP
#define VSMC_CXX11_TYPE_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/assert.hpp>

namespace vsmc {

namespace cxx11 {

/// \defgroup CXX11Traits C++11 style type traits
/// \brief C++11 style type traits
/// \ingroup Traits
///
/// \details
/// Most of C++11/14 type traits are defined, except those require compiler
/// sepcific funtionalities.
///
/// \bug vsmc::cxx11::is_union is not correctly implemented. This type traits
/// need compiler support to be implemented. The current behavior is that
/// `is_union<T>` derives from vsmc::cxx11::false_type for any `T`.  In
/// addition, `is_class<T>` derives vsmc::cxx11::true_type if the template
/// parameter type is a union, while it should derive from
/// vsmc::cxx11::false_type
///
/// \todo The following traits are declared but not implemented yet
/// ~~~{.cpp}
/// template <typename> struct is_trivial;
/// template <typename> struct is_trivially_copyable;
/// template <typename> struct is_standard_layout;
/// template <typename> struct is_pod;
/// template <typename> struct is_literal_type;
/// template <typename> struct is_constructible;
/// template <typename> struct is_trivially_constructible;
/// template <typename> struct is_nothrow_constructible;
/// template <typename> struct is_default_constructible;
/// template <typename> struct is_trivially_default_constructible;
/// template <typename> struct is_nothrow_default_constructible;
/// template <typename> struct is_copy_constructible;
/// template <typename> struct is_trivially_copy_constructible;
/// template <typename> struct is_nothrow_copy_constructible;
/// template <typename> struct is_move_constructible;
/// template <typename> struct is_trivially_move_constructible;
/// template <typename> struct is_nothrow_move_constructible;
/// template <typename> struct is_assignable;
/// template <typename> struct is_trivially_assignable;
/// template <typename> struct is_nothrow_assignable;
/// template <typename> struct is_copy_assignable;
/// template <typename> struct is_trivially_copy_assignable;
/// template <typename> struct is_nothrow_copy_assignable;
/// template <typename> struct is_move_assignable;
/// template <typename> struct is_trivially_move_assignable;
/// template <typename> struct is_nothrow_move_assignable;
/// template <typename> struct is_destructible;
/// template <typename> struct is_trivially_destructible;
/// template <typename> struct is_nothrow_destructible;
/// template <typename> struct has_virtual_destructor;
/// template <typename> struct alignment_of;
/// template <typename> struct aligned_storage;
/// template <typename> struct aligned_union;
/// template <typename> struct common_type;
/// template <typename> struct underlying_type;
/// template <typename> struct result_of;
/// ~~~
/// @{

//////////////////////////////////////////////////////////////////////////////
// Forward declarations
//////////////////////////////////////////////////////////////////////////////

// Helper classes
template <typename T, T> struct integral_constant;

// Primary type categories
template <typename> struct is_void;
template <typename> struct is_null_pointer;
template <typename> struct is_integral;
template <typename> struct is_floating_point;
template <typename> struct is_array;
template <typename> struct is_enum;
template <typename> struct is_union;
template <typename> struct is_class;
template <typename> struct is_function;
template <typename> struct is_pointer;
template <typename> struct is_lvalue_reference;
template <typename> struct is_rvalue_reference;
template <typename> struct is_member_object_pointer;
template <typename> struct is_member_function_pointer;

// Composite type categories
template <typename> struct is_fundamental;
template <typename> struct is_arithmetic;
template <typename> struct is_scalar;
template <typename> struct is_object;
template <typename> struct is_compound;
template <typename> struct is_reference;
template <typename> struct is_member_pointer;

// Type properties
template <typename> struct is_const;
template <typename> struct is_volatile;
template <typename> struct is_trivial;
template <typename> struct is_trivially_copyable;
template <typename> struct is_standard_layout;
template <typename> struct is_pod;
template <typename> struct is_literal_type;
template <typename> struct is_empty;
template <typename> struct is_polymorphic;
template <typename> struct is_abstract;
template <typename> struct is_signed;
template <typename> struct is_unsigned;

// Supported operations
template <typename> struct is_constructible;
template <typename> struct is_trivially_constructible;
template <typename> struct is_nothrow_constructible;
template <typename> struct is_default_constructible;
template <typename> struct is_trivially_default_constructible;
template <typename> struct is_nothrow_default_constructible;
template <typename> struct is_copy_constructible;
template <typename> struct is_trivially_copy_constructible;
template <typename> struct is_nothrow_copy_constructible;
template <typename> struct is_move_constructible;
template <typename> struct is_trivially_move_constructible;
template <typename> struct is_nothrow_move_constructible;
template <typename> struct is_assignable;
template <typename> struct is_trivially_assignable;
template <typename> struct is_nothrow_assignable;
template <typename> struct is_copy_assignable;
template <typename> struct is_trivially_copy_assignable;
template <typename> struct is_nothrow_copy_assignable;
template <typename> struct is_move_assignable;
template <typename> struct is_trivially_move_assignable;
template <typename> struct is_nothrow_move_assignable;
template <typename> struct is_destructible;
template <typename> struct is_trivially_destructible;
template <typename> struct is_nothrow_destructible;
template <typename> struct has_virtual_destructor;

// Property queries
template <typename> struct alignment_of;
template <typename> struct rank;
template <typename, unsigned> struct extent;

// Type relations
template <typename, typename> struct is_same;
template <typename, typename> struct is_base_of;
template <typename, typename> struct is_convertible;

// Const-volatility specifiers
template <typename> struct remove_cv;
template <typename> struct remove_const;
template <typename> struct remove_volatile;
template <typename> struct add_cv;
template <typename> struct add_const;
template <typename> struct add_volatile;

// References
template <typename> struct remove_reference;
template <typename> struct add_lvalue_reference;
template <typename> struct add_rvalue_reference;

// Pointers
template <typename> struct remove_pointer;
template <typename> struct add_pointer;

// Sign modifiers
template <typename> struct make_signed;
template <typename> struct make_unsigned;

// Arrays
template <typename> struct remove_extent;
template <typename> struct remove_all_extents;

// Miscellaneous transformations
template <typename> struct aligned_storage;
template <typename> struct aligned_union;
template <typename> struct decay;
template <bool, typename> struct enable_if;
template <bool, typename, typename> struct conditional;
template <typename> struct common_type;
template <typename> struct underlying_type;
template <typename> struct result_of;

// Utilities
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T>
typename add_rvalue_reference<T>::type declval() VSMC_NOEXCEPT;
#else
template <typename T>
typename add_lvalue_reference<T>::type declval() VSMC_NOEXCEPT;
#endif

/// \defgroup CXX11TraitsHelper Helper classes
/// \brief Helper classes
/// \ingroup CXX11Traits
/// @{

// integral_constant
template <typename T, T v>
struct integral_constant
{
    typedef T value_type;
    typedef integral_constant<T, v> type;
    static VSMC_CONSTEXPR const T value = v;
    VSMC_CONSTEXPR operator value_type () const {return value;}
    VSMC_CONSTEXPR value_type operator() () const {return value;}
};
typedef integral_constant<bool, true>  true_type;
typedef integral_constant<bool, false> false_type;

namespace internal {
typedef char tp_test_true;
struct tp_test_false {char a[2];};
} // namespace vsmc::internal

/// @}

/// \defgroup CXX11TraitsPrimary Primary type categories
/// \brief Primary type categories
/// \ingroup CXX11Traits
/// @{

// is_void
namespace internal {
template <typename> struct is_void_impl       : public false_type {};
template <>         struct is_void_impl<void> : public true_type  {};
} // namespace vsmc::internal
template <typename T> struct is_void :
    public internal::is_void_impl<typename remove_cv<T>::type> {};

// is_null_pointer
namespace internal {
template <typename T> struct is_null_pointer_impl : public false_type {};
#if VSMC_HAS_CXX11_NULLPTR
template <> struct is_null_pointer_impl<std::nullptr_t> : public true_type {};
#endif
} // namespace vsmc::internal
template <typename T> struct is_null_pointer :
    public internal::is_null_pointer_impl<typename remove_cv<T>::type> {};

// is_integral
namespace internal {
template <typename> struct is_integral_impl : public false_type {};
template <> struct is_integral_impl<char          > : public true_type {};
template <> struct is_integral_impl<signed char   > : public true_type {};
template <> struct is_integral_impl<unsigned char > : public true_type {};
template <> struct is_integral_impl<wchar_t       > : public true_type {};
template <> struct is_integral_impl<short         > : public true_type {};
template <> struct is_integral_impl<unsigned short> : public true_type {};
template <> struct is_integral_impl<int           > : public true_type {};
template <> struct is_integral_impl<unsigned      > : public true_type {};
template <> struct is_integral_impl<long          > : public true_type {};
template <> struct is_integral_impl<unsigned long > : public true_type {};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct is_integral_impl<long long         > : public true_type {};
template <> struct is_integral_impl<unsigned long long> : public true_type {};
#endif
#if VSMC_HAS_CXX11_UNICODE_LITERALS
template <> struct is_integral_impl<char16_t> : public true_type {};
template <> struct is_integral_impl<char32_t> : public true_type {};
#endif
} // namespace vsmc::internal
template <typename T> struct is_integral :
    public internal::is_integral_impl<typename remove_cv<T>::type> {};

// is_floating_point
namespace internal {
template <typename> struct is_floating_point_impl : public false_type {};
template <> struct is_floating_point_impl<float      > : public true_type  {};
template <> struct is_floating_point_impl<double     > : public true_type  {};
template <> struct is_floating_point_impl<long double> : public true_type  {};
} // namespace vsmc::internal
template <typename T> struct is_floating_point :
    public internal::is_floating_point_impl<typename remove_cv<T>::type> {};

// is_array
template <typename> struct is_array : public false_type {};
template <typename T> struct is_array<T []> : public true_type {};
template <typename T, std::size_t N> struct is_array<T [N]> :
    public true_type {};

// is_enum
template <typename T> struct is_enum :
    public integral_constant<bool,
    !is_void<T>::value           && !is_integral<T>::value       &&
    !is_floating_point<T>::value && !is_pointer<T>::value        &&
    !is_reference<T>::value      && !is_member_pointer<T>::value &&
    !is_union<T>::value          && !is_class<T>::value          &&
    !is_function<T>::value> {};

// is_union
namespace internal {
template <typename> struct is_union_impl : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_union :
    public internal::is_union_impl<typename remove_cv<T>::type> {};

// is_class
namespace internal {
template <typename T> tp_test_true  is_class_test (int T::*);
template <typename>   tp_test_false is_class_test (...);
} // namespace vsmc::internal
template <typename T> struct is_class :
    public integral_constant<bool,
    sizeof(internal::is_class_test<T>(0)) == sizeof(internal::tp_test_true) &&
    !is_union<T>::value> {};

// is_function
namespace internal {
template <typename T> tp_test_true  is_function_test (T *);
template <typename>   tp_test_false is_function_test (...);
template <typename T> T &is_function_test_src ();
template <typename T, bool =
    is_class<T>::value     || is_union<T>::value  || is_void<T>::value ||
    is_reference<T>::value || is_null_pointer<T>::value>
struct is_function_impl :
    public integral_constant<bool,
    sizeof(is_function_test<T>(is_function_test_src<T>())) ==
    sizeof(tp_test_true)> {};
template <typename T> struct is_function_impl<T, true> : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_function :
    public internal::is_function_impl<T> {};

// is_pointer
namespace internal {
template <typename>   struct is_pointer_impl :      public false_type {};
template <typename T> struct is_pointer_impl<T *> : public true_type  {};
} // namespace vsmc::internal
template <typename T> struct is_pointer :
    public internal::is_pointer_impl<typename remove_cv<T>::type> {};

// is_lvalue_reference
template <typename>   struct is_lvalue_reference      : public false_type {};
template <typename T> struct is_lvalue_reference<T &> : public true_type  {};

// is_rvalue_reference
template <typename>   struct is_rvalue_reference       : public false_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct is_rvalue_reference<T &&> : public true_type  {};
#endif

// is_member_object_pointer
template <typename T> struct is_member_object_pointer :
    public integral_constant<bool,
    is_member_pointer<T>::value && !is_member_function_pointer<T>::value> {};

// is_member_function_pointer
namespace internal {
template <typename>
struct is_member_function_pointer_impl         : public false_type     {};
template <typename T, typename U>
struct is_member_function_pointer_impl<T U::*> : public is_function<T> {};
} // namespace vsmc::internal
template <typename T> struct is_member_function_pointer :
    public internal::is_member_function_pointer_impl<
    typename remove_cv<T>::type> {};

/// @}

/// \defgroup CXX11TraitsComposite Composite type categories
/// \brief Composite type categories
/// \ingroup CXX11Traits
/// @{

// is_fundamental
template <typename T> struct is_fundamental :
    public integral_constant<bool,
    is_void<T>::value || is_null_pointer<T>::value ||
    is_arithmetic<T>::value> {};

// is_arithmetic
template <typename T> struct is_arithmetic :
    public integral_constant<bool,
    is_integral<T>::value || is_floating_point<T>::value> {};

// is_scalar
template <typename T> struct is_scalar :
    public integral_constant<bool,
    is_arithmetic<T>::value || is_member_pointer<T>::value ||
    is_pointer<T>::value    || is_null_pointer<T>::value   ||
    is_enum<T>::value> {};

// is_object
template <typename T> struct is_object :
    public integral_constant<bool,
    is_scalar<T>::value || is_array<T>::value ||
    is_union<T>::value  || is_class<T>::value> {};

// is_compound
template <typename T> struct is_compound :
    public integral_constant<bool, !is_fundamental<T>::value> {};

// is_reference
template <typename T> struct is_reference       : public false_type {};
template <typename T> struct is_reference<T &>  : public true_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct is_reference<T &&> : public true_type {};
#endif

// is_member_pointer
namespace internal {
template <typename>
struct is_member_pointer_impl         : public false_type {};
template <typename T, typename U>
struct is_member_pointer_impl<T U::*> : public true_type  {};
} // namespace vsmc::internal
template <typename T> struct is_member_pointer :
    public internal::is_member_pointer_impl<typename remove_cv<T>::type> {};

/// @}

/// \defgroup CXX11TraitsProperties Type properties
/// \brief Type properties
/// \ingroup CXX11Traits
/// @{

// is_const
template <typename>   struct is_const          : public false_type {};
template <typename T> struct is_const<const T> : public true_type  {};

// is_volatile
template <typename>   struct is_volatile             : public false_type {};
template <typename T> struct is_volatile<volatile T> : public true_type  {};

// is_trivial
// is_trivially_copyable
// is_standard_layout
// is_pod

// is_literal_type
template <typename T> struct is_literal_type :
    public integral_constant<bool,
    is_scalar<typename remove_all_extents<T>::type>::value ||
    is_reference<typename remove_all_extents<T>::type>::value> {};

// is_empty
namespace internal {
template <typename is_empty_base>
struct is_empty_derived : public is_empty_base {double x;};
template <typename>
struct is_empty_standalone {double x;};
} // namespace vsmc::internal
template <typename T> struct is_empty :
    public integral_constant<bool,
    sizeof(internal::is_empty_derived<T>) ==
    sizeof(internal::is_empty_standalone<T>)> {};

// is_polymorphic
namespace internal {
template <typename T> tp_test_true is_polymorphic_test (
        typename enable_if<sizeof(static_cast<T *>(const_cast<void *>(
                    dynamic_cast<const volatile void *>(declval<T *>())
                    ))) != 0, int>::type);
template <typename> tp_test_false is_polymorphic_test (...);
} // namespace vsmc::internal
template <typename T> struct is_polymorphic :
    public integral_constant<bool,
    sizeof(internal::is_polymorphic_test<T>(0)) ==
    sizeof(internal::tp_test_true)> {};

// is_abstract
namespace internal {
template <typename T> tp_test_false is_abstract_test (T (*) [1]);
template <typename>   tp_test_true  is_abstract_test (...);
template <typename T, bool = is_class<T>::value> struct is_abstract_impl :
    public integral_constant<bool,
    sizeof(is_abstract_test<T>(0)) == sizeof(tp_test_true)> {};
template <typename T> struct is_abstract_impl<T, false> : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_abstract :
    public internal::is_abstract_impl<T> {};

// is_signed
namespace internal {
template <typename T, bool = is_integral<T>::value>
struct is_signed_num_impl :
    public integral_constant<bool, static_cast<T>(-1) < static_cast<T>(0)> {};
template <typename T>
struct is_signed_num_impl<T, false> : public true_type {};
template <typename T, bool = is_arithmetic<T>::value> struct is_signed_impl :
    public is_signed_num_impl<T> {};
template <typename T> struct is_signed_impl<T, false> : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_signed :
    public internal::is_signed_impl<T> {};

// is_unsigned
namespace internal {
template <typename T, bool = is_integral<T>::value>
struct is_unsigned_num_impl :
    public integral_constant<bool, static_cast<T>(0) < static_cast<T>(-1)> {};
template <typename T>
struct is_unsigned_num_impl<T, false> : public false_type {};
template <typename T, bool = is_arithmetic<T>::value> struct is_unsigned_impl :
    public is_unsigned_num_impl<T> {};
template <typename T> struct is_unsigned_impl<T, false> : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_unsigned :
    public internal::is_unsigned_impl<T> {};

/// @}

/// \defgroup CXX11TraitsSupport Supported operations
/// \brief Supported operations
/// \ingroup CXX11Traits
/// @{

// is_constructible
// is_trivially_constructible
// is_nothrow_constructible
// is_default_constructible
// is_trivially_default_constructible
// is_nothrow_default_constructible
// is_copy_constructible
// is_trivially_copy_constructible
// is_nothrow_copy_constructible
// is_move_constructible
// is_trivially_move_constructible
// is_nothrow_move_constructible
// is_assignable
// is_trivially_assignable
// is_nothrow_assignable
// is_copy_assignable
// is_trivially_copy_assignable
// is_nothrow_copy_assignable
// is_move_assignable
// is_trivially_move_assignable
// is_nothrow_move_assignable
// is_destructible
// is_trivially_destructible
// is_nothrow_destructible
// has_virtual_destructor

/// @}

/// \defgroup CXX11TraitsQueries Property queries
/// \brief Property queries
/// \ingroup CXX11Traits
/// @{

// alignment_of

// rank
template <typename> struct rank :
    public integral_constant<std::size_t, 0> {};
template <typename T> struct rank<T []> :
    public integral_constant<std::size_t, rank<T>::value + 1> {};
template <typename T, std::size_t N> struct rank<T [N]> :
    public integral_constant<std::size_t, rank<T>::value + 1> {};

// extent
template <typename, unsigned = 0> struct extent :
    public integral_constant<std::size_t, 0> {};
template <typename T> struct extent<T [], 0> :
    public integral_constant<std::size_t, 0> {};
template <typename T, unsigned I> struct extent<T [], I> :
    public integral_constant<std::size_t, extent<T, I - 1>::value> {};
template <typename T, std::size_t N> struct extent<T [N], 0> :
    public integral_constant<std::size_t, N> {};
template <typename T, std::size_t N, unsigned I> struct extent<T [N], I> :
    public integral_constant<std::size_t, extent<T, I - 1>::value> {};

/// @}

/// \defgroup CXX11TraitsRelations Type relations
/// \brief Type relations
/// \ingroup CXX11Traits
/// @{

// is_same
template <typename, typename> struct is_same       : public false_type {};
template <typename T>         struct is_same<T, T> : public true_type  {};

// is_base_of
namespace internal {
template <typename T>
struct is_base_of_dest {is_base_of_dest (const volatile T &);};
template <typename T>
struct is_base_of_src
{
    operator const volatile T &();
    template <typename U> operator const is_base_of_dest<U> &();
};

template <std::size_t> struct is_base_of_fail {typedef tp_test_false type;};
template <typename B, typename D>
typename is_base_of_fail<sizeof(
        is_base_of_dest<B>(declval<is_base_of_src<D> >())
        )>::type is_base_of_test (int);
template <typename B, typename D> tp_test_true is_base_of_test (...);
} // namespace vsmc::internal
template <typename B, typename D> struct is_base_of :
    public integral_constant<bool, is_class<B>::value &&
    sizeof(internal::is_base_of_test<B, D>(0)) ==
    sizeof(internal::tp_test_true)> {};

// is_convertible
namespace internal {
template <typename T> tp_test_true  is_convertible_test (T);
template <typename>   tp_test_false is_convertible_test (...);
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> T && is_convertible_test_src ();
#else
template <typename T>
typename remove_reference<T>::type &is_convertible_test_src ();
#endif

template <typename T,
         bool IsArray    = is_array<T>::value,
         bool IsFunction = is_function<T>::value,
         bool IsVoid     = is_void<T>::value>
struct is_convertible_afv {enum {value = 0};};
template <typename T>
struct is_convertible_afv<T, true, false, false> {enum {value = 1};};
template <typename T>
struct is_convertible_afv<T, false, true, false> {enum {value = 2};};
template <typename T>
struct is_convertible_afv<T, false, false, true> {enum {value = 3};};

template <typename T1, typename T2,
         unsigned T1_afv = is_convertible_afv<T1>::value,
         unsigned T2_afv = is_convertible_afv<T2>::value>
struct is_convertible_impl :
    public integral_constant<bool,
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    sizeof(is_convertible_test<T2>(is_convertible_test_src<T1>())) ==
    sizeof(tp_test_true)
#else
    sizeof(is_convertible_test<T2>(is_convertible_test_src<T1>())) ==
    sizeof(tp_test_true) && !(!is_function<T1>::value &&
            !is_reference<T1>::value && is_reference<T2>::value &&
            (!is_const<typename remove_reference<T2>::type>::value ||
             is_volatile<typename remove_reference<T2>::type>::value) &&
            (is_same<typename remove_cv<T1>::type, typename remove_cv<
             typename remove_reference<T2>::type>::type>::value ||
             is_base_of<typename remove_reference<T2>::type, T1>::value))
#endif
    > {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 1, 0> : public false_type {};
template <typename T1>
struct is_convertible_impl<T1, const T1 &, 1, 0> : public true_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T1>
struct is_convertible_impl<T1, T1 &&, 1, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, const T1 &&, 1, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, volatile T1 &&, 1, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, const volatile T1 &&, 1, 0> :
    public true_type {};
#endif

template <typename T1, typename T2>
struct is_convertible_impl<T1, T2 *, 1, 0> :
    public integral_constant<bool, is_convertible_impl<
    typename remove_all_extents<T1>::type *, T2 *>::value> {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2 *const, 1, 0> :
    public integral_constant<bool, is_convertible_impl<
    typename remove_all_extents<T1>::type *, T2 *const>::value> {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2 *volatile, 1, 0> :
    public integral_constant<bool, is_convertible_impl<
    typename remove_all_extents<T1>::type *, T2 *volatile>::value> {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2 *const volatile, 1, 0> :
    public integral_constant<bool, is_convertible_impl<
    typename remove_all_extents<T1>::type *, T2 *const volatile>::value> {};

template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 2, 0> : public false_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T1>
struct is_convertible_impl<T1, T1 &&, 2, 0> : public true_type {};
#endif
template <typename T1>
struct is_convertible_impl<T1, T1 &, 2, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, T1 *, 2, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, T1 *const, 2, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, T1 *volatile, 2, 0> : public true_type {};
template <typename T1>
struct is_convertible_impl<T1, T1 *const volatile, 2, 0> : public true_type {};

template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 3, 0> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 0, 1> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 1, 1> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 2, 1> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 3, 1> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 0, 2> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 1, 2> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 2, 2> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 3, 2> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 0, 3> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 1, 3> : public false_type {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 3, 3> : public true_type {};
} // namespace vsmc::internal
template <typename T1, typename T2> struct is_convertible :
    public internal::is_convertible_impl<T1, T2> {};

/// @}

/// \defgroup CXX11TraitsCV Const-volatility specifiers
/// \brief Const-volatility specifiers
/// \ingroup CXX11Traits
/// @{

// remove_const
template <typename T> struct remove_const          {typedef T type;};
template <typename T> struct remove_const<const T> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_const_t = typename remove_const<T>::type;
#endif

// remove_volatile
template <typename T> struct remove_volatile             {typedef T type;};
template <typename T> struct remove_volatile<volatile T> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_volatile_t =
    typename remove_volatile<T>::type;
#endif

// remove_cv
template <typename T> struct remove_cv
{typedef typename remove_volatile<typename remove_const<T>::type>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_cv_t = typename remove_cv<T>::type;
#endif

// add_const
namespace internal {
template <typename T, bool =
    is_reference<T>::value || is_function<T>::value || is_const<T>::value>
struct add_const_impl {typedef T type;};
template <typename T>
struct add_const_impl<T, false> {typedef const T type;};
} // namespace vsmc::internal
template <typename T> struct add_const
{typedef typename internal::add_const_impl<T>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_const_t = typename add_const<T>::type;
#endif

// add_volatile
namespace internal {
template <typename T, bool =
    is_reference<T>::value || is_function<T>::value || is_volatile<T>::value>
struct add_volatile_impl {typedef T type;};
template <typename T>
struct add_volatile_impl<T, false> {typedef volatile T type;};
} // namespace vsmc::internal
template <typename T> struct add_volatile
{typedef typename internal::add_volatile_impl<T>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_volatile_t = typename add_volatile<T>::type;
#endif

// add_cv
template <typename T> struct add_cv
{typedef typename add_volatile<typename add_const<T>::type>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_cv_t = typename add_cv<T>::type;
#endif

/// @}

/// \defgroup CXX11TraitsRef References
/// \brief References
/// \ingroup CXX11Traits
/// @{

// remove_reference
template <typename T> struct remove_reference       {typedef T type;};
template <typename T> struct remove_reference<T &>  {typedef T type;};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct remove_reference<T &&> {typedef T type;};
#endif
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_reference_t =
    typename remove_reference<T>::type;
#endif

// add_lvalue_reference
template <typename T> struct add_lvalue_reference      {typedef T &  type;};
template <typename T> struct add_lvalue_reference<T &> {typedef T &  type;};
template <> struct add_lvalue_reference<void>
{typedef void type;};
template <> struct add_lvalue_reference<const void>
{typedef const void type;};
template <> struct add_lvalue_reference<volatile void>
{typedef volatile void type;};
template <> struct add_lvalue_reference<const volatile void>
{typedef const volatile void type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_lvalue_reference_t =
    typename add_lvalue_reference<T>::type;
#endif

// add_rvalue_reference
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct add_rvalue_reference  {typedef T &&  type;};
template <> struct add_rvalue_reference<void>
{typedef void type;};
template <> struct add_rvalue_reference<const void>
{typedef const void type;};
template <> struct add_rvalue_reference<volatile void>
{typedef volatile void type;};
template <> struct add_rvalue_reference<const volatile void>
{typedef const volatile void type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_rvalue_reference_t =
    typename add_rvalue_reference<T>::type;
#endif
#endif

/// @}

/// \defgroup CXX11TraitsPtr Pointers
/// \brief Pointers
/// \ingroup CXX11Traits
/// @{

// remove_pointer
template <typename T> struct remove_pointer              {typedef T type;};
template <typename T> struct remove_pointer<T *>         {typedef T type;};
template <typename T> struct remove_pointer<T *const>    {typedef T type;};
template <typename T> struct remove_pointer<T *volatile> {typedef T type;};
template <typename T> struct remove_pointer<T *const volatile>
{typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_pointer_t =
    typename remove_pointer<T>::type;
#endif

// add_pointer
template <typename T> struct add_pointer
{typedef typename remove_reference<T>::type * type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using add_pointer_t = typename add_pointer<T>::type;
#endif

/// @}

/// \defgroup CXX11TraitsSign Sign modifiers
/// \brief Sign modifiers
/// \ingroup CXX11Traits
/// @{

namespace internal {
template <typename T, typename U,
         bool = is_const<typename remove_reference<T>::type>::value,
         bool = is_volatile<typename remove_reference<T>::type>::value>
struct apply_cv;
template <typename T, typename U> struct apply_cv<T, U, true, true>
{typedef const volatile U type;};
template <typename T, typename U> struct apply_cv<T, U, true, false>
{typedef const U type;};
template <typename T, typename U> struct apply_cv<T, U, false, true>
{typedef volatile U type;};
template <typename T, typename U> struct apply_cv<T, U, false, false>
{typedef U type;};
template <typename T, typename U> struct apply_cv<T &, U, true, true>
{typedef const volatile U & type;};
template <typename T, typename U> struct apply_cv<T &, U, true, false>
{typedef const U & type;};
template <typename T, typename U> struct apply_cv<T &, U, false, true>
{typedef volatile U & type;};
template <typename T, typename U> struct apply_cv<T &, U, false, false>
{typedef U & type;};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T, typename U> struct apply_cv<T &&, U, true, true>
{typedef const volatile U && type;};
template <typename T, typename U> struct apply_cv<T &&, U, true, false>
{typedef const U && type;};
template <typename T, typename U> struct apply_cv<T &&, U, false, true>
{typedef volatile U && type;};
template <typename T, typename U> struct apply_cv<T &&, U, false, false>
{typedef U && type;};
#endif
} // namespace vsmc::internal

// make_signed
namespace internal {
template <typename T, bool = is_integral<T>::value || is_enum<T>::value>
struct make_signed_impl {};
template <> struct make_signed_impl<signed short, true>
{typedef signed short type;};
template <> struct make_signed_impl<signed int, true>
{typedef signed int type;};
template <> struct make_signed_impl<signed long, true>
{typedef signed long type;};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct make_signed_impl<signed long long, true>
{typedef signed long long type;};
#endif
template <> struct make_signed_impl<unsigned short, true>
{typedef signed short type;};
template <> struct make_signed_impl<unsigned int, true>
{typedef signed int type;};
template <> struct make_signed_impl<unsigned long, true>
{typedef signed long type;};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct make_signed_impl<unsigned long long, true>
{typedef signed long long type;};
#endif
} // namespace vsmc::internal
template <typename T>
struct make_signed
{
    typedef typename internal::apply_cv<T,
            typename internal::make_signed_impl<typename remove_cv<
                typename remove_reference<T>::type>::type>::type>::type type;
};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using make_signed_t = typename make_signed<T>::type;
#endif

// make_unsigned
namespace internal {
template <typename T, bool = is_integral<T>::value || is_enum<T>::value>
struct make_unsigned_impl {};
template <> struct make_unsigned_impl<signed short, true>
{typedef unsigned short type;};
template <> struct make_unsigned_impl<signed int, true>
{typedef unsigned int type;};
template <> struct make_unsigned_impl<signed long, true>
{typedef unsigned long type;};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct make_unsigned_impl<signed long long, true>
{typedef unsigned long long type;};
#endif
template <> struct make_unsigned_impl<unsigned short, true>
{typedef unsigned short type;};
template <> struct make_unsigned_impl<unsigned int, true>
{typedef unsigned int type;};
template <> struct make_unsigned_impl<unsigned long, true>
{typedef unsigned long type;};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct make_unsigned_impl<unsigned long long, true>
{typedef unsigned long long type;};
#endif
} // namespace vsmc::internal
template <typename T>
struct make_unsigned
{
    typedef typename internal::apply_cv<T,
            typename internal::make_unsigned_impl<typename remove_cv<
                typename remove_reference<T>::type>::type>::type>::type type;
};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using make_unsigned_t = typename make_unsigned<T>::type;
#endif

/// @}

/// \defgroup CXX11TraitsArray Arrays
/// \brief Arrays
/// \ingroup CXX11Traits
/// @{

// remove_extent
template <typename T>
struct remove_extent {typedef T type;};
template <typename T>
struct remove_extent<T []> {typedef T type;};
template <typename T, std::size_t N>
struct remove_extent<T [N]> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_extent_t = typename remove_extent<T>::type;
#endif

// remove_all_extents
template <typename T> struct remove_all_extents {typedef T type;};
template <typename T> struct remove_all_extents<T []>
{typedef typename remove_all_extents<T>::type type;};
template <typename T, std::size_t N> struct remove_all_extents<T [N]>
{typedef typename remove_all_extents<T>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_all_extents_t =
    typename remove_all_extents<T>::type;
#endif

/// @}

/// \defgroup CXX11TraitsMisc Miscellaneous transformations
/// \brief Miscellaneous transformations
/// \ingroup CXX11Traits
/// @{

// aligned_storage
// aligned_union

// decay
template <typename T>
struct decay
{
    private :

    typedef typename remove_reference<T>::type U;

    public :

    typedef
        typename conditional<
            is_array<U>::value, typename remove_extent<U>::type *,
        typename conditional<
            is_function<U>::value, typename add_pointer<U>::type,
        typename remove_cv<U>::type>::type>::type type;
};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using decay_t = typename decay<T>::type;
#endif

// enable_if
template <bool, typename = void> struct enable_if {};
template <typename T> struct enable_if<true, T> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <bool B, typename T = void> using enable_if_t =
    typename enable_if<B, T>::type;
#endif

// conditional
template <bool, typename, typename F> struct conditional {typedef F type;};
template <typename T, typename F> struct conditional<true, T, F>
{typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <bool B, typename T, typename F> using conditional_t =
    typename conditional<B, T, F>::type;
#endif

// common_type
// underlying_type
// result_of

/// @}

//////////////////////////////////////////////////////////////////////////////
// Utilities
//////////////////////////////////////////////////////////////////////////////

/// \defgroup CXX11TraitsUtility Utilities
/// \brief Utilities
/// \ingroup CXX11Traits
/// @{

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T>
inline typename remove_reference<T>::type &&move (T &&t) VSMC_NOEXCEPT;

template <typename T>
inline typename remove_reference<T>::type &&move (T &&t) VSMC_NOEXCEPT
{
    typedef typename remove_reference<T>::type U;
    return static_cast<U &&>(t);
}

template <typename T>
inline T &&forward (typename remove_reference<T>::type &t) VSMC_NOEXCEPT
{return static_cast<T &&>(t);}

template <typename T>
inline T &&forward (typename remove_reference<T>::type &&t) VSMC_NOEXCEPT
{
    VSMC_STATIC_ASSERT_FORWARD_RVALUE;
    return static_cast<T&&>(t);
}
#endif

/// @}

/// @}

} // namespace vsmc::cxx11

} // namespace vsmc

#endif // VSMC_CXX11_TYPE_TRAITS_HPP
