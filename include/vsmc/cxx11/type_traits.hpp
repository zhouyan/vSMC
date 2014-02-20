#ifndef VSMC_CXX11_TYPE_TRAITS_HPP
#define VSMC_CXX11_TYPE_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/assert.hpp>

namespace vsmc {

namespace cxx11 {

/// \defgroup CPP11Traits C++11 style type traits
/// \brief C++11 style type traits
/// \ingroup Traits
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
template <typename> struct is_trivally_copyable;
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
template <typename> struct is_trivally_constructible;
template <typename> struct is_nothrow_constructible;
template <typename> struct is_default_constructible;
template <typename> struct is_trivally_default_constructible;
template <typename> struct is_nothrow_default_constructible;
template <typename> struct is_copy_constructible;
template <typename> struct is_trivally_copy_constructible;
template <typename> struct is_nothrow_copy_constructible;
template <typename> struct is_move_constructible;
template <typename> struct is_trivally_move_constructible;
template <typename> struct is_nothrow_move_constructible;
template <typename> struct is_assignable;
template <typename> struct is_trivally_assignable;
template <typename> struct is_nothrow_assignable;
template <typename> struct is_copy_assignable;
template <typename> struct is_trivally_copy_assignable;
template <typename> struct is_nothrow_copy_assignable;
template <typename> struct is_move_assignable;
template <typename> struct is_trivally_move_assignable;
template <typename> struct is_nothrow_move_assignable;
template <typename> struct is_destructible;
template <typename> struct is_trivally_destructible;
template <typename> struct is_nothrow_destructible;
template <typename> struct has_virtual_destructor;

// Property queries
template <typename> struct alignment_of;
template <typename> struct rank;
template <typename> struct extent;

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
template <typename> struct conditional;
template <typename> struct common_type;
template <typename> struct underlying_type;
template <typename> struct result_of;

//////////////////////////////////////////////////////////////////////////////
// Helper classes
//////////////////////////////////////////////////////////////////////////////

// integral_constant
template <typename T, T v>
struct integral_constant
{
    static VSMC_CONSTEXPR const T value = v;
    typedef T value_type;
    typedef integral_constant<T, v> type;
    VSMC_CONSTEXPR operator value_type () const {return value;}
    VSMC_CONSTEXPR value_type operator() () const {return value;}
};
typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;

//////////////////////////////////////////////////////////////////////////////
// Constant-volatily specifiers
//////////////////////////////////////////////////////////////////////////////

// is_const
template <typename T> struct is_const :          public false_type {};
template <typename T> struct is_const<const T> : public true_type {};

// is_volatile
template <typename T> struct is_volatile :             public false_type {};
template <typename T> struct is_volatile<volatile T> : public true_type {};

// remove_cv
template <typename T> struct remove_const          {typedef T type;};
template <typename T> struct remove_const<const T> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_const_t = typename remove_const<T>::type;
#endif

template <typename T> struct remove_volatile             {typedef T type;};
template <typename T> struct remove_volatile<volatile T> {typedef T type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_volatile_t =
typename remove_volatile<T>::type;
#endif

template <typename T> struct remove_cv
{typedef typename remove_volatile<typename remove_const<T>::type>::type type;};
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <typename T> using remove_cv_t = typename remove_cv<T>::type;
#endif

//////////////////////////////////////////////////////////////////////////////
// Refrences
//////////////////////////////////////////////////////////////////////////////

// is_lvalue_reference
template <typename T> struct is_lvalue_reference :      public false_type {};
template <typename T> struct is_lvalue_reference<T &> : public true_type {};

// is_rvalue_reference
template <typename T> struct is_rvalue_reference :       public false_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct is_rvalue_reference<T &&> : public true_type {};
#endif

// is_reference
template <typename T> struct is_reference       : public false_type {};
template <typename T> struct is_reference<T &>  : public true_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct is_reference<T &&> : public true_type {};
#endif

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

//////////////////////////////////////////////////////////////////////////////
// Pointers
//////////////////////////////////////////////////////////////////////////////

// is_pointer
namespace internal {
template <typename T> struct is_pointer_impl :      public false_type {};
template <typename T> struct is_pointer_impl<T *> : public true_type {};
} // namespace vsmc::internal
template <typename T> struct is_pointer :
    public internal::is_pointer_impl<typename remove_cv<T>::type> {};

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

//////////////////////////////////////////////////////////////////////////////
// Primary type categories
//////////////////////////////////////////////////////////////////////////////

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
template <> struct is_integral_impl<char>           : public true_type {};
template <> struct is_integral_impl<signed char>    : public true_type {};
template <> struct is_integral_impl<unsigned char>  : public true_type {};
template <> struct is_integral_impl<wchar_t>        : public true_type {};
template <> struct is_integral_impl<short>          : public true_type {};
template <> struct is_integral_impl<unsigned short> : public true_type {};
template <> struct is_integral_impl<int>            : public true_type {};
template <> struct is_integral_impl<unsigned>       : public true_type {};
template <> struct is_integral_impl<long>           : public true_type {};
template <> struct is_integral_impl<unsigned long>  : public true_type {};
#if VSMC_HAS_CXX11_LONG_LONG
template <> struct is_integral_impl<long long>          : public true_type {};
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
template <> struct is_floating_point_impl<float>       : public true_type  {};
template <> struct is_floating_point_impl<double>      : public true_type  {};
template <> struct is_floating_point_impl<long double> : public true_type  {};
} // namespace vsmc::internal
template <typename T> struct is_floating_point :
    public internal::is_floating_point_impl<typename remove_cv<T>::type> {};

// is_array
template <typename>   struct is_array       : public false_type {};
template <typename T> struct is_array<T []> : public true_type  {};
template <typename T, std::size_t N>
struct is_array<T [N]> : public true_type {};

// remove_extent
template <typename T> struct remove_extent       {typedef T type;};
template <typename T> struct remove_extent<T []> {typedef T type;};
template <typename T, std::size_t N> struct remove_extent<T [N]>
{typedef T type;};
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

// is_union
namespace internal {
template <typename> struct is_union_impl : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_union :
    public internal::is_union_impl<typename remove_cv<T>::type> {};

// is_class
namespace internal {
typedef char is_class_true;
struct is_class_false {char a; char b;};
template <typename T> inline is_class_true  is_class_test (int T::*);
template <typename T> inline is_class_false is_class_test (...);
} // namespace vsmc::internal
template <typename T> struct is_class : public integral_constant<bool,
    sizeof(internal::is_class_test<T>(0)) == sizeof(internal::is_class_true)
                                        && !is_union<T>::value> {};

// is_function
namespace internal {
typedef char is_function_true;
struct is_function_false {char a; char b;};
template <typename T> inline is_function_true  is_function_test (T *);
template <typename T> inline is_function_false is_function_test (...);
template <typename T> inline T &is_function_test_src ();
template <typename T, bool =
    is_class<T>::value     || is_union<T>::value  || is_void<T>::value ||
    is_reference<T>::value || is_null_pointer<T>::value>
struct is_function_impl : public integral_constant<bool,
    sizeof(is_function_test<T>(is_function_test_src<T>())) ==
                          sizeof(is_function_true)> {};
template <typename T> struct is_function_impl<T, true> : public false_type {};
} // namespace vsmc::internal
template <typename T> struct is_function :
    public internal::is_function_impl<T> {};

//////////////////////////////////////////////////////////////////////////////
// Member pointer
//////////////////////////////////////////////////////////////////////////////

// is_member_pointer
namespace internal {
template <typename> struct is_member_pointer_impl : public false_type {};
template <typename T, typename U> struct is_member_pointer_impl<T U::*> :
    public true_type {};
} // namespace vsmc::internal
template <typename T> struct is_member_pointer :
    public internal::is_member_pointer_impl<typename remove_cv<T>::type> {};

// is_member_function_pointer
namespace internal {
template <typename>
struct is_member_function_pointer_impl : public false_type {};
template <typename T, typename U>
struct is_member_function_pointer_impl<T U::*> : public is_function<T> {};
} // namespace vsmc::internal
template <typename T> struct is_member_function_pointer :
    public internal::is_member_function_pointer_impl<
    typename remove_cv<T>::type> {};

// is_member_object_pointer
template <typename T> struct is_member_object_pointer :
    public integral_constant<bool,
    is_member_pointer<T>::value && !is_member_function_pointer<T>::value> {};

//////////////////////////////////////////////////////////////////////////////
// Composite type categories
//////////////////////////////////////////////////////////////////////////////

// is_enum
template <typename T> struct is_enum :
    public integral_constant<bool,
    !is_void<T>::value           && !is_integral<T>::value       &&
    !is_floating_point<T>::value && !is_pointer<T>::value        &&
    !is_reference<T>::value      && !is_member_pointer<T>::value &&
    !is_union<T>::value          && !is_class<T>::value          &&
    !is_function<T>::value> {};

// is_arithmetic
template <typename T> struct is_arithmetic : public integral_constant<bool,
    is_integral<T>::value || is_floating_point<T>::value> {};

// is_fundamental
template <typename T> struct is_fundamental : public integral_constant<bool,
    is_void<T>::value || is_null_pointer<T>::value || is_arithmetic<T>::value>
{};

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
template <typename T> struct is_compound : public integral_constant<bool,
    !is_fundamental<T>::value> {};

//////////////////////////////////////////////////////////////////////////////
// Add cv
//////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////
// Type properties
//////////////////////////////////////////////////////////////////////////////

// is_trivial
// is_trivially_copyable
// is_standard_layout
// is_pod
// is_literal_type
// is_empty
// is_polymorphic
// is_abstract
// is_signed
// is_unsigned

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
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

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T>
typename add_rvalue_reference<T>::type declval() VSMC_NOEXCEPT;
#else
template <typename T>
typename add_lvalue_reference<T>::type declval() VSMC_NOEXCEPT;
#endif

// enable_if
template <bool, typename = void> struct enable_if {};
template <typename T>            struct enable_if<true, T> {typedef T type;};

//////////////////////////////////////////////////////////////////////////////
// Type relations
//////////////////////////////////////////////////////////////////////////////

// is_same
template <typename T, typename U> struct is_same :       public false_type {};
template <typename T>             struct is_same<T, T> : public true_type {};

// is_base_of
namespace internal {
template <typename T> struct is_base_of_dest
{is_base_of_dest (const volatile T &);};
template <typename T> struct is_base_of_src
{
    operator const volatile T &();
    template <typename U> operator const is_base_of_dest<U> &();
};

template <std::size_t> struct is_base_of_true {typedef char type;};
struct is_base_of_false {char a; char b;};
// if convertible to base, then ambiguous
// otherwise this is the overloading match
template <typename B, typename D>
inline typename is_base_of_true<sizeof(
        is_base_of_dest<B>(declval<is_base_of_src<D> >())
        )>::type is_base_of_test (int);
template <typename B, typename D>

inline is_base_of_false is_base_of_test (...);
} // namespace vsmc::internal
template <typename B, typename D> struct is_base_of :
    public integral_constant<bool, is_class<B>::value &&
    sizeof(internal::is_base_of_test<B, D>(0)) ==
    sizeof(internal::is_base_of_false)> {};

// is_convertible
namespace internal {
typedef char is_convertible_true;
struct is_convertible_false {char a; char b;};
template <typename T> is_convertible_true  is_convertible_test (T);
template <typename>   is_convertible_false is_convertible_test (...);
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
    sizeof(is_convertible_true)
#else
    sizeof(is_convertible_test<T2>(is_convertible_test_src<T1>())) ==
    sizeof(is_convertible_true) && !(!is_function<T1>::value &&
            !is_reference<T1>::value && is_reference<T2>::value &&
            (!is_const<typename remove_reference<T2>::type>::value ||
             is_volatile<typename remove_reference<T2>::type>::value) &&
            (is_same<typename remove_cv<T1>::type, typename remove_cv<
             typename remove_reference<T2>::type>::type>::value ||
             is_base_of<typename remove_reference<T2>::type, T1>::value))
#endif
    > {};
template <typename T1, typename T2>
struct is_convertible_impl<T1, T2, 1, 0> : false_type {};
template <typename T1>
struct is_convertible_impl<T1, const T1 &, 1, 0> : true_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T1>
struct is_convertible_impl<T1, T1 &&, 1, 0> : true_type {};
template <typename T1>
struct is_convertible_impl<T1, const T1 &&, 1, 0> : true_type {};
template <typename T1>
struct is_convertible_impl<T1, volatile T1 &&, 1, 0> : true_type {};
template <typename T1>
struct is_convertible_impl<T1, const volatile T1 &&, 1, 0> : true_type {};
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

} // namespace vsmc::cxx11

} // namespace vsmc

#endif // VSMC_CXX11_TYPE_TRAITS_HPP
