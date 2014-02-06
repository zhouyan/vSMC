#ifndef VSMC_CXX11_TYPE_TRAITS_HPP
#define VSMC_CXX11_TYPE_TRAITS_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>

namespace vsmc {

namespace cxx11 {

/// \defgroup CPP11Traits C++11 style type traits
/// \brief C++11 style type traits
/// \ingroup Traits
/// @{

// integral_constant
template <typename T, T v>
struct integral_constant
{
    static VSMC_CONSTEXPR const T value = v;
    typedef T value_type;
    typedef integral_constant<T, v> type;
    VSMC_CONSTEXPR operator value_type () const {return value;}
};
typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;

// enable_if
template <bool, typename T = void> struct enable_if {};
template <typename T>              struct enable_if<true, T> {typedef T type;};

// is_same
template <typename T, typename U> struct is_same :       public false_type {};
template <typename T>             struct is_same<T, T> : public true_type {};

// is_const
template <typename T> struct is_const :          public false_type {};
template <typename T> struct is_const<const T> : public true_type {};

// is_volatile
template <typename T> struct is_volatile :             public false_type {};
template <typename T> struct is_volatile<volatile T> : public true_type {};

// remove_cv
template <typename T> struct remove_const          {typedef T type;};
template <typename T> struct remove_const<const T> {typedef T type;};

template <typename T> struct remove_volatile             {typedef T type;};
template <typename T> struct remove_volatile<volatile T> {typedef T type;};

template <typename T> struct remove_cv
{typedef typename remove_volatile<typename remove_const<T>::type>::type type;};

// add_cv
template <typename T> struct add_const {typedef const T type;};

template <typename T> struct add_volatile {typedef volatile T type;};

template <typename T> struct add_cv
{typedef typename add_volatile<typename add_const<T>::type>::type type;};

// is_void
template <typename T> struct is_void :
    public integral_constant<bool,
    is_same<T, typename remove_cv<T>::type>::value> {};

// is_array
template <typename T> struct is_array :      public false_type {};
template <typename T> struct is_array<T[]> : public true_type {};
template <typename T, std::size_t N>
struct is_array<T[N]> : public true_type {};

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

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T>
inline typename remove_reference<T>::type &&move (T &&t) VSMC_NOEXCEPT
{
    typedef typename remove_reference<T>::type U;
    return static_cast<U &&>(t);
}
#endif

// is_pointer
namespace is_pointer_impl {
template <typename T> struct helper :      public false_type {};
template <typename T> struct helper<T *> : public true_type {};
}
template <typename T> struct is_pointer :
    public is_pointer_impl::helper<typename remove_cv<T>::type> {};

// remove_pointer
template <typename T> struct remove_pointer              {typedef T type;};
template <typename T> struct remove_pointer<T *>         {typedef T type;};
template <typename T> struct remove_pointer<T *const>    {typedef T type;};
template <typename T> struct remove_pointer<T *volatile> {typedef T type;};
template <typename T> struct remove_pointer<T *const volatile>
{typedef T type;};

// add_pointer
template <typename T> struct add_pointer
{typedef typename remove_reference<T>::type * type;};

// is_floating_point
namespace is_floating_point_impl {
template <typename T> struct helper :              public false_type {};
template <>           struct helper<float> :       public true_type {};
template <>           struct helper<double> :      public true_type {};
template <>           struct helper<long double> : public true_type {};
}
template <typename T> struct is_floating_point :
    public is_floating_point_impl::helper<typename remove_cv<T>::type> {};

/// @}

} // namespace vsmc::cxx11

} // namespace vsmc

#endif // VSMC_CXX11_TYPE_TRAITS_HPP
