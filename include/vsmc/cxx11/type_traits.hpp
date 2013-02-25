#ifndef VSMC_INTERNAL_CXX11_TYPE_TRAITS_HPP
#define VSMC_INTERNAL_CXX11_TYPE_TRAITS_HPP

namespace vsmc { namespace cxx11 {

template <typename T, T v>
struct integral_constant
{
    static const T value = v;
    typedef T value_type;
    typedef integral_constant<T, v> type;
    VSMC_CONSTEXPR operator value_type () const {return value;}
};

typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;

template <typename T, typename U> struct is_same :       public false_type {};
template <typename T>             struct is_same<T, T> : public true_type {};

template <typename T> struct is_reference       : public false_type {};
template <typename T> struct is_reference<T &>  : public true_type {};
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
template <typename T> struct is_reference<T &&> : public true_type {};
#endif

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

} } // namespace vsmc::cxx11

#endif // VSMC_INTERNAL_CXX11_TYPE_TRAITS_HPP
