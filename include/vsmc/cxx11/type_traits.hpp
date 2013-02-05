#ifndef VSMC_CXX11_TYPE_TRATIS_HPP
#define VSMC_CXX11_TYPE_TRATIS_HPP

#include <vsmc/internal/config.hpp>

namespace vsmc { namespace cxx11 {

// integral constant
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

// is_same
template <typename T, typename U> struct is_same : public false_type {};
template <typename T> struct is_same<T, T> : public true_type {};

// enable if
template<bool, class T = void> struct enable_if {};
template<class T> struct enable_if<true, T> {typedef T type;};

} } // namespace vsmc::cxx11

#endif // VSMC_CXX11_TYPE_TRATIS_HPP
