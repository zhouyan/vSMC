#ifndef VSMC_MATH_INTEGER_FUNCTION_HPP
#define VSMC_MATH_INTEGER_FUNCTION_HPP

#include <vsmc/cxx11/type_traits.hpp>

#ifndef VSMC_MATH_INTEGER_FUNCTION_UINT
#if VSMC_HAS_CXX11_LONG_LONG
#define VSMC_MATH_INTEGER_FUNCTION_INT long long
#define VSMC_MATH_INTEGER_FUNCTION_UINT unsigned long long
#else
#define VSMC_MATH_INTEGER_FUNCTION_INT long
#define VSMC_MATH_INTEGER_FUNCTION_UINT unsigned long
#endif
#endif

namespace vsmc { namespace math {

namespace internal {

template <VSMC_MATH_INTEGER_FUNCTION_UINT N>
struct FactorialImpl
{
    static VSMC_CONSTEXPR const VSMC_MATH_INTEGER_FUNCTION_UINT value =
        N * FactorialImpl<N - 1>::value;
}; // struct FactorialImpl

template <> struct FactorialImpl<0>
{static VSMC_CONSTEXPR const VSMC_MATH_INTEGER_FUNCTION_UINT value = 1ULL;};

} // namespace vsmc::math::internal

/// \brief Factorial of an unsigned integer
/// \ingroup IntgerFunction
template <VSMC_MATH_INTEGER_FUNCTION_UINT N>
struct Factorial : public internal::FactorialImpl<N> {};

namespace internal {

template <VSMC_MATH_INTEGER_FUNCTION_UINT N, VSMC_MATH_INTEGER_FUNCTION_UINT K>
struct IsPowerOfImpl
{
    private :

    static VSMC_CONSTEXPR const bool nvalue = (N % K != 0);

    public :

    static VSMC_CONSTEXPR const bool value =
        nvalue ? false : IsPowerOfImpl<N / K, K>::value;
}; // struct IsPowerOfImpl

template <> struct IsPowerOfImpl<0, 0>
{static VSMC_CONSTEXPR const bool value = true;};

template <> struct IsPowerOfImpl<1, 0>
{static VSMC_CONSTEXPR const bool value = true;};

template <VSMC_MATH_INTEGER_FUNCTION_UINT N> struct IsPowerOfImpl<N, 0>
{static VSMC_CONSTEXPR const bool value = false;};

template <VSMC_MATH_INTEGER_FUNCTION_UINT K> struct IsPowerOfImpl<0, K>
{static VSMC_CONSTEXPR const bool value = false;};

template <VSMC_MATH_INTEGER_FUNCTION_UINT N> struct IsPowerOfImpl<N, N>
{static VSMC_CONSTEXPR const bool value = true;};

} // namespace vsmc::math::internal

/// \brief Check if an unsigned integer is power of another
/// \ingroup IntegerFunction
template <VSMC_MATH_INTEGER_FUNCTION_UINT N, VSMC_MATH_INTEGER_FUNCTION_UINT K>
struct IsPowerOf :
    public cxx11::integral_constant<bool,
    internal::IsPowerOfImpl<N, K>::value> {};

namespace internal {

template <VSMC_MATH_INTEGER_FUNCTION_INT N, VSMC_MATH_INTEGER_FUNCTION_UINT K>
struct PowerImpl
{
    static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value =
        N * PowerImpl<N, K - 1>::value;
}; // struct PowerImpl

template <>
struct PowerImpl<0, 0>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = 1;};

template <VSMC_MATH_INTEGER_FUNCTION_UINT K>
struct PowerImpl<0, K>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = 0;};

template <VSMC_MATH_INTEGER_FUNCTION_INT N>
struct PowerImpl<N, 0>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = 1;};

} // namespace vsmc::math::internal

/// \brief Raise an unsigned integer to a given power
/// \ingroup IntegerFunction
template <VSMC_MATH_INTEGER_FUNCTION_INT N, VSMC_MATH_INTEGER_FUNCTION_UINT K>
struct Power :
    public cxx11::integral_constant<VSMC_MATH_INTEGER_FUNCTION_INT,
    internal::PowerImpl<N, K>::value> {};

namespace internal {

template <VSMC_MATH_INTEGER_FUNCTION_UINT K,
         VSMC_MATH_INTEGER_FUNCTION_INT A, VSMC_MATH_INTEGER_FUNCTION_INT B>
struct FibonacciImpl
{
    static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value =
        FibonacciImpl<K - 1, A, B>::value + FibonacciImpl<K - 2, A, B>::value;
}; // struct FibonacciImpl

template <VSMC_MATH_INTEGER_FUNCTION_INT A, VSMC_MATH_INTEGER_FUNCTION_INT B>
struct FibonacciImpl<0, A, B>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = 0;};

template <VSMC_MATH_INTEGER_FUNCTION_INT A, VSMC_MATH_INTEGER_FUNCTION_INT B>
struct FibonacciImpl<1, A, B>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = A;};

template <VSMC_MATH_INTEGER_FUNCTION_INT A, VSMC_MATH_INTEGER_FUNCTION_INT B>
struct FibonacciImpl<2, A, B>
{static VSMC_CONSTEXPR VSMC_MATH_INTEGER_FUNCTION_INT value = B;};

} // namespace vsmc::math::internal

/// \brief The Fibonacci sequence (default: 0, 1, 1, ...)
/// \ingroup IntegerFunction
///
/// \details
/// The default sequence starts with 0, 1. The zero-th element or elements with
/// negative index (not defined by the templates since the first argument is of
/// type unsigned integers) in the sequence are defined as zero. Therefore, in
/// the default sequence, \f$x_n = x_{n-1} + x_{n-2}\f$ for all \f$n\f$. With a
/// starting value other than zero, this is not true for \f$n = 2\f$.
template <VSMC_MATH_INTEGER_FUNCTION_UINT K,
         VSMC_MATH_INTEGER_FUNCTION_INT A = 0,
         VSMC_MATH_INTEGER_FUNCTION_INT B = 1>
struct Fibonacci :
    public cxx11::integral_constant<VSMC_MATH_INTEGER_FUNCTION_INT,
    internal::FibonacciImpl<K, A, B>::value> {};

} } // namespace vsmc::math

#endif // VSMC_MATH_INTEGER_FUNCTION_HPP
