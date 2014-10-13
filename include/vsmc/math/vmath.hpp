//============================================================================
// include/vsmc/math/vmath.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_MATH_VMATH_HPP
#define VSMC_MATH_VMATH_HPP

#include <vsmc/cxx11/cmath.hpp>

#define VSMC_DEFINE_VMATH_1(ns, name) \
template <typename T>                                                        \
inline void v##name (std::size_t n, const T *a, T *y)                        \
{                                                                            \
    using ns::name;                                                          \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = name(a[i]);                                                   \
}

#define VSMC_DEFINE_VMATH_2(ns, name) \
template <typename T>                                                        \
inline void v##name (std::size_t n, const T *a, const T *b, T *y)            \
{                                                                            \
    using ns::name;                                                          \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = name(a[i], b[i]);                                             \
}

#define VSMC_DEFINE_VMATH_B(name, op) \
template <typename T>                                                        \
inline void v##name (std::size_t n, const T *a, const T *b, T *y)            \
{                                                                            \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = a[i] op b[i];                                                 \
}

namespace vsmc {

namespace math {

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = |a_i|\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, abs)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{acos}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, acos)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{asin}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, asin)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{atan}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, atan)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \cos(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, cos)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \cosh(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, cosh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = e^{a_i}\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, exp)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \log(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, log)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \log_{10}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, log10)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \sin(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, sin)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \sinh(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, sinh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i}\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, sqrt)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \tan(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, tan)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \tanh(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(std, tanh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{acosh}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, acosh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{asinh}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, asinh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{atanh}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, atanh)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \sqrt[3]{a_i}\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, cbrt)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{Erf}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, erf)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \mathrm{Erfc}(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, erfc)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = e^{a_i} - 1\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, expm1)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \log\Gamma(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, lgamma)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \log(a_i - 1)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, log1p)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \Gamma(a_i)\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_1(cxx11, tgamma)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i^{b_i}\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_2(std, pow)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i^2 + b_i^2}\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_2(cxx11, hypot)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i + b_i\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_B(add, +)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i - b_i\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_B(sub, -)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i * b_i\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_B(mul, *)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i / b_i\f$
/// \ingroup vMath
VSMC_DEFINE_VMATH_B(div, /)

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i^2\f$
/// \ingroup vMath
template <typename T>
inline void vsqr (std::size_t n, const T *a, T *y)
{for (std::size_t i = 0; i != n; ++i) y[i] = a[i] * a[i];}

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = a_i^b\f$
/// \ingroup vMath
template <typename T>
inline void vpowx (std::size_t n, const T *a, T b, T *y)
{using std::pow; for (std::size_t i = 0; i != n; ++i) y[i] = pow(a[i], b);}

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = 1 / a_i\f$
/// \ingroup vMath
template <typename T>
inline void vinv (std::size_t n, const T *a, T *y)
{
    const T one = static_cast<T>(1);
    for (std::size_t i = 0; i != n; ++i)
        y[i] = one / a[i];
}

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt{a_i}\f$
/// \ingroup vMath
template <typename T>
inline void vinvsqrt (std::size_t n, const T *a, T *y)
{
    using std::sqrt;
    const T one = static_cast<T>(1);
    for (std::size_t i = 0; i != n; ++i)
        y[i] = one / sqrt(a[i]);
}

/// \brief For \f$i = 1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt[3]{a_i}\f$
/// \ingroup vMath
template <typename T>
inline void vinvcbrt (std::size_t n, const T *a, T *y)
{
    using cxx11::cbrt;
    const T one = static_cast<T>(1);
    for (std::size_t i = 0; i != n; ++i)
        y[i] = one / cbrt(a[i]);
}

} // namespace vsmc::math

} // namespace vsmc

#if VSMC_USE_MKL_VML

#include <mkl_vml.h>

#ifndef VSMC_VMATH_VML_THRESHOLD_abs
#define VSMC_VMATH_VML_THRESHOLD_abs 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_acos
#define VSMC_VMATH_VML_THRESHOLD_acos 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_acosh
#define VSMC_VMATH_VML_THRESHOLD_acosh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_asin
#define VSMC_VMATH_VML_THRESHOLD_asin 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_asinh
#define VSMC_VMATH_VML_THRESHOLD_asinh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_atan
#define VSMC_VMATH_VML_THRESHOLD_atan 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_atanh
#define VSMC_VMATH_VML_THRESHOLD_atanh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_cbrt
#define VSMC_VMATH_VML_THRESHOLD_cbrt 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_cos
#define VSMC_VMATH_VML_THRESHOLD_cos 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_cosh
#define VSMC_VMATH_VML_THRESHOLD_cosh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_erf
#define VSMC_VMATH_VML_THRESHOLD_erf 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_erfc
#define VSMC_VMATH_VML_THRESHOLD_erfc 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_exp
#define VSMC_VMATH_VML_THRESHOLD_exp 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_expm1
#define VSMC_VMATH_VML_THRESHOLD_expm1 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_inv
#define VSMC_VMATH_VML_THRESHOLD_inv 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_invcbrt
#define VSMC_VMATH_VML_THRESHOLD_invcbrt 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_invsqrt
#define VSMC_VMATH_VML_THRESHOLD_invsqrt 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_lgamma
#define VSMC_VMATH_VML_THRESHOLD_lgamma 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_log
#define VSMC_VMATH_VML_THRESHOLD_log 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_log10
#define VSMC_VMATH_VML_THRESHOLD_log10 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_log1p
#define VSMC_VMATH_VML_THRESHOLD_log1p 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_sin
#define VSMC_VMATH_VML_THRESHOLD_sin 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_sinh
#define VSMC_VMATH_VML_THRESHOLD_sinh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_sqr
#define VSMC_VMATH_VML_THRESHOLD_sqr 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_sqrt
#define VSMC_VMATH_VML_THRESHOLD_sqrt 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_tan
#define VSMC_VMATH_VML_THRESHOLD_tan 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_tanh
#define VSMC_VMATH_VML_THRESHOLD_tanh 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_tgamma
#define VSMC_VMATH_VML_THRESHOLD_tgamma 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_add
#define VSMC_VMATH_VML_THRESHOLD_add 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_div
#define VSMC_VMATH_VML_THRESHOLD_div 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_hypot
#define VSMC_VMATH_VML_THRESHOLD_hypot 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_mul
#define VSMC_VMATH_VML_THRESHOLD_mul 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_pow
#define VSMC_VMATH_VML_THRESHOLD_pow 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_sub
#define VSMC_VMATH_VML_THRESHOLD_sub 1000
#endif

#ifndef VSMC_VMATH_VML_THRESHOLD_powx
#define VSMC_VMATH_VML_THRESHOLD_powx 1000
#endif

#define VSMC_DEFINE_VMATH_VML_1(name, vname) \
inline void v##name                                                          \
(std::size_t n, const float *a, float *y)                                    \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD_##name ?                                    \
        v##name<float>(n, a, y):                                             \
        ::vs##vname(static_cast<MKL_INT>(n), a, y);                          \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, double *y)                                  \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD_##name ?                                    \
        v##name<double>(n, a, y):                                            \
        ::vd##vname(static_cast<MKL_INT>(n), a, y);                          \
}

#define VSMC_DEFINE_VMATH_VML_2(name, vname) \
inline void v##name                                                          \
(std::size_t n, const float *a, const float *b, float *y)                    \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD_##name ?                                    \
        v##name<float>(n, a, b, y):                                          \
        ::vs##vname(static_cast<MKL_INT>(n), a, b, y);                       \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, const double *b, double *y)                 \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD_##name ?                                    \
        v##name<double>(n, a, b, y):                                         \
        ::vd##vname(static_cast<MKL_INT>(n), a, b, y);                       \
}

namespace vsmc {

namespace math {

VSMC_DEFINE_VMATH_VML_1(abs,     Abs)
VSMC_DEFINE_VMATH_VML_1(acos,    Acos)
VSMC_DEFINE_VMATH_VML_1(acosh,   Acosh)
VSMC_DEFINE_VMATH_VML_1(asin,    Asin)
VSMC_DEFINE_VMATH_VML_1(asinh,   Asinh)
VSMC_DEFINE_VMATH_VML_1(atan,    Atan)
VSMC_DEFINE_VMATH_VML_1(atanh,   Atanh)
VSMC_DEFINE_VMATH_VML_1(cbrt,    Cbrt)
VSMC_DEFINE_VMATH_VML_1(cos,     Cos)
VSMC_DEFINE_VMATH_VML_1(cosh,    Cosh)
VSMC_DEFINE_VMATH_VML_1(erf,     Erf)
VSMC_DEFINE_VMATH_VML_1(erfc,    Erfc)
VSMC_DEFINE_VMATH_VML_1(exp,     Exp)
VSMC_DEFINE_VMATH_VML_1(expm1,   Expm1)
VSMC_DEFINE_VMATH_VML_1(inv,     Inv)
VSMC_DEFINE_VMATH_VML_1(invcbrt, InvCbrt)
VSMC_DEFINE_VMATH_VML_1(invsqrt, InvSqrt)
VSMC_DEFINE_VMATH_VML_1(lgamma,  LGamma)
VSMC_DEFINE_VMATH_VML_1(log,     Ln)
VSMC_DEFINE_VMATH_VML_1(log10,   Log10)
VSMC_DEFINE_VMATH_VML_1(log1p,   Log1p)
VSMC_DEFINE_VMATH_VML_1(sin,     Sin)
VSMC_DEFINE_VMATH_VML_1(sinh,    Sinh)
VSMC_DEFINE_VMATH_VML_1(sqr,     Sqr)
VSMC_DEFINE_VMATH_VML_1(sqrt,    Sqrt)
VSMC_DEFINE_VMATH_VML_1(tan,     Tan)
VSMC_DEFINE_VMATH_VML_1(tanh,    Tanh)
VSMC_DEFINE_VMATH_VML_1(tgamma,  TGamma)

VSMC_DEFINE_VMATH_VML_2(add,   Add)
VSMC_DEFINE_VMATH_VML_2(div,   Div)
VSMC_DEFINE_VMATH_VML_2(hypot, Hypot)
VSMC_DEFINE_VMATH_VML_2(mul,   Mul)
VSMC_DEFINE_VMATH_VML_2(pow,   Pow)
VSMC_DEFINE_VMATH_VML_2(sub,   Sub)

inline void vpowx (std::size_t n, const float *a, float b, float *y)
{
    n < VSMC_VMATH_VML_THRESHOLD_powx ?
        vpowx<float>(n, a, b, y):
        ::vsPowx(static_cast<MKL_INT>(n), a, b, y);
}

inline void vpowx (std::size_t n, const double *a, double b, double *y)
{
    n < VSMC_VMATH_VML_THRESHOLD_powx ?
        vpowx<double>(n, a, b, y):
        ::vdPowx(static_cast<MKL_INT>(n), a, b, y);
}

} // namespace vsmc::math

} // namespace vsmc

#endif // VSMC_USE_MKL_VML

#endif // VSMC_MATH_VMATH_HPP
