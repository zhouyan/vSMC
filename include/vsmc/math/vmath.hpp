#ifndef VSMC_MATH_VMATH_HPP
#define VSMC_MATH_VMATH_HPP

#include <vsmc/cxx11/cmath.hpp>

#define VSMC_DEFINE_VMATH_1(ns, name) \
    template <typename T>                                                    \
    inline void v##name (int n, const T *a, T *y)                            \
    {                                                                        \
        using ns::name;                                                      \
        for (int i = 0; i != n; ++i)                                         \
            y[i] = name(a[i]);                                               \
    }

#define VSMC_DEFINE_VMATH_2(ns, name) \
    template <typename T>                                                    \
    inline void v##name (int n, const T *a, const T *b, T *y)                \
    {                                                                        \
        using ns::name;                                                      \
        for (int i = 0; i != n; ++i)                                         \
            y[i] = name(a[i], b[i]);                                         \
    }

#define VSMC_DEFINE_VMATH_B(name, op) \
    template <typename T>                                                    \
    inline void v##name (int n, const T *a, const T *b, T *y)                \
    {                                                                        \
        for (int i = 0; i != n; ++i)                                         \
            y[i] = a[i] op b[i];                                             \
    }

namespace vsmc {

namespace math {

VSMC_DEFINE_VMATH_1(std, abs)
VSMC_DEFINE_VMATH_1(std, acos)
VSMC_DEFINE_VMATH_1(std, asin)
VSMC_DEFINE_VMATH_1(std, atan)
VSMC_DEFINE_VMATH_1(std, cos)
VSMC_DEFINE_VMATH_1(std, cosh)
VSMC_DEFINE_VMATH_1(std, exp)
VSMC_DEFINE_VMATH_1(std, log)
VSMC_DEFINE_VMATH_1(std, log10)
VSMC_DEFINE_VMATH_1(std, sin)
VSMC_DEFINE_VMATH_1(std, sinh)
VSMC_DEFINE_VMATH_1(std, sqrt)
VSMC_DEFINE_VMATH_1(std, tan)
VSMC_DEFINE_VMATH_1(std, tanh)

VSMC_DEFINE_VMATH_1(cxx11, acosh)
VSMC_DEFINE_VMATH_1(cxx11, asinh)
VSMC_DEFINE_VMATH_1(cxx11, atanh)
VSMC_DEFINE_VMATH_1(cxx11, cbrt)
VSMC_DEFINE_VMATH_1(cxx11, erf)
VSMC_DEFINE_VMATH_1(cxx11, erfc)
VSMC_DEFINE_VMATH_1(cxx11, expm1)
VSMC_DEFINE_VMATH_1(cxx11, lgamma)
VSMC_DEFINE_VMATH_1(cxx11, log1p)
VSMC_DEFINE_VMATH_1(cxx11, tgamma)

VSMC_DEFINE_VMATH_2(std, pow)

VSMC_DEFINE_VMATH_2(cxx11, hypot)

VSMC_DEFINE_VMATH_B(add, +)
VSMC_DEFINE_VMATH_B(sub, -)
VSMC_DEFINE_VMATH_B(mul, *)
VSMC_DEFINE_VMATH_B(div, /)

template <typename T>
inline void vsqr (int n, const T *a, T *y)
{for (int i = 0; i != n; ++i) y[i] = a[i] * a[i];}

template <typename T>
inline void vpowx (int n, const T *a, T b, T *y)
{using std::pow; for (int i = 0; i != n; ++i) y[i] = pow(a[i], b);}

template <typename T>
inline void vinv (int n, const T *a, T *y)
{
    const T one = static_cast<T>(1);
    for (int i = 0; i != n; ++i)
        y[i] = one / a[i];
}

template <typename T>
inline void vinvsqrt (int n, const T *a, T *y)
{
    using std::sqrt;
    const T one = static_cast<T>(1);
    for (int i = 0; i != n; ++i)
        y[i] = one / sqrt(a[i]);
}

template <typename T>
inline void vinvcbrt (int n, const T *a, T *y)
{
    using std::cbrt;
    const T one = static_cast<T>(1);
    for (int i = 0; i != n; ++i)
        y[i] = one / cbrt(a[i]);
}

} // namespace vsmc::math

} // namespace vsmc

#if VSMC_USE_MKL

#include <mkl_vml.h>

#define VSMC_DEFINE_VMATH_MKL_1(name, vname) \
    template <> inline void v##name<float>                                   \
    (int n, const float *a, float *y)                                        \
    {::vs##vname(static_cast<MKL_INT>(n), a, y);}                            \
    template <> inline void v##name<double>                                  \
    (int n, const double *a, double *y)                                      \
    {::vd##vname(static_cast<MKL_INT>(n), a, y);}

#define VSMC_DEFINE_VMATH_MKL_2(name, vname) \
    template <> inline void v##name<float>                                   \
    (int n, const float *a, const float *b, float *y)                        \
    {::vs##vname(static_cast<MKL_INT>(n), a, b, y);}                         \
    template <> inline void v##name<double>                                  \
    (int n, const double *a, const double *b, double *y)                     \
    {::vd##vname(static_cast<MKL_INT>(n), a, b, y);}

namespace vsmc {

namespace math {

VSMC_DEFINE_VMATH_MKL_1(abs,     Abs)
VSMC_DEFINE_VMATH_MKL_1(acos,    Acos)
VSMC_DEFINE_VMATH_MKL_1(acosh,   Acosh)
VSMC_DEFINE_VMATH_MKL_1(asin,    Asin)
VSMC_DEFINE_VMATH_MKL_1(asinh,   Asinh)
VSMC_DEFINE_VMATH_MKL_1(atan,    Atan)
VSMC_DEFINE_VMATH_MKL_1(atanh,   Atanh)
VSMC_DEFINE_VMATH_MKL_1(cbrt,    Cbrt)
VSMC_DEFINE_VMATH_MKL_1(cos,     Cos)
VSMC_DEFINE_VMATH_MKL_1(cosh,    Cosh)
VSMC_DEFINE_VMATH_MKL_1(erf,     Erf)
VSMC_DEFINE_VMATH_MKL_1(erfc,    Erfc)
VSMC_DEFINE_VMATH_MKL_1(exp,     Exp)
VSMC_DEFINE_VMATH_MKL_1(expm1,   Expm1)
VSMC_DEFINE_VMATH_MKL_1(inv,     Inv)
VSMC_DEFINE_VMATH_MKL_1(invcbrt, InvCbrt)
VSMC_DEFINE_VMATH_MKL_1(invsqrt, InvSqrt)
VSMC_DEFINE_VMATH_MKL_1(lgamma,  LGamma)
VSMC_DEFINE_VMATH_MKL_1(log,     Ln)
VSMC_DEFINE_VMATH_MKL_1(log10,   Log10)
VSMC_DEFINE_VMATH_MKL_1(log1p,   Log1p)
VSMC_DEFINE_VMATH_MKL_1(sin,     Sin)
VSMC_DEFINE_VMATH_MKL_1(sinh,    Sinh)
VSMC_DEFINE_VMATH_MKL_1(sqr,     Sqr)
VSMC_DEFINE_VMATH_MKL_1(sqrt,    Sqrt)
VSMC_DEFINE_VMATH_MKL_1(tan,     Tan)
VSMC_DEFINE_VMATH_MKL_1(tanh,    Tanh)
VSMC_DEFINE_VMATH_MKL_1(tgamma,  TGamma)

VSMC_DEFINE_VMATH_MKL_2(add,   Add)
VSMC_DEFINE_VMATH_MKL_2(div,   Div)
VSMC_DEFINE_VMATH_MKL_2(hypot, Hypot)
VSMC_DEFINE_VMATH_MKL_2(mul,   Mul)
VSMC_DEFINE_VMATH_MKL_2(pow,   Pow)
VSMC_DEFINE_VMATH_MKL_2(sub,   Sub)

template<>
inline void vpowx<float> (int n, const float *a, float b, float *y)
{::vsPowx(static_cast<MKL_INT>(n), a, b, y);}

template<>
inline void vpowx<double> (int n, const double *a, double b, double *y)
{::vdPowx(static_cast<MKL_INT>(n), a, b, y);}

} // namespace vsmc::math

} // namespace vsmc

#endif // VSMC_USE_MKL

#endif // VSMC_MATH_VMATH_HPP
