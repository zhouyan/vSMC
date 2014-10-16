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
#include <vsmc/math/constants.hpp>

#define VSMC_DEFINE_VMATH_1(ns, func, name) \
template <typename T>                                                        \
inline void v##name (std::size_t n, const T *a, T *y)                        \
{                                                                            \
    using ns::func;                                                          \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = func(a[i]);                                                   \
}

#define VSMC_DEFINE_VMATH_2(ns, func, name) \
template <typename T>                                                        \
inline void v##name (std::size_t n, const T *a, const T *b, T *y)            \
{                                                                            \
    using ns::func;                                                          \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = func(a[i], b[i]);                                             \
}

#define VSMC_DEFINE_VMATH_B(op, vname) \
template <typename T>                                                        \
inline void v##vname (std::size_t n, const T *a, const T *b, T *y)           \
{                                                                            \
    for (std::size_t i = 0; i != n; ++i)                                     \
        y[i] = a[i] op b[i];                                                 \
}

namespace vsmc {

namespace math {

namespace internal {

template <typename T>
inline T vmath_sqr (T a)
{return a * a;}

template <typename T>
inline T vmath_inv (T a)
{return static_cast<T>(1) / a;}

template <typename T>
inline T vmath_invsqrt (T a)
{using std::sqrt; return static_cast<T>(1) / sqrt(a);}

template <typename T>
inline T vmath_invcbrt (T a)
{using cxx11::cbrt; return static_cast<T>(1) / cbrt(a);}

template <typename T>
inline T vmath_pow2o3 (T a)
{using cxx11::cbrt; a = cbrt(a); return a * a;}

template <typename T>
inline T vmath_pow3o2 (T a)
{using std::sqrt; a = sqrt(a); return a * a * a;}

template <typename T>
inline T vmath_cdfnorm (T a)
{
    using cxx11::erf;
    return static_cast<T>(0.5) + static_cast<T>(0.5) * erf(
            a *  sqrt_1by2<T>());
}

template <typename T>
inline T vmath_erfinv (T a)
{using cxx11::erf; return static_cast<T>(1) / erf(a);}

template <typename T>
inline T vmath_erfcinv (T a)
{return vmath_erfinv(static_cast<T>(1) - a);}

template <typename T>
inline T vmath_cdfnorminv (T a)
{return static_cast<T>(1) / vmath_cdfnorm(a);}

} // namespace vsmc::math::internal

/// \defgroup vArithmetic Arithmetic functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i + b_i\f$
VSMC_DEFINE_VMATH_B(+, Add)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i - b_i\f$
VSMC_DEFINE_VMATH_B(-, Sub)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^2\f$
VSMC_DEFINE_VMATH_1(internal, vmath_sqr, Sqr)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i b_i\f$
VSMC_DEFINE_VMATH_B(*, Mul)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = |a_i|\f$
VSMC_DEFINE_VMATH_1(std, abs, Abs)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = (\beta_a a_i + \mu_a) / (\beta_b b_i + \mu_b)\f$
template <typename T>
inline void vLinearFrac (std::size_t n, const T *a, const T *b,
        T beta_a, T beta_b, T mu_a, T mu_b, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = beta_a * a[i] + mu_a;
    for (std::size_t i = 0; i != n; ++i)
        y[i] /= beta_b * b[i] + mu_b;
}

/// @}

/// \defgroup vPower Power and root functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{-1}\f$
VSMC_DEFINE_VMATH_1(internal, vmath_inv, Inv)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i / b_i\f$
VSMC_DEFINE_VMATH_B(/, Div)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i}\f$
VSMC_DEFINE_VMATH_1(std, sqrt, Sqrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt{a_i}\f$
VSMC_DEFINE_VMATH_1(internal, vmath_invsqrt, InvSqrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt[3]{a_i}\f$
VSMC_DEFINE_VMATH_1(cxx11, cbrt, Cbrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt[3]{a_i}\f$
VSMC_DEFINE_VMATH_1(internal, vmath_invcbrt, InvCbrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{2/3}\f$
VSMC_DEFINE_VMATH_1(internal, vmath_pow2o3, Pow2o3)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{3/2}\f$
VSMC_DEFINE_VMATH_1(internal, vmath_pow3o2, Pow3o2)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{b_i}\f$
VSMC_DEFINE_VMATH_2(std, pow, Pow)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^b\f$
template <typename T>
inline void vPowx (std::size_t n, const T *a, T b, T *y)
{
    using std::pow;
    for (std::size_t i = 0; i != n; ++i)
        y[i] = pow(a[i], b);
}

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i^2 + b_i^2}\f$
VSMC_DEFINE_VMATH_2(cxx11, hypot, Hypot)

/// @}

/// \defgroup vExponential Exponential and logarithm functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = e^{a_i}\f$
VSMC_DEFINE_VMATH_1(std, exp, Exp)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = e^{a_i} - 1\f$
VSMC_DEFINE_VMATH_1(cxx11, expm1, Expm1)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \ln(a_i)\f$
VSMC_DEFINE_VMATH_1(std, log, Ln)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log_{10}(a_i)\f$
VSMC_DEFINE_VMATH_1(std, log10, Log10)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log(a_i + 1)\f$
VSMC_DEFINE_VMATH_1(cxx11, log1p, Log1p)
/// @}

/// \defgroup vTrigonometric Trigonometric functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sin(a_i)\f$
VSMC_DEFINE_VMATH_1(std, cos, Cos)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \cos(a_i)\f$
VSMC_DEFINE_VMATH_1(std, sin, Sin)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \sin(a_i), z_i = \cos(a_i)\f$
template <typename T>
inline void vSinCos (std::size_t n, const T *a, T *y, T *z)
{
    using std::sin;
    using std::cos;
    for (std::size_t i = 0; i != n; ++i)
        y[i] = sin(a[i]);
    for (std::size_t i = 0; i != n; ++i)
        z[i] = cos(a[i]);
}

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \tan(a_i)\f$
VSMC_DEFINE_VMATH_1(std, tan, Tan)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arccos(a_i)\f$
VSMC_DEFINE_VMATH_1(std, acos, Acos)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arcsin(a_i)\f$
VSMC_DEFINE_VMATH_1(std, asin, Asin)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arctan(a_i)\f$
VSMC_DEFINE_VMATH_1(std, atan, Atan)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arctan(a_i / b_i)\f$ with
/// signs to determine the quadrant
VSMC_DEFINE_VMATH_2(std, atan2, Atan2)

/// @}

/// \defgroup vHyperbolic Hyperbolic functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \cosh(a_i)\f$
VSMC_DEFINE_VMATH_1(std, cosh, Cosh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sinh(a_i)\f$
VSMC_DEFINE_VMATH_1(std, sinh, Sinh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \tanh(a_i)\f$
VSMC_DEFINE_VMATH_1(std, tanh, Tanh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\cosh(a_i)\f$
VSMC_DEFINE_VMATH_1(cxx11, acosh, Acosh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\sinh(a_i)\f$
VSMC_DEFINE_VMATH_1(cxx11, asinh, Asinh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\tanh(a_i)\f$
VSMC_DEFINE_VMATH_1(cxx11, atanh, Atanh)

/// @}

/// \defgroup vSpecial Special functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{Erf}(a_i)\f$
VSMC_DEFINE_VMATH_1(cxx11, erf, Erf)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \mathrm{Erfc}(a_i) = \mathrm{Erf}(1 - a_i)\f$
VSMC_DEFINE_VMATH_1(cxx11, erfc, Erfc)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = 1 - \mathrm{Erfc}(a_i / \sqrt{2}) / 2\f$, the standard Normal CDF
VSMC_DEFINE_VMATH_1(internal, vmath_cdfnorm, CdfNorm)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{Erf}^{-1}(a_i)\f$
VSMC_DEFINE_VMATH_1(internal, vmath_erfinv, ErfInv)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \mathrm{Erf}^{-1}(1 - a_i)\f$
VSMC_DEFINE_VMATH_1(internal, vmath_erfcinv, ErfcInv)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \sqrt{2}\mathrm{Erf}^{-1}(2a_i - 1)\f$, inverse of the standard
/// Nomral CDF
VSMC_DEFINE_VMATH_1(internal, vmath_cdfnorminv, CdfNormInv)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \ln\Gamma(a_i)\f$,
/// logarithm of the Gamma function
VSMC_DEFINE_VMATH_1(cxx11, lgamma, LGamma)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \Gamma(a_i)\f$, the Gamma
/// function
VSMC_DEFINE_VMATH_1(cxx11, tgamma, TGamma)
/// @}

} // namespace vsmc::math

} // namespace vsmc

#if VSMC_USE_MKL_VML

#include <mkl_vml.h>

#ifndef VSMC_VMATH_VML_THRESHOLD
#define VSMC_VMATH_VML_THRESHOLD 1000
#endif

#define VSMC_DEFINE_VMATH_VML_1(name) \
inline void v##name                                                          \
(std::size_t n, const float *a, float *y)                                    \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD ?                                           \
        v##name<float>(n, a, y):                                             \
        ::vs##name(static_cast<MKL_INT>(n), a, y);                           \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, double *y)                                  \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD ?                                           \
        v##name<double>(n, a, y):                                            \
        ::vd##name(static_cast<MKL_INT>(n), a, y);                           \
}

#define VSMC_DEFINE_VMATH_VML_2(name) \
inline void v##name                                                          \
(std::size_t n, const float *a, const float *b, float *y)                    \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD ?                                           \
        v##name<float>(n, a, b, y):                                          \
        ::vs##name(static_cast<MKL_INT>(n), a, b, y);                        \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, const double *b, double *y)                 \
{                                                                            \
    n < VSMC_VMATH_VML_THRESHOLD ?                                           \
        v##name<double>(n, a, b, y):                                         \
        ::vd##name(static_cast<MKL_INT>(n), a, b, y);                        \
}

namespace vsmc {

namespace math {

VSMC_DEFINE_VMATH_VML_2(Add)
VSMC_DEFINE_VMATH_VML_2(Sub)
VSMC_DEFINE_VMATH_VML_1(Sqr)
VSMC_DEFINE_VMATH_VML_2(Mul)
VSMC_DEFINE_VMATH_VML_1(Abs)
inline void vLinearFrac (std::size_t n, const float *a, const float *b,
        float beta_a, float beta_b, float mu_a, float mu_b, float *y)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vLinearFrac<float>(n, a, b, beta_a, beta_b, mu_a, mu_b, y):
        ::vsLinearFrac(static_cast<MKL_INT>(n), a, b,
                beta_a, beta_b, mu_a, mu_b, y);
}
inline void vLinearFrac (std::size_t n, const double *a, const double *b,
        double beta_a, double beta_b, double mu_a, double mu_b, double *y)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vLinearFrac<double>(n, a, b, beta_a, beta_b, mu_a, mu_b, y):
        ::vdLinearFrac(static_cast<MKL_INT>(n), a, b,
                beta_a, beta_b, mu_a, mu_b, y);
}

VSMC_DEFINE_VMATH_VML_1(Inv)
VSMC_DEFINE_VMATH_VML_2(Div)
VSMC_DEFINE_VMATH_VML_1(Sqrt)
VSMC_DEFINE_VMATH_VML_1(InvSqrt)
VSMC_DEFINE_VMATH_VML_1(Cbrt)
VSMC_DEFINE_VMATH_VML_1(InvCbrt)
VSMC_DEFINE_VMATH_VML_1(Pow2o3)
VSMC_DEFINE_VMATH_VML_1(Pow3o2)
VSMC_DEFINE_VMATH_VML_2(Pow)
inline void vPowx (std::size_t n, const float *a, float b, float *y)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vPowx<float>(n, a, b, y):
        ::vsPowx(static_cast<MKL_INT>(n), a, b, y);
}
inline void vPowx (std::size_t n, const double *a, double b, double *y)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vPowx<double>(n, a, b, y):
        ::vdPowx(static_cast<MKL_INT>(n), a, b, y);
}
VSMC_DEFINE_VMATH_VML_2(Hypot)

VSMC_DEFINE_VMATH_VML_1(Exp)
VSMC_DEFINE_VMATH_VML_1(Expm1)
VSMC_DEFINE_VMATH_VML_1(Ln)
VSMC_DEFINE_VMATH_VML_1(Log10)
VSMC_DEFINE_VMATH_VML_1(Log1p)

VSMC_DEFINE_VMATH_VML_1(Cos)
VSMC_DEFINE_VMATH_VML_1(Sin)
inline void vSinCos (std::size_t n, const float *a, float *y, float *z)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vSinCos<float>(n, a, y, z):
        ::vsSinCos(static_cast<MKL_INT>(n), a, y, z);
}
inline void vSinCos (std::size_t n, const double *a, double *y, double *z)
{
    n < VSMC_VMATH_VML_THRESHOLD ?
        vSinCos<double>(n, a, y, z):
        ::vdSinCos(static_cast<MKL_INT>(n), a, y, z);
}
VSMC_DEFINE_VMATH_VML_1(Tan)
VSMC_DEFINE_VMATH_VML_1(Acos)
VSMC_DEFINE_VMATH_VML_1(Asin)
VSMC_DEFINE_VMATH_VML_1(Atan)
VSMC_DEFINE_VMATH_VML_2(Atan2)

VSMC_DEFINE_VMATH_VML_1(Cosh)
VSMC_DEFINE_VMATH_VML_1(Sinh)
VSMC_DEFINE_VMATH_VML_1(Tanh)
VSMC_DEFINE_VMATH_VML_1(Acosh)
VSMC_DEFINE_VMATH_VML_1(Asinh)
VSMC_DEFINE_VMATH_VML_1(Atanh)

VSMC_DEFINE_VMATH_VML_1(Erf)
VSMC_DEFINE_VMATH_VML_1(Erfc)
VSMC_DEFINE_VMATH_VML_1(CdfNorm)
VSMC_DEFINE_VMATH_VML_1(ErfInv)
VSMC_DEFINE_VMATH_VML_1(ErfcInv)
VSMC_DEFINE_VMATH_VML_1(CdfNormInv)
VSMC_DEFINE_VMATH_VML_1(LGamma)
VSMC_DEFINE_VMATH_VML_1(TGamma)

} // namespace vsmc::math

} // namespace vsmc

#elif VSMC_USE_VECLIB_VFORCE

#include <vecLib/vForce.h>

#ifndef VSMC_VMATH_VFORCE_THRESHOLD
#define VSMC_VMATH_VFORCE_THRESHOLD 1000
#endif

#define VSMC_DEFINE_VMATH_VFORCE_1(func, name) \
inline void v##name                                                          \
(std::size_t n, const float *a, float *y)                                    \
{                                                                            \
    const int in = static_cast<int>(n);                                      \
    n < VSMC_VMATH_VFORCE_THRESHOLD ?                                        \
        v##name<float>(n, a, y):                                             \
        ::vv##func##f(y, a, &in);                                            \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, double *y)                                  \
{                                                                            \
    const int in = static_cast<int>(n);                                      \
    n < VSMC_VMATH_VFORCE_THRESHOLD ?                                        \
        v##name<double>(n, a, y):                                            \
        ::vv##func(y, a, &in);                                               \
}

#define VSMC_DEFINE_VMATH_VFORCE_2(func, name) \
inline void v##name                                                          \
(std::size_t n, const float *a, const float *b, float *y)                    \
{                                                                            \
    const int in = static_cast<int>(n);                                      \
    n < VSMC_VMATH_VFORCE_THRESHOLD ?                                        \
        v##name<float>(n, a, b, y):                                          \
        ::vv##func##f(y, a, b, &in);                                         \
}                                                                            \
inline void v##name                                                          \
(std::size_t n, const double *a, const double *b, double *y)                 \
{                                                                            \
    const int in = static_cast<int>(n);                                      \
    n < VSMC_VMATH_VFORCE_THRESHOLD ?                                        \
        v##name<double>(n, a, b, y):                                         \
        ::vv##func(y, a, b, &in);                                            \
}

namespace vsmc {

namespace math {

VSMC_DEFINE_VMATH_VFORCE_1(rec,   Inv)
VSMC_DEFINE_VMATH_VFORCE_2(div,   Div)
VSMC_DEFINE_VMATH_VFORCE_1(sqrt,  Sqrt)
VSMC_DEFINE_VMATH_VFORCE_1(rsqrt, InvSqrt)
VSMC_DEFINE_VMATH_VFORCE_2(pow,   Pow)

VSMC_DEFINE_VMATH_VFORCE_1(exp,   Exp)
VSMC_DEFINE_VMATH_VFORCE_1(log,   Ln)
VSMC_DEFINE_VMATH_VFORCE_1(log10, Log10)
VSMC_DEFINE_VMATH_VFORCE_1(log1p, Log1p)

VSMC_DEFINE_VMATH_VFORCE_1(cos, Cos)
VSMC_DEFINE_VMATH_VFORCE_1(sin, Sin)
inline void vSinCos (std::size_t n, const float *a, float *y, float *z)
{
    int in = static_cast<int>(n);
    n < VSMC_VMATH_VFORCE_THRESHOLD ?
        vSinCos<float>(n, a, y, z):
        ::vvsincosf(z, y, a, &in);
}
inline void vSinCos (std::size_t n, const double *a, double *y, double *z)
{
    int in = static_cast<int>(n);
    n < VSMC_VMATH_VFORCE_THRESHOLD ?
        vSinCos<double>(n, a, y, z):
        ::vvsincos(z, y, a, &in);
}
VSMC_DEFINE_VMATH_VFORCE_1(tan,   Tan)
VSMC_DEFINE_VMATH_VFORCE_1(acos,  Acos)
VSMC_DEFINE_VMATH_VFORCE_1(asin,  Asin)
VSMC_DEFINE_VMATH_VFORCE_1(atan,  Atan)
VSMC_DEFINE_VMATH_VFORCE_2(atan2, Atan2)

VSMC_DEFINE_VMATH_VFORCE_1(cosh,  Cosh)
VSMC_DEFINE_VMATH_VFORCE_1(sinh,  Sinh)
VSMC_DEFINE_VMATH_VFORCE_1(tanh,  Tanh)
VSMC_DEFINE_VMATH_VFORCE_1(acosh,  Acosh)
VSMC_DEFINE_VMATH_VFORCE_1(asinh,  Asinh)
VSMC_DEFINE_VMATH_VFORCE_1(atanh,  Atanh)

} // namespace vsmc::math

} // namespace vsmc

#endif // VSMC_USE_MKL_VML

#endif // VSMC_MATH_VMATH_HPP
