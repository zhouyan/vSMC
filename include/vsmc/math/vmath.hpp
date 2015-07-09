//============================================================================
// vSMC/include/vsmc/math/vmath.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_MATH_VMATH_HPP
#define VSMC_MATH_VMATH_HPP

#include <vsmc/internal/config.h>
#include <vsmc/math/constants.hpp>
#include <cmath>

#if VSMC_USE_MKL_VML
#include <mkl.h>
#endif

#define VSMC_DEFINE_MATH_VMATH_1(func, name)                                  \
    template <typename T>                                                     \
    inline void name(std::size_t n, const T *a, T *y)                         \
    {                                                                         \
        for (std::size_t i = 0; i != n; ++i)                                  \
            y[i] = func(a[i]);                                                \
    }

#define VSMC_DEFINE_MATH_VMATH_2(func, name)                                  \
    template <typename T>                                                     \
    inline void name(std::size_t n, const T *a, const T *b, T *y)             \
    {                                                                         \
        for (std::size_t i = 0; i != n; ++i)                                  \
            y[i] = func(a[i], b[i]);                                          \
    }

#define VSMC_DEFINE_MATH_VMATH_B(op, name)                                    \
    template <typename T>                                                     \
    inline void name(std::size_t n, const T *a, const T *b, T *y)             \
    {                                                                         \
        for (std::size_t i = 0; i != n; ++i)                                  \
            y[i] = a[i] op b[i];                                              \
    }

#define VSMC_DEFINE_MATH_VMATH_VS(op, name)                                   \
    template <typename T>                                                     \
    inline void name(std::size_t n, const T *a, T b, T *y)                    \
    {                                                                         \
        for (std::size_t i = 0; i != n; ++i)                                  \
            y[i] = a[i] op b;                                                 \
    }

#define VSMC_DEFINE_MATH_VMATH_SV(op, name)                                   \
    template <typename T>                                                     \
    inline void name(std::size_t n, T a, const T *b, T *y)                    \
    {                                                                         \
        for (std::size_t i = 0; i != n; ++i)                                  \
            y[i] = a op b[i];                                                 \
    }

namespace vsmc
{

namespace internal
{

template <typename T>
inline T vmath_sqr(T a)
{
    return a * a;
}

template <typename T>
inline T vmath_inv(T a)
{
    return static_cast<T>(1) / a;
}

template <typename T>
inline T vmath_invsqrt(T a)
{
    return static_cast<T>(1) / std::sqrt(a);
}

template <typename T>
inline T vmath_invcbrt(T a)
{
    return static_cast<T>(1) / std::cbrt(a);
}

template <typename T>
inline T vmath_pow2o3(T a)
{
    a = std::cbrt(a);

    return a * a;
}

template <typename T>
inline T vmath_pow3o2(T a)
{
    a = std::sqrt(a);

    return a * a * a;
}

template <typename T>
inline T vmath_exp10(T a)
{
    return std::exp(a * const_ln_10<T>());
}

template <typename T>
inline T vmath_cdfnorm(T a)
{
    return static_cast<T>(0.5) +
        static_cast<T>(0.5) * std::erf(a * const_sqrt_1by2<T>());
}

template <typename T>
inline T vmath_erfinv(T a)
{
    return static_cast<T>(1) / std::erf(a);
}

template <typename T>
inline T vmath_erfcinv(T a)
{
    return vmath_erfinv(static_cast<T>(1) - a);
}

template <typename T>
inline T vmath_cdfnorminv(T a)
{
    return static_cast<T>(1) / vmath_cdfnorm(a);
}

} // namespace vsmc::internal

/// \defgroup vArithmetic Arithmetic functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i + b_i\f$
VSMC_DEFINE_MATH_VMATH_B(+, add)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i + b\f$
VSMC_DEFINE_MATH_VMATH_VS(+, add)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a + b_i\f$
VSMC_DEFINE_MATH_VMATH_SV(+, add)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i - b_i\f$
VSMC_DEFINE_MATH_VMATH_B(-, sub)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i - b\f$
VSMC_DEFINE_MATH_VMATH_VS(-, sub)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a - b_i\f$
VSMC_DEFINE_MATH_VMATH_SV(-, sub)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^2\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_sqr, sqr)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i b_i\f$
VSMC_DEFINE_MATH_VMATH_B(*, mul)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i b\f$
VSMC_DEFINE_MATH_VMATH_VS(*, mul)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a b_i\f$
VSMC_DEFINE_MATH_VMATH_SV(*, mul)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = |a_i|\f$
VSMC_DEFINE_MATH_VMATH_1(std::abs, abs)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = (\beta_a a_i + \mu_a) / (\beta_b b_i + \mu_b)\f$
template <typename T>
inline void linear_frac(std::size_t n, const T *a, const T *b, T beta_a,
    T beta_b, T mu_a, T mu_b, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = beta_a * a[i] + mu_a;
    for (std::size_t i = 0; i != n; ++i)
        y[i] /= beta_b * b[i] + mu_b;
}

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = a_i + b_i * x_i\f$.
template <typename T>
inline void fma(std::size_t n, const T *a, const T *b, const T *x, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = a[i] + b[i] * x[i];
}

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = a + b_i * x_i\f$.
template <typename T>
inline void fma(std::size_t n, T a, const T *b, const T *x, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = a + b[i] * x[i];
}

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = a_i + b * x_i\f$.
template <typename T>
inline void fma(std::size_t n, const T *a, T b, const T *x, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = a[i] + b * x[i];
}

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = a + b * x_i\f$.
template <typename T>
inline void fma(std::size_t n, T a, T b, const T *x, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = a + b * x[i];
}

/// @}

/// \defgroup vPower Power and root functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{-1}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_inv, inv)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i / b_i\f$
VSMC_DEFINE_MATH_VMATH_B(/, div)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i / b\f$
VSMC_DEFINE_MATH_VMATH_VS(/, div)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a / b_i\f$
VSMC_DEFINE_MATH_VMATH_SV(/, div)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(std::sqrt, sqrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_invsqrt, invsqrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt[3]{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(std::cbrt, cbrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 1 / \sqrt[3]{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_invcbrt, invcbrt)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{2/3}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_pow2o3, pow2o3)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{3/2}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_pow3o2, pow3o2)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^{b_i}\f$
VSMC_DEFINE_MATH_VMATH_2(std::pow, pow)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = a_i^b\f$
template <typename T>
inline void pow(std::size_t n, const T *a, T b, T *y)
{
    for (std::size_t i = 0; i != n; ++i)
        y[i] = std::pow(a[i], b);
}

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sqrt{a_i^2 + b_i^2}\f$
VSMC_DEFINE_MATH_VMATH_2(std::hypot, hypot)

/// @}

/// \defgroup vExponential Exponential and logarithm functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = e^{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(std::exp, exp)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 2^{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(std::exp2, exp2)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = 10^{a_i}\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_exp10, exp10)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = e^{a_i} - 1\f$
VSMC_DEFINE_MATH_VMATH_1(std::expm1, expm1)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::log, log)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log_2(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::log2, log2)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log_{10}(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::log10, log10)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \log(a_i + 1)\f$
VSMC_DEFINE_MATH_VMATH_1(std::log1p, log1p)
/// @}

/// \defgroup vTrigonometric Trigonometric functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sin(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::cos, cos)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \cos(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::sin, sin)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \sin(a_i), z_i = \cos(a_i)\f$
template <typename T>
inline void sincos(std::size_t n, const T *a, T *y, T *z)
{
    sin(n, a, y);
    cos(n, a, z);
}

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \tan(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::tan, tan)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arccos(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::acos, acos)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arcsin(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::asin, asin)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arctan(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::atan, atan)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \arctan(a_i / b_i)\f$ with
/// signs to determine the quadrant
VSMC_DEFINE_MATH_VMATH_2(std::atan2, atan2)

/// @}

/// \defgroup vHyperbolic Hyperbolic functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \cosh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::cosh, cosh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \sinh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::sinh, sinh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \tanh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::tanh, tanh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\cosh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::acosh, acosh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\sinh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::asinh, asinh)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{arc}\tanh(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::atanh, atanh)

/// @}

/// \defgroup vSpecial Special functions
/// \ingroup vMath
/// @{

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{Erf}(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::erf, erf)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \mathrm{Erfc}(a_i) = \mathrm{Erf}(1 - a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(std::erfc, erfc)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = 1 - \mathrm{Erfc}(a_i / \sqrt{2}) / 2\f$, the standard Normal CDF
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_cdfnorm, cdfnorm)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \mathrm{Erf}^{-1}(a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_erfinv, erfinv)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \mathrm{Erf}^{-1}(1 - a_i)\f$
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_erfcinv, erfcinv)

/// \brief For \f$i=1,\ldots,n\f$, compute
/// \f$y_i = \sqrt{2}\mathrm{Erf}^{-1}(2a_i - 1)\f$, inverse of the standard
/// Nomral CDF
VSMC_DEFINE_MATH_VMATH_1(internal::vmath_cdfnorminv, cdfnorminv)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \ln\Gamma(a_i)\f$,
/// logarithm of the Gamma function
VSMC_DEFINE_MATH_VMATH_1(std::lgamma, lgamma)

/// \brief For \f$i=1,\ldots,n\f$, compute \f$y_i = \Gamma(a_i)\f$, the Gamma
/// function
VSMC_DEFINE_MATH_VMATH_1(std::tgamma, tgamma)
/// @}

} // namespace vsmc

#if VSMC_USE_MKL_VML

#define VSMC_DEFINE_MATH_VMATH_VML_1(func, name)                              \
    inline void name(std::size_t n, const float *a, float *y)                 \
    {                                                                         \
        ::vs##func(static_cast<MKL_INT>(n), a, y);                            \
    }                                                                         \
    inline void name(std::size_t n, const double *a, double *y)               \
    {                                                                         \
        ::vd##func(static_cast<MKL_INT>(n), a, y);                            \
    }

#define VSMC_DEFINE_MATH_VMATH_VML_2(func, name)                              \
    inline void name(std::size_t n, const float *a, const float *b, float *y) \
    {                                                                         \
        ::vs##func(static_cast<MKL_INT>(n), a, b, y);                         \
    }                                                                         \
    inline void name(                                                         \
        std::size_t n, const double *a, const double *b, double *y)           \
    {                                                                         \
        ::vd##func(static_cast<MKL_INT>(n), a, b, y);                         \
    }

namespace vsmc
{

VSMC_DEFINE_MATH_VMATH_VML_2(Add, add)
VSMC_DEFINE_MATH_VMATH_VML_2(Sub, sub)
VSMC_DEFINE_MATH_VMATH_VML_1(Sqr, sqr)
VSMC_DEFINE_MATH_VMATH_VML_2(Mul, mul)
VSMC_DEFINE_MATH_VMATH_VML_1(Abs, abs)
inline void linear_frac(std::size_t n, const float *a, const float *b,
    float beta_a, float beta_b, float mu_a, float mu_b, float *y)
{
    ::vsLinearFrac(
        static_cast<MKL_INT>(n), a, b, beta_a, beta_b, mu_a, mu_b, y);
}
inline void linear_frac(std::size_t n, const double *a, const double *b,
    double beta_a, double beta_b, double mu_a, double mu_b, double *y)
{
    ::vdLinearFrac(
        static_cast<MKL_INT>(n), a, b, beta_a, beta_b, mu_a, mu_b, y);
}

VSMC_DEFINE_MATH_VMATH_VML_1(Inv, inv)
VSMC_DEFINE_MATH_VMATH_VML_2(Div, div)
VSMC_DEFINE_MATH_VMATH_VML_1(Sqrt, sqrt)
VSMC_DEFINE_MATH_VMATH_VML_1(InvSqrt, invsqrt)
VSMC_DEFINE_MATH_VMATH_VML_1(Cbrt, cbrt)
VSMC_DEFINE_MATH_VMATH_VML_1(InvCbrt, invcbrt)
VSMC_DEFINE_MATH_VMATH_VML_1(Pow2o3, pow2o3)
VSMC_DEFINE_MATH_VMATH_VML_1(Pow3o2, pow3o2)
VSMC_DEFINE_MATH_VMATH_VML_2(Pow, pow)
inline void pow(std::size_t n, const float *a, float b, float *y)
{
    ::vsPowx(static_cast<MKL_INT>(n), a, b, y);
}
inline void pow(std::size_t n, const double *a, double b, double *y)
{
    ::vdPowx(static_cast<MKL_INT>(n), a, b, y);
}
VSMC_DEFINE_MATH_VMATH_VML_2(Hypot, hypot)

VSMC_DEFINE_MATH_VMATH_VML_1(Exp, exp)
VSMC_DEFINE_MATH_VMATH_VML_1(Expm1, expm1)
VSMC_DEFINE_MATH_VMATH_VML_1(Ln, log)
VSMC_DEFINE_MATH_VMATH_VML_1(Log10, log10)
VSMC_DEFINE_MATH_VMATH_VML_1(Log1p, log1p)

VSMC_DEFINE_MATH_VMATH_VML_1(Cos, cos)
VSMC_DEFINE_MATH_VMATH_VML_1(Sin, sin)
inline void sincos(std::size_t n, const float *a, float *y, float *z)
{
    ::vsSinCos(static_cast<MKL_INT>(n), a, y, z);
}
inline void sincos(std::size_t n, const double *a, double *y, double *z)
{
    ::vdSinCos(static_cast<MKL_INT>(n), a, y, z);
}
VSMC_DEFINE_MATH_VMATH_VML_1(Tan, tan)
VSMC_DEFINE_MATH_VMATH_VML_1(Acos, acos)
VSMC_DEFINE_MATH_VMATH_VML_1(Asin, asin)
VSMC_DEFINE_MATH_VMATH_VML_1(Atan, atan)
VSMC_DEFINE_MATH_VMATH_VML_2(Atan2, atan2)

VSMC_DEFINE_MATH_VMATH_VML_1(Cosh, cosh)
VSMC_DEFINE_MATH_VMATH_VML_1(Sinh, sinh)
VSMC_DEFINE_MATH_VMATH_VML_1(Tanh, tanh)
VSMC_DEFINE_MATH_VMATH_VML_1(Acosh, acosh)
VSMC_DEFINE_MATH_VMATH_VML_1(Asinh, asinh)
VSMC_DEFINE_MATH_VMATH_VML_1(Atanh, atanh)

VSMC_DEFINE_MATH_VMATH_VML_1(Erf, erf)
VSMC_DEFINE_MATH_VMATH_VML_1(Erfc, erfc)
VSMC_DEFINE_MATH_VMATH_VML_1(CdfNorm, cdfnorm)
VSMC_DEFINE_MATH_VMATH_VML_1(ErfInv, erfinv)
VSMC_DEFINE_MATH_VMATH_VML_1(ErfcInv, erfcinv)
VSMC_DEFINE_MATH_VMATH_VML_1(CdfNormInv, cdfnorminv)
VSMC_DEFINE_MATH_VMATH_VML_1(LGamma, lgamma)
VSMC_DEFINE_MATH_VMATH_VML_1(TGamma, tgamm)

} // namespace vsmc

#endif // VSMC_USE_MKL_VML

#endif // VSMC_MATH_VMATH_HPP
