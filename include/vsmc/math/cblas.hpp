//============================================================================
// include/vsmc/math/cblas.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_MATH_CBLAS_HPP
#define VSMC_MATH_CBLAS_HPP

#include <vsmc/internal/config.hpp>
#include <cmath>

#if VSMC_USE_MKL_CBLAS
#include <mkl_cblas.h>
#define VSMC_CBLAS_INT MKL_INT
#elif VSMC_USE_VECLIB_CBLAS
#include <vecLib/cblas.h>
#define VSMC_CBLAS_INT int
#endif

/// \brief When MKL or vecLib CBLAS is available, the threshold of the number
/// of elements above which these libraries will be used
/// \ingroup Config
#ifndef VSMC_CBLAS_THRESHOLD
#define VSMC_CBLAS_THRESHOLD 1000
#endif

namespace vsmc {

namespace math {

/// \brief Sum of vector magnitudes
/// \ingroup CBLAS
template <typename T>
inline T asum (std::size_t n, const T *x)
{
    using std::fabs;

    T sum = 0;
    for (std::size_t i = 0; i != n; ++i)
        sum += fabs(x[i]);

    return sum;
}

/// \brief The dot product
/// \ingroup CBLAS
template <typename T>
inline T dot (std::size_t n, const T *x, const T *y)
{
    T sum = 0;
    for (std::size_t i = 0; i != n; ++i)
        sum += x[i] * y[i];

    return sum;
}

/// \brief Scale a vector
template <typename T>
inline void scal (std::size_t n, T a, T *x)
{
    for (std::size_t i = 0; i != n; ++i)
        x[i] *= a;
}

} // namespace vsmc::math

} // namespace vsmc

#ifdef VSMC_CBLAS_INT

#define VSMC_DEFINE_MATH_CBLAS_S1(name, sname, dname) \
inline float name (std::size_t n, const float *x)                            \
{                                                                            \
    return n < VSMC_CBLAS_THRESHOLD ?                                        \
        name<float>(n, x):                                                   \
        ::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), x, 1);               \
}                                                                            \
inline double name (std::size_t n, const double *x)                          \
{                                                                            \
    return n < VSMC_CBLAS_THRESHOLD ?                                        \
        name<double>(n, x):                                                  \
        ::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), x, 1);               \
}

#define VSMC_DEFINE_MATH_CBLAS_S2(name, sname, dname) \
inline float name (std::size_t n, const float *x, const float *y)            \
{                                                                            \
    return n < VSMC_CBLAS_THRESHOLD ?                                        \
        name<float>(n, x, y):                                                \
        ::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), x, 1, y, 1);         \
}                                                                            \
inline double name (std::size_t n, const double *x, const double *y)         \
{                                                                            \
    return n < VSMC_CBLAS_THRESHOLD ?                                        \
        name<double>(n, x, y):                                               \
        ::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), x, 1, y, 1);         \
}

#define VSMC_DEFINE_MATH_CBLAS_SV(name, sname, dname) \
inline void name (std::size_t n, float a, float *x)                          \
{                                                                            \
    n < VSMC_CBLAS_THRESHOLD ?                                               \
        name<float>(n, a, x):                                                \
        ::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), a, x, 1);            \
}                                                                            \
inline void name (std::size_t n, double a, double *x)                        \
{                                                                            \
    n < VSMC_CBLAS_THRESHOLD ?                                               \
        name<double>(n, a, x):                                               \
        ::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), a, x, 1);            \
}

namespace vsmc { namespace math {

VSMC_DEFINE_MATH_CBLAS_S1(asum, sasum, dasum)
VSMC_DEFINE_MATH_CBLAS_S2(dot, sdot, ddot)
VSMC_DEFINE_MATH_CBLAS_SV(scal, sscal, dscal)

} } // namespace vsmc::math

#endif // VSMC_CBLAS_INT

#endif // VSMC_MATH_CBLAS_HPP
