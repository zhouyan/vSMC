//============================================================================
// vSMC/include/vsmc/math/cblas.hpp
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

#ifndef VSMC_MATH_CBLAS_HPP
#define VSMC_MATH_CBLAS_HPP

#include <vsmc/internal/config.hpp>
#include <cmath>
#include <numeric>

#if VSMC_USE_MKL_CBLAS
#include <mkl.h>
#define VSMC_CBLAS_INT MKL_INT
#elif VSMC_USE_ACCELERATE_CBLAS
#include <Accelerate/Accelerate.h>
#define VSMC_CBLAS_INT int
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
{return std::inner_product(x, x + n, y, static_cast<T>(0));}

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
{return ::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), x, 1);}              \
inline double name (std::size_t n, const double *x)                          \
{return ::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), x, 1);}

#define VSMC_DEFINE_MATH_CBLAS_S2(name, sname, dname) \
inline float name (std::size_t n, const float *x, const float *y)            \
{return ::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), x, 1, y, 1);}        \
inline double name (std::size_t n, const double *x, const double *y)         \
{return ::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), x, 1, y, 1);}

#define VSMC_DEFINE_MATH_CBLAS_SV(name, sname, dname) \
inline void name (std::size_t n, float a, float *x)                          \
{::cblas_##sname(static_cast<VSMC_CBLAS_INT>(n), a, x, 1);}                  \
inline void name (std::size_t n, double a, double *x)                        \
{::cblas_##dname(static_cast<VSMC_CBLAS_INT>(n), a, x, 1);}

namespace vsmc { namespace math {

VSMC_DEFINE_MATH_CBLAS_S1(asum, sasum, dasum)
VSMC_DEFINE_MATH_CBLAS_S2(dot, sdot, ddot)
VSMC_DEFINE_MATH_CBLAS_SV(scal, sscal, dscal)

} } // namespace vsmc::math

#endif // VSMC_CBLAS_INT

#endif // VSMC_MATH_CBLAS_HPP
