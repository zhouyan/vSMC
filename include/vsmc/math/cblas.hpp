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

#include <vsmc/internal/config.h>
#include <vsmc/internal/defines.hpp>
#include <cmath>
#include <numeric>

#if VSMC_USE_MKL_CBLAS
#include <mkl.h>
#define VSMC_CBLAS_INT MKL_INT
#elif VSMC_HAS_CBLAS
#include <cblas.h>
#ifndef VSMC_CBLAS_INT
#define VSMC_CBLAS_INT int
#endif
#endif

namespace vsmc
{

/// \defgroup CBLAS1 BLAS level 1 routines and functions
/// \ingroup CBLAS
/// @{

/// \brief Computes the sum of magnitudes of the vector elements
template <typename T>
inline T asum(std::size_t n, const T *x, std::size_t incx)
{
    T sum = 0;
    std::size_t j = 0;
    for (std::size_t i = 0; i != n; ++i, j += incx)
        sum += std::fabs(x[j]);

    return sum;
}

/// \brief Computes a vector-scalar product and adds the result to a vector
template <typename T>
inline void axpy(
    std::size_t n, T a, const T *x, std::size_t incx, T *y, std::size_t incy)
{
    std::size_t j = 0;
    std::size_t k = 0;
    for (std::size_t i = 0; i != n; ++i, j += incx, k += incy)
        y[k] += a * x[j];
}

/// \brief Copies vector to another vector
template <typename T>
inline void copy(
    std::size_t n, const T *x, std::size_t incx, T *y, std::size_t incy)
{
    std::size_t j = 0;
    std::size_t k = 0;
    for (std::size_t i = 0; i != n; ++i, j += incx, k += incy)
        y[k] = x[j];
}

/// \brief Computes a vector-vector dot product
template <typename T>
inline T dot(
    std::size_t n, const T *x, std::size_t incx, const T *y, std::size_t incy)
{
    T sum = 0;
    std::size_t j = 0;
    std::size_t k = 0;
    for (std::size_t i = 0; i != n; ++i, j += incx, k += incy)
        sum += x[j] * y[k];

    return sum;
}

/// \brief Computes the Euclidean norm of a vector
template <typename T>
inline T nrm2(std::size_t n, const T *x, std::size_t incx)
{
    return std::sqrt(dot(n, x, incx, x, incx));
}

/// \brief Computes the product of a vector by a scalar
template <typename T>
inline void scal(std::size_t n, T a, T *x, std::size_t incx)
{
    std::size_t j = 0;
    for (std::size_t i = 0; i != n; ++i, j += incx)
        x[j] *= a;
}

/// @}

/// \defgroup CBLAS2 BLAS level 2 routines
/// \ingroup CBLAS
/// @{

/// \brief Computes a matrix-vector product using a general matrix
template <typename T>
inline void gemv(MatrixLayout layout, MatrixTrans trans, std::size_t m,
    std::size_t n, T alpha, const T *A, std::size_t lda, const T *x,
    std::size_t incx, T beta, T *y, std::size_t incy)
{
    std::size_t nrow = trans == NoTrans ? m : n;
    std::size_t ncol = trans == NoTrans ? n : m;

    scal(nrow, beta, y, incy);

    if ((layout == RowMajor && trans == NoTrans) ||
        (layout == ColMajor && trans == Trans)) {
        std::size_t k = 0;
        for (std::size_t r = 0; r != nrow; ++r, k += incy)
            y[k] += alpha * dot<T>(ncol, x, incx, A + r * lda, 1);
    } else {
        std::size_t j = 0;
        for (std::size_t c = 0; c != ncol; ++c, j += incx) {
            std::size_t k = 0;
            std::size_t l = c * lda;
            const double ax = alpha * x[j];
            for (std::size_t r = 0; r != nrow; ++r, ++l, k += incy)
                y[k] += ax * A[l];
        }
    }
}

/// @}

} // namespace vsmc

#ifdef VSMC_CBLAS_INT

namespace vsmc
{

inline float asum(std::size_t n, const float *x, std::size_t incx)
{
    return ::cblas_sasum(
        static_cast<VSMC_CBLAS_INT>(n), x, static_cast<VSMC_CBLAS_INT>(incx));
}

inline double asum(std::size_t n, const double *x, std::size_t incx)
{
    return ::cblas_dasum(
        static_cast<VSMC_CBLAS_INT>(n), x, static_cast<VSMC_CBLAS_INT>(incx));
}

inline void axpy(std::size_t n, float a, const float *x, std::size_t incx,
    float *y, std::size_t incy)
{
    ::cblas_saxpy(static_cast<VSMC_CBLAS_INT>(n), a, x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline void axpy(std::size_t n, double a, const double *x, std::size_t incx,
    double *y, std::size_t incy)
{
    ::cblas_daxpy(static_cast<VSMC_CBLAS_INT>(n), a, x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline void copy(std::size_t n, const float *x, std::size_t incx, float *y,
    std::size_t incy)
{
    ::cblas_scopy(static_cast<VSMC_CBLAS_INT>(n), x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline void copy(std::size_t n, const double *x, std::size_t incx, double *y,
    std::size_t incy)
{
    ::cblas_dcopy(static_cast<VSMC_CBLAS_INT>(n), x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline float dot(std::size_t n, const float *x, std::size_t incx,
    const float *y, std::size_t incy)
{
    return ::cblas_sdot(static_cast<VSMC_CBLAS_INT>(n), x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline double dot(std::size_t n, const double *x, std::size_t incx,
    const double *y, std::size_t incy)
{
    return ::cblas_ddot(static_cast<VSMC_CBLAS_INT>(n), x,
        static_cast<VSMC_CBLAS_INT>(incx), y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline float nrm2(std::size_t n, const float *x, std::size_t incx)
{
    return ::cblas_snrm2(
        static_cast<VSMC_CBLAS_INT>(n), x, static_cast<VSMC_CBLAS_INT>(incx));
}

inline double nrm2(std::size_t n, const double *x, std::size_t incx)
{
    return ::cblas_dnrm2(
        static_cast<VSMC_CBLAS_INT>(n), x, static_cast<VSMC_CBLAS_INT>(incx));
}

inline void scal(std::size_t n, float a, float *x, std::size_t incx)
{
    ::cblas_sscal(static_cast<VSMC_CBLAS_INT>(n), a, x,
        static_cast<VSMC_CBLAS_INT>(incx));
}

inline void scal(std::size_t n, double a, double *x, std::size_t incx)
{
    ::cblas_dscal(static_cast<VSMC_CBLAS_INT>(n), a, x,
        static_cast<VSMC_CBLAS_INT>(incx));
}

inline void gemv(MatrixLayout layout, MatrixTrans trans, std::size_t m,
    std::size_t n, float alpha, const float *A, std::size_t lda,
    const float *x, std::size_t incx, float beta, float *y, std::size_t incy)
{
    ::cblas_sgemv((layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor),
        (trans == NoTrans ? ::CblasNoTrans : ::CblasTrans),
        static_cast<VSMC_CBLAS_INT>(m), static_cast<VSMC_CBLAS_INT>(n), alpha,
        A, static_cast<VSMC_CBLAS_INT>(lda), x,
        static_cast<VSMC_CBLAS_INT>(incx), beta, y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

inline void gemv(MatrixLayout layout, MatrixTrans trans, std::size_t m,
    std::size_t n, double alpha, const double *A, std::size_t lda,
    const double *x, std::size_t incx, double beta, double *y,
    std::size_t incy)
{
    ::cblas_dgemv((layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor),
        (trans == NoTrans ? ::CblasNoTrans : ::CblasTrans),
        static_cast<VSMC_CBLAS_INT>(m), static_cast<VSMC_CBLAS_INT>(n), alpha,
        A, static_cast<VSMC_CBLAS_INT>(lda), x,
        static_cast<VSMC_CBLAS_INT>(incx), beta, y,
        static_cast<VSMC_CBLAS_INT>(incy));
}

} // namespace vsmc

#endif // VSMC_CBLAS_INT

#endif // VSMC_MATH_CBLAS_HPP
