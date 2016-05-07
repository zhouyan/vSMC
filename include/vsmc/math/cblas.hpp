//============================================================================
// vSMC/include/vsmc/math/cblas.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#if VSMC_USE_CBLAS

#if VSMC_USE_MKL_CBLAS
#include <mkl_cblas.h>
#ifndef VSMC_BLAS_INT
#define VSMC_BLAS_INT MKL_INT
#endif
#elif VSMC_USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif

#ifndef VSMC_BLAS_INT
#define VSMC_BLAS_INT int
#endif

namespace vsmc
{

namespace internal
{

using ::CblasRowMajor;
using ::CblasColMajor;

using ::CblasNoTrans;
using ::CblasTrans;
using ::CblasConjTrans;

using ::CblasUpper;
using ::CblasLower;

using ::CblasNonUnit;
using ::CblasUnit;

using ::CblasLeft;
using ::CblasRight;

using ::cblas_sdot;
using ::cblas_ddot;

using ::cblas_sgemv;
using ::cblas_dgemv;

using ::cblas_stpmv;
using ::cblas_dtpmv;

using ::cblas_ssyr;
using ::cblas_dsyr;

using ::cblas_strmm;
using ::cblas_dtrmm;

using ::cblas_ssyrk;
using ::cblas_dsyrk;

} // namespace vsmc::internal

} // namespace vsmc

#else // VSMC_USE_CBLAS

#ifndef VSMC_BLAS_NAME
#ifdef VSMC_BLAS_NAME_NO_UNDERSCORE
#define VSMC_BLAS_NAME(x) x
#else
#define VSMC_BLAS_NAME(x) x##_
#endif
#endif

#ifndef VSMC_BLAS_INT
#define VSMC_BLAS_INT int
#endif

extern "C" {

void VSMC_BLAS_NAME(sgemv)(const char *trans, const VSMC_BLAS_INT *m,
    const VSMC_BLAS_INT *n, const float *alpha, const float *a,
    const VSMC_BLAS_INT *lda, const float *x, const VSMC_BLAS_INT *incx,
    const float *beta, float *y, const VSMC_BLAS_INT *incy);

void VSMC_BLAS_NAME(dgemv)(const char *trans, const VSMC_BLAS_INT *m,
    const VSMC_BLAS_INT *n, const double *alpha, const double *a,
    const VSMC_BLAS_INT *lda, const double *x, const VSMC_BLAS_INT *incx,
    const double *beta, double *y, const VSMC_BLAS_INT *incy);

void VSMC_BLAS_NAME(stpmv)(const char *uplo, const char *trans,
    const char *diag, const VSMC_BLAS_INT *n, const float *ap, float *x,
    const VSMC_BLAS_INT *incx);

void VSMC_BLAS_NAME(dtpmv)(const char *uplo, const char *trans,
    const char *diag, const VSMC_BLAS_INT *n, const double *ap, double *x,
    const VSMC_BLAS_INT *incx);

void VSMC_BLAS_NAME(ssyr)(const char *uplo, const VSMC_BLAS_INT *n,
    const float *alpha, const float *x, const VSMC_BLAS_INT *incx, float *a,
    const VSMC_BLAS_INT *lda);

void VSMC_BLAS_NAME(dsyr)(const char *uplo, const VSMC_BLAS_INT *n,
    const double *alpha, const double *x, const VSMC_BLAS_INT *incx, double *a,
    const VSMC_BLAS_INT *lda);

void VSMC_BLAS_NAME(strmm)(const char *side, const char *uplo,
    const char *transa, const char *diag, const VSMC_BLAS_INT *m,
    const VSMC_BLAS_INT *n, const float *alpha, const float *a,
    const VSMC_BLAS_INT *lda, float *b, const VSMC_BLAS_INT *ldb);

void VSMC_BLAS_NAME(dtrmm)(const char *side, const char *uplo,
    const char *transa, const char *diag, const VSMC_BLAS_INT *m,
    const VSMC_BLAS_INT *n, const double *alpha, const double *a,
    const VSMC_BLAS_INT *lda, double *b, const VSMC_BLAS_INT *ldb);

void VSMC_BLAS_NAME(ssyrk)(const char *uplo, const char *trans,
    const VSMC_BLAS_INT *n, const VSMC_BLAS_INT *k, const float *alpha,
    const float *a, const VSMC_BLAS_INT *lda, const float *beta, float *c,
    const VSMC_BLAS_INT *ldc);

void VSMC_BLAS_NAME(dsyrk)(const char *uplo, const char *trans,
    const VSMC_BLAS_INT *n, const VSMC_BLAS_INT *k, const double *alpha,
    const double *a, const VSMC_BLAS_INT *lda, const double *beta, double *c,
    const VSMC_BLAS_INT *ldc);

} // extern "C"

namespace vsmc
{

namespace internal
{

enum CBLAS_LAYOUT { CblasRowMajor = 101, CblasColMajor = 102 };

using CBLAS_ORDER = CBLAS_LAYOUT;

enum CBLAS_TRANSPOSE {
    CblasNoTrans = 111,
    CblasTrans = 112,
    CblasConjTrans = 113
};

enum CBLAS_UPLO { CblasUpper = 121, CblasLower = 122 };

enum CBLAS_DIAG { CblasNonUnit = 131, CblasUnit = 132 };

enum CBLAS_SIDE { CblasLeft = 141, CblasRight = 142 };

inline char cblas_trans(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE trans)
{
    if (layout == CblasColMajor) {
        if (trans == CblasNoTrans)
            return 'N';
        else if (trans == CblasTrans)
            return 'T';
        else if (trans == CblasConjTrans)
            return 'C';
    }
    if (layout == CblasRowMajor) {
        if (trans == CblasNoTrans)
            return 'T';
        else if (trans == CblasTrans)
            return 'N';
        else if (trans == CblasConjTrans)
            return 'N';
    }
    return 'N';
}

inline char cblas_uplo(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo)
{
    if (layout == CblasColMajor) {
        if (uplo == CblasUpper)
            return 'U';
        if (uplo == CblasLower)
            return 'L';
    }
    if (layout == CblasRowMajor) {
        if (uplo == CblasUpper)
            return 'L';
        if (uplo == CblasLower)
            return 'U';
    }
    return 'U';
}

inline char cblas_diag(const CBLAS_DIAG diag)
{
    if (diag == CblasUnit)
        return 'U';
    if (diag == CblasNonUnit)
        return 'N';
    return 'N';
}

inline char cblas_side(const CBLAS_LAYOUT layout, const CBLAS_SIDE side)
{
    if (layout == CblasColMajor) {
        if (side == CblasLeft)
            return 'L';
        if (side == CblasRight)
            return 'R';
    }
    if (layout == CblasRowMajor) {
        if (side == CblasLeft)
            return 'R';
        if (side == CblasRight)
            return 'L';
    }
    return 'L';
}

inline float cblas_sdot(const VSMC_BLAS_INT n, const float *x,
    const VSMC_BLAS_INT incx, const float *y, const VSMC_BLAS_INT incy)
{
    float s = 0;
    for (VSMC_BLAS_INT i = 0; i != n; ++i, x += incx, y += incy)
        s += (*x) * (*y);

    return s;
}

inline double cblas_ddot(const VSMC_BLAS_INT n, const double *x,
    const VSMC_BLAS_INT incx, const double *y, const VSMC_BLAS_INT incy)
{
    double s = 0;
    for (VSMC_BLAS_INT i = 0; i != n; ++i, x += incx, y += incy)
        s += (*x) * (*y);

    return s;
}

inline void cblas_sgemv(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE trans,
    const VSMC_BLAS_INT m, const VSMC_BLAS_INT n, const float alpha,
    const float *a, const VSMC_BLAS_INT lda, const float *x,
    const VSMC_BLAS_INT incx, const float beta, float *y,
    const VSMC_BLAS_INT incy)
{
    const char transf = cblas_trans(layout, trans);
    VSMC_BLAS_NAME(sgemv)
    (&transf, &m, &n, &alpha, a, &lda, x, &incx, &beta, y, &incy);
}

inline void cblas_dgemv(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE trans,
    const VSMC_BLAS_INT m, const VSMC_BLAS_INT n, const double alpha,
    const double *a, const VSMC_BLAS_INT lda, const double *x,
    const VSMC_BLAS_INT incx, const double beta, double *y,
    const VSMC_BLAS_INT incy)
{
    const char transf = cblas_trans(layout, trans);
    VSMC_BLAS_NAME(dgemv)
    (&transf, &m, &n, &alpha, a, &lda, x, &incx, &beta, y, &incy);
}

inline void cblas_stpmv(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const CBLAS_TRANSPOSE trans, const CBLAS_DIAG diag, const VSMC_BLAS_INT n,
    const float *ap, float *x, const VSMC_BLAS_INT incx)
{
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(layout, trans);
    const char diagf = cblas_diag(diag);
    VSMC_BLAS_NAME(stpmv)(&uplof, &transf, &diagf, &n, ap, x, &incx);
}

inline void cblas_dtpmv(CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const CBLAS_TRANSPOSE trans, const CBLAS_DIAG diag, const VSMC_BLAS_INT n,
    const double *ap, double *x, const VSMC_BLAS_INT incx)
{
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(layout, trans);
    const char diagf = cblas_diag(diag);
    VSMC_BLAS_NAME(dtpmv)(&uplof, &transf, &diagf, &n, ap, x, &incx);
}

inline void cblas_ssyr(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const VSMC_BLAS_INT n, const float alpha, const float *x,
    const VSMC_BLAS_INT incx, float *a, const VSMC_BLAS_INT lda)
{
    const char uplof = cblas_uplo(layout, uplo);
    VSMC_BLAS_NAME(ssyr)(&uplof, &n, &alpha, x, &incx, a, &lda);
}

inline void cblas_dsyr(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const VSMC_BLAS_INT n, const double alpha, const double *x,
    const VSMC_BLAS_INT incx, double *a, const VSMC_BLAS_INT lda)
{
    const char uplof = cblas_uplo(layout, uplo);
    VSMC_BLAS_NAME(dsyr)(&uplof, &n, &alpha, x, &incx, a, &lda);
}

inline void cblas_strmm(const CBLAS_LAYOUT layout, const CBLAS_SIDE side,
    const CBLAS_UPLO uplo, const CBLAS_TRANSPOSE trans, const CBLAS_DIAG diag,
    const VSMC_BLAS_INT m, const VSMC_BLAS_INT n, const float alpha,
    const float *a, const VSMC_BLAS_INT lda, float *b, const VSMC_BLAS_INT ldb)
{
    const char sidef = cblas_side(layout, side);
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(CblasColMajor, trans);
    const char diagf = cblas_diag(diag);
    VSMC_BLAS_NAME(strmm)
    (&sidef, &uplof, &transf, &diagf, &m, &n, &alpha, a, &lda, b, &ldb);
}

inline void cblas_dtrmm(const CBLAS_LAYOUT layout, const CBLAS_SIDE side,
    const CBLAS_UPLO uplo, const CBLAS_TRANSPOSE trans, const CBLAS_DIAG diag,
    const VSMC_BLAS_INT m, const VSMC_BLAS_INT n, const double alpha,
    const double *a, const VSMC_BLAS_INT lda, double *b,
    const VSMC_BLAS_INT ldb)
{
    const char sidef = cblas_side(layout, side);
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(CblasColMajor, trans);
    const char diagf = cblas_diag(diag);
    VSMC_BLAS_NAME(dtrmm)
    (&sidef, &uplof, &transf, &diagf, &m, &n, &alpha, a, &lda, b, &ldb);
}

inline void cblas_ssyrk(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const CBLAS_TRANSPOSE trans, const VSMC_BLAS_INT n, const VSMC_BLAS_INT k,
    const float alpha, const float *a, const VSMC_BLAS_INT lda,
    const float beta, float *c, const VSMC_BLAS_INT ldc)
{
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(layout, trans);
    VSMC_BLAS_NAME(ssyrk)
    (&uplof, &transf, &n, &k, &alpha, a, &lda, &beta, c, &ldc);
}

inline void cblas_dsyrk(const CBLAS_LAYOUT layout, const CBLAS_UPLO uplo,
    const CBLAS_TRANSPOSE trans, const VSMC_BLAS_INT n, const VSMC_BLAS_INT k,
    const double alpha, const double *a, const VSMC_BLAS_INT lda,
    const double beta, double *c, const VSMC_BLAS_INT ldc)
{
    const char uplof = cblas_uplo(layout, uplo);
    const char transf = cblas_trans(layout, trans);
    VSMC_BLAS_NAME(dsyrk)
    (&uplof, &transf, &n, &k, &alpha, a, &lda, &beta, c, &ldc);
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_USE_CBLAS

#endif // VSMC_MATH_CBLAS_HPP
