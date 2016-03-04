//============================================================================
// vSMC/include/vsmc/utility/covariance.hpp
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

#ifndef VSMC_UTILITY_COVARIANCE_HPP
#define VSMC_UTILITY_COVARIANCE_HPP

#include <vsmc/internal/common.hpp>
#if VSMC_USE_MKL_VSL
#include <vsmc/utility/mkl.hpp>
#endif

namespace vsmc
{

namespace internal
{
template <typename RealType>
inline void cov_pack(std::size_t dim, const RealType *cov, RealType *chol,
    MatrixLayout layout, bool upper, bool packed)
{
    unsigned l = layout == RowMajor ? 0 : 1;
    unsigned u = upper ? 1 : 0;
    unsigned p = packed ? 1 : 0;
    unsigned c = (l << 2) + (u << 1) + p;
    switch (c) {
        case 0: // Row, Lower, Full
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j <= i; ++j)
                    *chol++ = cov[i * dim + j];
            break;
        case 1: // Row, Lower, Pack
            std::copy_n(cov, dim * (dim + 1) / 2, chol);
            break;
        case 2: // Row, Upper, Full
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j <= i; ++j)
                    *chol++ = cov[j * dim + i];
            break;
        case 3: // Row, Upper, Pack
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j <= i; ++j)
                    *chol++ = cov[dim * j - j * (j + 1) / 2 + i];
            break;
        case 4: // Col, Lower, Full
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j <= i; ++j)
                    *chol++ = cov[j * dim + i];
            break;
        case 5: // Col, Lower, Pack
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j <= i; ++j)
                    *chol++ = cov[dim * j - j * (j + 1) / 2 + i];
            break;
        case 6: // Col, Upper, Full
            for (std::size_t j = 0; j != dim; ++j)
                for (std::size_t i = 0; i <= j; ++i)
                    *chol++ = cov[j * dim + i];
            break;
        case 7: // Col, Upper, Pack
            std::copy_n(cov, dim * (dim + 1) / 2, chol);
            break;
        default: break;
    }
}

inline int cov_chol(std::size_t dim, float *chol)
{
    return static_cast<int>(::LAPACKE_spptrf(
        LAPACK_ROW_MAJOR, 'L', static_cast<lapack_int>(dim), chol));
}

inline int cov_chol(std::size_t dim, double *chol)
{
    return static_cast<int>(::LAPACKE_dpptrf(
        LAPACK_ROW_MAJOR, 'L', static_cast<lapack_int>(dim), chol));
}

} // namespace vsmc::internal

/// \brief Compute Cholesky decomposition of the covariance matrix
/// \ingroup Covariance
///
/// \param dim The number of rows of the covariance matrix
/// \param cov The covariance matrix
/// \param chol The output lower triangular elements of the Cholesky
/// decomposition, packed row by row. This can be directly used as the input
/// parameter of the NormalMVDistribution constructors.
/// \param layout The storage layout of the covariance matrix
/// \param upper If `true`, the upper triangular of the covariance matrix shall
/// be used. Otherwise the lower triangular shall be used.
/// \param packed If the upper or lower triangular of covariance matrix is
/// packed, row by row if `layout == RowMajor`, or column by column if `layout
/// == ColMajor`.
///
/// \return
/// - `0` If successful
/// - Positive value `i` if the `i`th minor of the covariance matrix is not
/// psotive-definite.
template <typename RealType>
inline int cov_chol(std::size_t dim, const RealType *cov, RealType *chol,
    MatrixLayout layout = RowMajor, bool upper = false, bool packed = false)
{
    internal::cov_pack(dim, cov, chol, layout, upper, packed);
    return internal::cov_chol(dim, chol);
}

/// \brief Covariance
/// \ingroup Covariance
template <typename RealType = double>
class Covariance
{
    public:
    using result_type = RealType;

    /// \brief Compute the sample covariance matrix
    ///
    /// \param layout The storage layout of the data `x`. The data is considere
    /// to be `N` by `dim` matrix. In `RowMajor` storage.
    /// \param n Sample size. If `N == 0` then no computation is carried out.
    /// \param dim Dimension of the random variable. If `dim == 0` then no
    /// computation carried out.
    /// \param x The samples. If it is a null pointer, then no computation is
    /// carried out.
    /// \param mean Output storage of the mean. If it is a null pointer, then
    /// it is ignored.
    /// \param cov Output storage of the covarianc matrix. If it is a null
    /// pointer, then it is ignored.
    /// \param cov_layout The storage layout of the covariance matrix.
    /// \param cov_upper If true, the upper triangular of the covariance matrix
    /// is packed, otherwise the lower triangular is packed. Ignored if
    /// `cov_pack` is `false`.
    /// \param cov_packed If true, the matrix is packed.
    void operator()(MatrixLayout layout, std::size_t n, std::size_t dim,
        const result_type *x, const result_type *w, result_type *mean,
        result_type *cov, MatrixLayout cov_layout = RowMajor,
        bool cov_upper = false, bool cov_packed = false)
    {
        if (n * dim == 0)
            return;

        if (x == nullptr)
            return;

        if (mean == nullptr && cov == nullptr)
            return;

#if VSMC_USE_MKL_VSL
        MKL_INT px = static_cast<MKL_INT>(dim);
        MKL_INT nx = static_cast<MKL_INT>(n);
        MKL_INT xstorage = layout == RowMajor ? VSL_SS_MATRIX_STORAGE_COLS :
                                                VSL_SS_MATRIX_STORAGE_ROWS;
        MKL_INT cov_storage = storage(cov_layout, cov_upper, cov_packed);
        unsigned MKL_INT64 estimates = 0;
        if (mean != nullptr)
            estimates |= VSL_SS_MEAN;
        if (cov != nullptr)
            estimates |= VSL_SS_COV;

        MKLSSTask<result_type> task(&px, &nx, &xstorage, x, w, nullptr);
        task.edit_cov_cor(mean, cov, &cov_storage, nullptr, nullptr);
        task.compute(estimates, VSL_SS_METHOD_FAST);
#else  // VSMC_USE_MKL_VSL
        result_type sw = w == nullptr ?
            static_cast<result_type>(n) :
            std::accumulate(w, w + n, static_cast<result_type>(0));
        mean_.resize(dim);
        if (w == nullptr) {
            if (layout == RowMajor) {
                std::fill(mean_.begin(), mean_.end(), 0);
                for (std::size_t i = 0; i != n; ++i)
                    add(dim, x + i * dim, mean_.data(), mean_.data());
            } else {
                for (std::size_t i = 0; i != dim; ++i) {
                    mean_[i] = std::accumulate(x + i * n, x + (i + 1) * n,
                        static_cast<result_type>(0));
                }
            }
        } else {
            mean_init(layout, n, dim, x, w);
        }
        div(dim, mean_.data(), sw, mean_.data());
        if (mean != nullptr)
            std::copy(mean_.begin(), mean_.end(), mean);
        if (cov == nullptr)
            return;

        result_type sw2 =
            w == nullptr ? static_cast<result_type>(n) : sw2 = swsqr(n, w);
        result_type B = sw / (sw * sw - sw2);
        result_type BW = B * sw;
        cov_.resize(dim * dim);
        std::fill(cov_.begin(), cov_.end(), 0);
        cov_init(layout, dim, static_cast<result_type *>(nullptr));
        if (w == nullptr) {
            cov_update(layout, n, dim, x, B, BW);
        } else {
            wsqrt_.resize(n);
            buffer_.resize(n * dim);
            sqrt(n, w, wsqrt_.data());
            if (layout == RowMajor) {
                for (std::size_t i = 0; i != n; ++i)
                    mul(dim, x + i * dim, wsqrt_[i], buffer_.data() + i * dim);
            } else {
                for (std::size_t i = 0; i != dim; ++i)
                    mul(n, x + i * n, wsqrt_.data(), buffer_.data() + i * n);
            }
            cov_update(layout, n, dim, buffer_.data(), B, BW);
        }
        cov_pack(dim, cov, layout, cov_layout, cov_upper, cov_packed);
#endif // VSMC_USE_MKL_VSL
    }

    private:
#if VSMC_USE_MKL_VSL
    MKL_INT storage(MatrixLayout layout, bool upper, bool packed)
    {
        if (!packed)
            return VSL_SS_MATRIX_STORAGE_FULL;

        if (layout == RowMajor)
            return upper ? VSL_SS_MATRIX_STORAGE_U_PACKED :
                           VSL_SS_MATRIX_STORAGE_L_PACKED;

        return upper ? VSL_SS_MATRIX_STORAGE_L_PACKED :
                       VSL_SS_MATRIX_STORAGE_U_PACKED;
    }
#else  // VSMC_USE_MKL_VSL
    Vector<result_type> mean_;
    Vector<result_type> cov_;
    Vector<result_type> wsqrt_;
    Vector<result_type> buffer_;

    void mean_init(MatrixLayout layout, std::size_t n, std::size_t dim,
        const float *x, const float *w)
    {
        ::cblas_sgemv(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasTrans, static_cast<VSMC_CBLAS_INT>(n),
            static_cast<VSMC_CBLAS_INT>(dim), 1.0, x,
            static_cast<VSMC_CBLAS_INT>(layout == RowMajor ? dim : n), w, 1,
            0.0, mean_.data(), 1);
    }

    void mean_init(MatrixLayout layout, std::size_t n, std::size_t dim,
        const double *x, const double *w)
    {
        ::cblas_dgemv(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasTrans, static_cast<VSMC_CBLAS_INT>(n),
            static_cast<VSMC_CBLAS_INT>(dim), 1.0, x,
            static_cast<VSMC_CBLAS_INT>(layout == RowMajor ? dim : n), w, 1,
            0.0, mean_.data(), 1);
    }

    static float swsqr(std::size_t n, const float *w)
    {
        return ::cblas_sdot(static_cast<VSMC_CBLAS_INT>(n), w, 1, w, 1);
    }

    static double swsqr(std::size_t n, const double *w)
    {
        return ::cblas_ddot(static_cast<VSMC_CBLAS_INT>(n), w, 1, w, 1);
    }

    void cov_init(MatrixLayout layout, std::size_t dim, float *)
    {
        ::cblas_ssyr(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasLower, static_cast<VSMC_CBLAS_INT>(dim), 1, mean_.data(), 1,
            cov_.data(), static_cast<VSMC_CBLAS_INT>(dim));
    }

    void cov_init(MatrixLayout layout, std::size_t dim, double *)
    {
        ::cblas_dsyr(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasLower, static_cast<VSMC_CBLAS_INT>(dim), 1, mean_.data(), 1,
            cov_.data(), static_cast<VSMC_CBLAS_INT>(dim));
    }

    void cov_update(MatrixLayout layout, std::size_t n, std::size_t dim,
        const float *x, float B, float BW)
    {
        ::cblas_ssyrk(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasLower, ::CblasTrans, static_cast<VSMC_CBLAS_INT>(dim),
            static_cast<VSMC_CBLAS_INT>(n), B, x,
            static_cast<VSMC_CBLAS_INT>(layout == RowMajor ? dim : n), -BW,
            cov_.data(), static_cast<VSMC_CBLAS_INT>(dim));
    }

    void cov_update(MatrixLayout layout, std::size_t n, std::size_t dim,
        const double *x, double B, double BW)
    {
        ::cblas_dsyrk(layout == RowMajor ? ::CblasRowMajor : ::CblasColMajor,
            ::CblasLower, ::CblasTrans, static_cast<VSMC_CBLAS_INT>(dim),
            static_cast<VSMC_CBLAS_INT>(n), B, x,
            static_cast<VSMC_CBLAS_INT>(layout == RowMajor ? dim : n), -BW,
            cov_.data(), static_cast<VSMC_CBLAS_INT>(dim));
    }

    void cov_pack(std::size_t dim, result_type *cov, MatrixLayout layout,
        MatrixLayout cov_layout, bool cov_upper, bool cov_packed)
    {
        if (layout == RowMajor)
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j != i; ++j)
                    cov_[j * dim + i] = cov_[i * dim + j];

        if (layout == ColMajor)
            for (std::size_t i = 0; i != dim; ++i)
                for (std::size_t j = 0; j != i; ++j)
                    cov_[i * dim + j] = cov_[j * dim + i];

        if (!cov_packed) {
            std::copy(cov_.begin(), cov_.end(), cov);
            return;
        }

        unsigned l = cov_layout == RowMajor ? 0 : 1;
        unsigned u = cov_upper ? 1 : 0;
        unsigned c = (l << 1) + u;
        switch (c) {
            case 0: // Row, Lower, Pack
                for (size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        *cov++ = cov_[i * dim + j];
                break;
            case 1: // Row, Upper, Pack
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = i; j != dim; ++j)
                        *cov++ = cov_[i * dim + j];
                break;
            case 2: // Col, Lower, Pack
                for (std::size_t j = 0; j != dim; ++j)
                    for (std::size_t i = j; i != dim; ++i)
                        *cov++ = cov_[j * dim + i];
                break;
            case 3: // Col, Upper, Pack
                for (std::size_t j = 0; j != dim; ++j)
                    for (std::size_t i = 0; i <= j; ++i)
                        *cov++ = cov_[j * dim + i];
                break;
            default: break;
        }
    }
#endif // VSMC_USE_MKL_VSL
};     // class Covariance

} // namespace vsmc

#endif // VSMC_UTILITY_COVARIANCE_HPP
