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

namespace vsmc
{

/// \brief Covariance
/// \ingroup Covariance
template <typename RealType = double>
class Covariance
{
    static_assert(internal::is_one_of<RealType, float, double>::value,
        "**Covariance** USED WITH RealType OTHER THAN float OR double");

    public:
    using result_type = RealType;

    /// \brief Compute the sample covariance matrix
    ///
    /// \param layout The storage layout of sample `x`. It is assumed to be an
    /// `n` by `p` matrix.
    /// \param n Sample size. If `n == 0` then no computation is carried out.
    /// \param p Dimension of the random variable. If `p == 0` then no
    /// computation carried out.
    /// \param x The sample matrix. If it is a null pointer, then no
    /// computation is carried out.
    /// \param w The weight vector. If it is a null pointer, then all samples
    /// are assigned weight 1.
    /// \param mean Output storage of the mean. If it is a null pointer, then
    /// it is ignored.
    /// \param cov Output storage of the covarianc matrix. If it is a null
    /// pointer, then it is ignored.
    /// \param cov_layout The storage layout of the covariance matrix.
    /// \param cov_upper If true, then the upper triangular of the covariance
    /// matrix is packed, otherwise the lower triangular is packed. Ignored if
    /// `cov_pack` is `false`.
    /// \param cov_packed If true, then the covariance matrix is packed.
    void operator()(MatrixLayout layout, std::size_t n, std::size_t p,
        const result_type *x, const result_type *w, result_type *mean,
        result_type *cov, MatrixLayout cov_layout = RowMajor,
        bool cov_upper = false, bool cov_packed = false)
    {
        if (n * p == 0)
            return;

        if (x == nullptr)
            return;

        if (mean == nullptr && cov == nullptr)
            return;

        internal::size_check<VSMC_BLAS_INT>(p, "Covariance::operator()");
        internal::size_check<VSMC_BLAS_INT>(n, "Covariance::operator()");

        result_type sw = w == nullptr ?
            static_cast<result_type>(n) :
            std::accumulate(w, w + n, const_zero<result_type>());
        mean_.resize(p);
        if (w == nullptr) {
            if (layout == RowMajor) {
                std::fill(mean_.begin(), mean_.end(), 0);
                for (std::size_t i = 0; i != n; ++i)
                    add(p, x + i * p, mean_.data(), mean_.data());
            } else {
                for (std::size_t i = 0; i != p; ++i) {
                    mean_[i] = std::accumulate(
                        x + i * n, x + (i + 1) * n, const_zero<result_type>());
                }
            }
        } else {
            mean_init(layout, n, p, x, w);
        }
        div(p, mean_.data(), sw, mean_.data());
        if (mean != nullptr)
            std::copy(mean_.begin(), mean_.end(), mean);
        if (cov == nullptr)
            return;

        result_type sw2 =
            w == nullptr ? static_cast<result_type>(n) : swsqr(n, w);
        result_type B = sw / (sw * sw - sw2);
        result_type BW = B * sw;
        cov_.resize(p * p);
        std::fill(cov_.begin(), cov_.end(), 0);
        cov_init(layout, p, static_cast<result_type *>(nullptr));
        if (w == nullptr) {
            cov_update(layout, n, p, x, B, BW);
        } else {
            wsqrt_.resize(n);
            buffer_.resize(n * p);
            sqrt(n, w, wsqrt_.data());
            if (layout == RowMajor) {
                for (std::size_t i = 0; i != n; ++i)
                    mul(p, x + i * p, wsqrt_[i], buffer_.data() + i * p);
            } else {
                for (std::size_t i = 0; i != p; ++i)
                    mul(n, x + i * n, wsqrt_.data(), buffer_.data() + i * n);
            }
            cov_update(layout, n, p, buffer_.data(), B, BW);
        }
        cov_pack(p, cov, layout, cov_layout, cov_upper, cov_packed);
    }

    private:
    Vector<result_type> mean_;
    Vector<result_type> cov_;
    Vector<result_type> wsqrt_;
    Vector<result_type> buffer_;

    void mean_init(MatrixLayout layout, std::size_t n, std::size_t p,
        const float *x, const float *w)
    {
        internal::cblas_sgemv(layout == RowMajor ? internal::CblasRowMajor :
                                                   internal::CblasColMajor,
            internal::CblasTrans, static_cast<VSMC_BLAS_INT>(n),
            static_cast<VSMC_BLAS_INT>(p), 1.0, x,
            static_cast<VSMC_BLAS_INT>(layout == RowMajor ? p : n), w, 1, 0.0,
            mean_.data(), 1);
    }

    void mean_init(MatrixLayout layout, std::size_t n, std::size_t p,
        const double *x, const double *w)
    {
        internal::cblas_dgemv(layout == RowMajor ? internal::CblasRowMajor :
                                                   internal::CblasColMajor,
            internal::CblasTrans, static_cast<VSMC_BLAS_INT>(n),
            static_cast<VSMC_BLAS_INT>(p), 1.0, x,
            static_cast<VSMC_BLAS_INT>(layout == RowMajor ? p : n), w, 1, 0.0,
            mean_.data(), 1);
    }

    static float swsqr(std::size_t n, const float *w)
    {
        return internal::cblas_sdot(static_cast<VSMC_BLAS_INT>(n), w, 1, w, 1);
    }

    static double swsqr(std::size_t n, const double *w)
    {
        return internal::cblas_ddot(static_cast<VSMC_BLAS_INT>(n), w, 1, w, 1);
    }

    void cov_init(MatrixLayout layout, std::size_t p, float *)
    {
        internal::cblas_ssyr(layout == RowMajor ? internal::CblasRowMajor :
                                                  internal::CblasColMajor,
            internal::CblasLower, static_cast<VSMC_BLAS_INT>(p), 1,
            mean_.data(), 1, cov_.data(), static_cast<VSMC_BLAS_INT>(p));
    }

    void cov_init(MatrixLayout layout, std::size_t p, double *)
    {
        internal::cblas_dsyr(layout == RowMajor ? internal::CblasRowMajor :
                                                  internal::CblasColMajor,
            internal::CblasLower, static_cast<VSMC_BLAS_INT>(p), 1,
            mean_.data(), 1, cov_.data(), static_cast<VSMC_BLAS_INT>(p));
    }

    void cov_update(MatrixLayout layout, std::size_t n, std::size_t p,
        const float *x, float B, float BW)
    {
        internal::cblas_ssyrk(layout == RowMajor ? internal::CblasRowMajor :
                                                   internal::CblasColMajor,
            internal::CblasLower, internal::CblasTrans,
            static_cast<VSMC_BLAS_INT>(p), static_cast<VSMC_BLAS_INT>(n), B, x,
            static_cast<VSMC_BLAS_INT>(layout == RowMajor ? p : n), -BW,
            cov_.data(), static_cast<VSMC_BLAS_INT>(p));
    }

    void cov_update(MatrixLayout layout, std::size_t n, std::size_t p,
        const double *x, double B, double BW)
    {
        internal::cblas_dsyrk(layout == RowMajor ? internal::CblasRowMajor :
                                                   internal::CblasColMajor,
            internal::CblasLower, internal::CblasTrans,
            static_cast<VSMC_BLAS_INT>(p), static_cast<VSMC_BLAS_INT>(n), B, x,
            static_cast<VSMC_BLAS_INT>(layout == RowMajor ? p : n), -BW,
            cov_.data(), static_cast<VSMC_BLAS_INT>(p));
    }

    void cov_pack(std::size_t p, result_type *cov, MatrixLayout layout,
        MatrixLayout cov_layout, bool cov_upper, bool cov_packed)
    {
        if (layout == RowMajor)
            for (std::size_t i = 0; i != p; ++i)
                for (std::size_t j = 0; j != i; ++j)
                    cov_[j * p + i] = cov_[i * p + j];

        if (layout == ColMajor)
            for (std::size_t i = 0; i != p; ++i)
                for (std::size_t j = 0; j != i; ++j)
                    cov_[i * p + j] = cov_[j * p + i];

        if (!cov_packed) {
            std::copy(cov_.begin(), cov_.end(), cov);
            return;
        }

        unsigned l = cov_layout == RowMajor ? 0 : 1;
        unsigned u = cov_upper ? 1 : 0;
        unsigned c = (l << 1) + u;
        switch (c) {
            case 0: // Row, Lower, Pack
                for (size_t i = 0; i != p; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        *cov++ = cov_[i * p + j];
                break;
            case 1: // Row, Upper, Pack
                for (std::size_t i = 0; i != p; ++i)
                    for (std::size_t j = i; j != p; ++j)
                        *cov++ = cov_[i * p + j];
                break;
            case 2: // Col, Lower, Pack
                for (std::size_t j = 0; j != p; ++j)
                    for (std::size_t i = j; i != p; ++i)
                        *cov++ = cov_[j * p + i];
                break;
            case 3: // Col, Upper, Pack
                for (std::size_t j = 0; j != p; ++j)
                    for (std::size_t i = 0; i <= j; ++i)
                        *cov++ = cov_[j * p + i];
                break;
            default: break;
        }
    }
}; // class Covariance

} // namespace vsmc

#endif // VSMC_UTILITY_COVARIANCE_HPP
