//============================================================================
// vSMC/include/vsmc/rng/normal_mv_distribution.hpp
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

#ifndef VSMC_RNG_NORMAL_MV_DISTRIBUTION_HPP
#define VSMC_RNG_NORMAL_MV_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>

#define VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_FIXED_DIM(Dim)          \
    VSMC_STATIC_ASSERT((Dim != Dynamic),                                      \
        "**NormalMVDistribution** OBJECT DECLARED WITH DYNAMIC DIMENSION")

#define VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_DYNAMIC_DIM(Dim)        \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                      \
        "**NormalMVDistribution** OBJECT DECLARED WITH FIXED DIMENSION")

namespace vsmc
{

namespace internal
{

template <typename MatrixType>
inline bool normal_mv_distribution_check_param(const MatrixType &)
{
    return true;
}

} // namespace vsmc::internal

/// \brief Multivariate Normal distribution
/// \ingroup Distribution
///
/// \details
/// The distribution is parameterized by its mean vector and the lower
/// triangular elements of the Cholesky decomposition of the covaraince matrix,
/// packed row by row.
///
/// \tparam RealType Only `float` and `double` are supported.
/// \tparam Dim If `Dim > 0`, then the distribution has a static size and does
/// not use dynamic memory. If `Dim == Dynamic`, then the dimension of the
/// distribution is specified at runtime.
template <typename RealType, std::size_t Dim>
class NormalMVDistribution
{
    private:
    using vector_type = typename std::conditional<Dim == Dynamic,
        Vector<RealType>, std::array<RealType, Dim>>::type;
    using matrix_type = typename std::conditional<Dim == Dynamic,
        Vector<RealType>, std::array<RealType, Dim *(Dim + 1) / 2>>::type;

    template <typename T, std::size_t N>
    static void resize(std::array<T, N> &, std::size_t)
    {
    }

    template <typename T>
    static void resize(Vector<T> &vec, std::size_t n)
    {
        vec.resize(n);
    }

    public:
    using result_type = RealType;
    using distribution_type = NormalMVDistribution<RealType, Dim>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = NormalMVDistribution<RealType, Dim>;

        explicit param_type(const result_type *mean = nullptr,
            const result_type *chol = nullptr)
        {
            VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_FIXED_DIM(Dim);

            init(mean, chol);
        }

        explicit param_type(std::size_t dim, const result_type *mean = nullptr,
            const result_type *chol = nullptr)
            : mean_(dim), chol_(dim * (dim + 1) / 2)
        {
            VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_DYNAMIC_DIM(Dim);

            init(mean, chol);
        }

        std::size_t dim() const { return mean_.size(); }
        const result_type *mean() const { return mean_.data(); }
        const result_type *chol() const { return chol_.data(); }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.mean_ != param2.mean_)
                return false;
            if (param1.chol_ != param2.chol_)
                return false;
            return true;
        }

        friend bool operator!=(
            const param_type &param1, const param_type &param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.dim() << ' ';
            os << param.mean_ << ' ';
            os << param.chol_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, const param_type &param)
        {
            if (!is.good())
                return is;

            std::size_t dim = 0;
            is >> std::ws >> dim;
            if (!is.good())
                return is;

            vector_type mean;
            matrix_type chol;
            resize(mean, dim);
            resize(chol, dim * (dim + 1) / 2);
            is >> std::ws >> mean;
            is >> std::ws >> chol;

            if (is.good()) {
                if (internal::normal_mv_distribution_check_param(mean, chol)) {
                    param.mean_ = std::move(mean);
                    param.chol_ = std::move(chol);
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        vector_type mean_;
        matrix_type chol_;

        void init(const result_type *mean, const result_type *chol)
        {
            if (mean == nullptr)
                std::fill(mean_.begin(), mean_.end(), 0);
            else
                std::copy_n(mean, mean_.size(), mean_.begin());

            if (chol == nullptr)
                std::fill(chol_.begin(), chol_.end(), 0);
            else
                std::copy_n(chol, chol_.size(), chol_.begin());

            if (chol == nullptr)
                for (std::size_t i = 0; i != chol_.size(); ++i)
                    chol_[i * (i + 1) / 2 + i] = 1;

            VSMC_RUNTIME_ASSERT_RNG_DISTRIBUTION_PARAM(
                internal::normal_mv_distribution_check_param(chol_), NormalMV);
        }
    }; // class param_type

    /// \brief Only usable when `Dim > 0`
    explicit NormalMVDistribution(
        const result_type *mean = nullptr, const result_type *chol = nullptr)
        : param_(mean, chol), rnorm_(0, 1)
    {
        reset();
    }

    /// \brief Only usable when `Dim == Dynamic`
    explicit NormalMVDistribution(std::size_t dim,
        const result_type *mean = nullptr, const result_type *chol = nullptr)
        : param_(dim, mean, chol), rnorm_(0, 1)
    {
        reset();
    }

    std::size_t dim() const { return param_.dim(); }
    const result_type *mean() const { return param_.mean(); }
    const result_type *chol() const { return param_.chol(); }

    param_type param() const { return param_; }

    void param(const param_type &param)
    {
        param_ = param;
        reset();
    }

    template <typename RNGType>
    void operator()(RNGType &rng, result_type *r)
    {
        operator()(rng, r, param_);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, result_type *r, const param_type &param)
    {
        generate(rng, r, param);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        operator()(rng, n, r, param_);
    }

    template <typename RNGType>
    void operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &param)
    {
        normal_mv_distribution(
            rng, n, r, param.dim(), param.mean(), param.chol());
    }

    void min VSMC_MNE(result_type *x) const
    {
        std::fill_n(
            x, dim(), -std::numeric_limits<result_type>::max VSMC_MNE());
    }

    void max VSMC_MNE(result_type *x) const
    {
        std::fill_n(
            x, dim(), std::numeric_limits<result_type>::max VSMC_MNE());
    }

    void reset() { rnorm_.reset(); }

    friend bool operator==(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        if (dist1.param_ != dist2.param_)
            return false;
        if (dist1.rnorm_ != dist2.rnorm_)
            return false;
        return true;
    }

    friend bool operator!=(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        return !(dist1 == dist2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist)
    {
        if (!os.good())
            return os;

        os << dist.param_ << ' ';
        os << dist.rnorm_ << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)
    {
        if (!is.good())
            return is;

        param_type param;
        NormalDistribution<RealType> rnorm;
        is >> std::ws >> param;
        is >> std::ws >> rnorm;
        if (is.good()) {
            dist.param_ = std::move(param);
            dist.rnorm_ = std::move(rnorm);
        }

        return is;
    }

    /// \brief Compute Cholesky decomposition of the covariance matrix
    ///
    /// \param dim The number of rows of the covariance matrix
    /// \param cov The covariance matrix
    /// \param chol The output lower triangular elements of the Cholesky
    /// decomposition, packed row by row. This can be directly used as the
    /// input parameter of the NormalMVDistribution constructors.
    /// \param layout The storage layout of the covariance matrix
    /// \param upper If `true`, the upper triangular of the covariance matrix
    /// shall be used. Otherwise the lower triangular shall be used.
    /// \param packed If the upper or lower triangular of covariance matrix is
    /// packed, row by row if `layout == RowMajor`, or column by column if
    /// `layout == ColMajor`.
    ///
    /// \return
    /// - `0` If successful
    /// - Positive value `i` if the `i`th minor of the covariance matrix is not
    /// psotive-definite.
    static int cov_chol(std::size_t dim, const result_type *cov,
        result_type *chol, MatrixLayout layout = RowMajor, bool upper = false,
        bool packed = false)
    {
        unsigned l = layout == RowMajor ? 0 : 1;
        unsigned u = upper ? 1 : 0;
        unsigned p = packed ? 1 : 0;
        unsigned c = (l << 2) + (u << 1) + p;
        std::size_t k = 0;
        switch (c) {
            case 0: // Row, Lower, Full
                k = 0;
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        chol[k++] = cov[i * dim + j];
                break;
            case 1: // Row, Lower, Pack
                std::copy_n(cov, dim * (dim + 1) / 2, chol);
                break;
            case 2: // Row, Upper, Full
                k = 0;
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        chol[k++] = cov[j * dim + i];
                break;
            case 3: // Row, Upper, Pack
                k = 0;
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        chol[k++] = cov[dim * j - j * (j + 1) / 2 + i];
                break;
            case 4: // Col, Lower, Full
                k = 0;
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        chol[k++] = cov[j * dim + i];
                break;
            case 5: // Col, Lower, Pack
                k = 0;
                for (std::size_t i = 0; i != dim; ++i)
                    for (std::size_t j = 0; j <= i; ++j)
                        chol[k++] = cov[dim * j - j * (j + 1) / 2 + i];
                break;
            case 6: // Col, Upper, Full
                k = 0;
                for (std::size_t j = 0; j != dim; ++j)
                    for (std::size_t i = 0; i <= j; ++i)
                        chol[k++] = cov[j * dim + i];
                break;
            case 7: // Col, Upper, Pack
                std::copy_n(cov, dim * (dim + 1) / 2, chol);
                break;
            default: break;
        }

        return static_cast<int>(LAPACKE_dpptrf(
            LAPACK_ROW_MAJOR, 'L', static_cast<lapack_int>(dim), chol));
    }

    private:
    param_type param_;
    NormalDistribution<RealType> rnorm_;

    template <typename RNGType>
    void generate(RNGType &rng, result_type *r, const param_type &param)
    {
        rnorm_(rng, param.dim(), r);
        mulchol(r, param);
        add(param.dim(), param.mean(), r, r);
    }

    void mulchol(float *r, const param_type &param)
    {
        ::cblas_stpmv(::CblasRowMajor, ::CblasLower, ::CblasNoTrans,
            ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(dim()), param.chol(),
            r, 1);
    }

    void mulchol(double *r, const param_type &param)
    {
        ::cblas_dtpmv(::CblasRowMajor, ::CblasLower, ::CblasNoTrans,
            ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(dim()), param.chol(),
            r, 1);
    }
}; // class NormalMVDistribution

namespace internal
{

inline void normal_mv_distribution_mulchol(
    std::size_t n, float *r, std::size_t m, const float *chol)
{
    ::cblas_strmm(::CblasRowMajor, ::CblasRight, ::CblasLower, ::CblasTrans,
        ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(n),
        static_cast<VSMC_CBLAS_INT>(m), 1, chol,
        static_cast<VSMC_CBLAS_INT>(m), r, static_cast<VSMC_CBLAS_INT>(m));
}

inline void normal_mv_distribution_mulchol(
    std::size_t n, double *r, std::size_t m, const double *chol)
{
    ::cblas_dtrmm(::CblasRowMajor, ::CblasRight, ::CblasLower, ::CblasTrans,
        ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(n),
        static_cast<VSMC_CBLAS_INT>(m), 1, chol,
        static_cast<VSMC_CBLAS_INT>(m), r, static_cast<VSMC_CBLAS_INT>(m));
}

} // namespace vsmc::internal

/// \brief Generating multivariate Normal random varaites
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void normal_mv_distribution(RNGType &rng, std::size_t n, RealType *r,
    std::size_t dim, const RealType *mean, const RealType *chol)
{
    Vector<RealType> cholf(dim * dim);
    for (std::size_t i = 0; i != dim; ++i)
        for (std::size_t j = 0; j <= i; ++j)
            cholf[i * dim + j] = *chol++;
    normal_distribution(rng, n * dim, r, 0.0, 1.0);
    internal::normal_mv_distribution_mulchol(n, r, dim, cholf.data());
    for (std::size_t i = 0; i != n; ++i, r += dim)
        add(dim, mean, r, r);
}

template <typename RealType, std::size_t Dim, typename RNGType>
inline void rng_rand(RNGType &rng, NormalMVDistribution<RealType, Dim> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_NORMAL_DISTRIBUTION_HPP
