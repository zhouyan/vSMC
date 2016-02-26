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

template <typename ResultType, typename MatrixType>
inline bool normal_mv_distribution_check_param(
    const ResultType &mean, const MatrixType &chol)
{
    return true;
}

} // namespace vsmc::internal

/// \brief Multivariate Normal distribution
/// \ingroup Distribution
template <typename RealType, std::size_t Dim>
class NormalMVDistribution
{
    private:
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
    using result_type = typename std::conditional<Dim == Dynamic,
        Vector<RealType>, std::array<RealType, Dim>>::type;
    using matrix_type = typename std::conditional<Dim == Dynamic,
        Vector<RealType>, std::array<RealType, Dim *(Dim + 1) / 2>>::type;
    using distribution_type = NormalMVDistribution<RealType, Dim>;

    class param_type
    {
        public:
        using result_type = typename std::conditional<Dim == Dynamic,
            Vector<RealType>, std::array<RealType, Dim>>::type;
        using matrix_type = typename std::conditional<Dim == Dynamic,
            Vector<RealType>, std::array<RealType, Dim *(Dim + 1) / 2>>::type;
        using distribution_type = NormalMVDistribution<RealType, Dim>;

        explicit param_type(
            const RealType *mean = nullptr, const RealType *chol = nullptr)
        {
            VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_FIXED_DIM(Dim);

            init(mean, chol);
        }

        explicit param_type(std::size_t dim = 1,
            const RealType *mean = nullptr, const RealType *chol = nullptr)
            : mean_(dim), chol_(dim * (dim + 1) / 2)
        {
            VSMC_STATIC_ASSERT_RNG_NORMAL_MV_DISTRIBUTION_DYNAMIC_DIM(Dim);

            init(mean, chol);
        }

        std::size_t dim() const { return mean_.size(); }
        const RealType *mean() const { return mean_.data(); }
        const RealType *chol() const { return chol_.data(); }

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

            result_type mean;
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
        result_type mean_;
        matrix_type chol_;

        void init(const RealType *mean, const RealType *chol)
        {
            if (mean == nullptr)
                std::fill(mean_.begin(), mean_.end(), 0.0);
            else
                std::copy_n(mean, mean_.size(), mean_.begin());

            if (chol == nullptr)
                std::fill(chol_.begin(), chol_.end(), 0.0);
            else
                std::copy_n(chol, chol_.size(), chol_.begin());

            if (chol == nullptr)
                for (std::size_t i = 0; i != chol_.size(); ++i)
                    chol_[i * (i + 1) / 2 + i] = 1;
        }
    }; // class param_type

    explicit NormalMVDistribution(
        const RealType *mean = nullptr, const RealType *chol = nullptr)
        : param_(mean, chol), rnorm_(0, 1)
    {
        reset();
    }

    explicit NormalMVDistribution(std::size_t dim = 1,
        const RealType *mean = nullptr, const RealType *chol = nullptr)
        : param_(dim, mean, chol), rnorm_(0, 1)
    {
        reset();
    }

    std::size_t dim() const { return param_.dim(); }
    const RealType *mean() const { return param_.mean(); }
    const RealType *chol() const { return param_.chol(); }

    param_type param() const { return param_; }

    void param(const param_type &param)
    {
        param_ = param;
        reset();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return operator()(rng, param_);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const param_type &param)
    {
        return generate(rng, param);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        operator()(rng, n, r, param_);
    }

    template <typename RNGType>
    result_type operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &param)
    {
        normal_mv_distribution(
            rng, n, r, param.dim(), param.mean(), param.chol());
    }

    result_type min VSMC_MNE() const
    {
        result_type m;
        resize(m, dim());
        std::fill(m.begin(), m.end(),
            -std::numeric_limits<result_type>::max VSMC_MNE());

        return m;
    }

    result_type max VSMC_MNE() const
    {
        result_type m;
        resize(m, dim());
        std::fill(m.begin(), m.end(),
            std::numeric_limits<result_type>::max VSMC_MNE());

        return m;
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

    private:
    param_type param_;
    NormalDistribution<RealType> rnorm_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        result_type r;
        resize(r, dim());
        rnorm_(rng, dim(), r.data());
        mulchol(r.data());
        add(dim(), mean(), r.data(), r.data());

        return r;
    }

    void mulchol(float *r)
    {
        ::cblas_stpmv(::CblasRowMajor, ::CblasLower, ::CblasNoTrans,
            ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(dim()), chol(), r, 1);
    }

    void mulchol(double *r)
    {
        ::cblas_dtpmv(::CblasRowMajor, ::CblasLower, ::CblasNoTrans,
            ::CblasNonUnit, static_cast<VSMC_CBLAS_INT>(dim()), chol(), r, 1);
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
