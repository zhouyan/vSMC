//============================================================================
// vSMC/include/vsmc/rng/dirichlet_distribution.hpp
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

#ifndef VSMC_RNG_DIRICHLET_DISTRIBUTION_HPP
#define VSMC_RNG_DIRICHLET_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/gamma_distribution.hpp>

namespace vsmc
{

template <typename RealType, std::size_t Dim>
class DirichletDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = DirichletDistribution<RealType, Dim>;

    class param_type
    {
        static_assert(std::is_floating_point<RealType>::value,
            "**DirichletDistribution::param_type** USED WITH RealType OTHER "
            "THAN FLOATING POINT TYPES");

        public:
        using result_type = RealType;
        using distribution_type = DirichletDistribution<RealType, Dim>;

        explicit param_type(result_type alpha) : is_scalar_(true)
        {
            static_assert(Dim != Dynamic, "**DirichletDistribution::param_"
                                          "type** OBJECT DECLARED WITH "
                                          "DYNAMIC DIMENSION");
            init(alpha);
        }

        explicit param_type(const result_type *alpha) : is_scalar_(false)
        {
            static_assert(Dim != Dynamic, "**DirichletDistribution::param_"
                                          "type** OBJECT DECLARED WITH "
                                          "DYNAMIC DIMENSION");
            init(alpha);
        }

        explicit param_type(std::size_t dim, result_type alpha)
            : alpha_(dim), is_scalar_(false)
        {
            static_assert(Dim == Dynamic, "**DirichletDistribution::param_"
                                          "type** OBJECT DELCARED WITH FIXED "
                                          "DIMENSION");
            init(alpha);
        }

        explicit param_type(std::size_t dim, const result_type *alpha)
            : alpha_(dim), is_scalar_(false)
        {
            static_assert(Dim == Dynamic, "**DirichletDistribution::param_"
                                          "type** OBJECT DELCARED WITH FIXED "
                                          "DIMENSION");
            init(alpha);
        }

        std::size_t dim() const { return alpha_.size(); }

        const result_type *alpha() const { return alpha_.data(); }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.alpha_ != param2.alpha_)
                return false;
            if (param1.is_scalar_ != param2.is_scalar_)
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
            if (!os)
                return os;

            os << param.alpha_ << ' ';
            os << param.is_scalar_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, const param_type &param)
        {
            if (!is)
                return is;

            internal::StaticVector<result_type, Dim> alpha;
            bool is_scalar;

            is >> std::ws >> alpha;
            is >> std::ws >> is_scalar;

            if (is) {
                param.alpha_ = std::move(alpha);
                param.is_scalar_ = is_scalar;
            } else {
                is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private:
        internal::StaticVector<result_type, Dim> alpha_;
        bool is_scalar_;

        friend distribution_type;

        void init(double alpha)
        {
            std::fill(alpha_.begin(), alpha_.end(), alpha);
        }

        void init(const double *alpha)
        {
            std::copy_n(alpha, alpha_.size(), alpha_.begin());
        }
    }; // class param_type

    /// \brief Only usable when `Dim > 0`
    explicit DirichletDistribution(double alpha) : param_(alpha) {}

    /// \brief Only usable when `Dim > 0`
    explicit DirichletDistribution(const double *alpha) : param_(alpha) {}

    /// \brief Only usable when `Dim == Dynamic`
    explicit DirichletDistribution(std::size_t dim, double alpha)
        : param_(dim, alpha)
    {
        reset();
    }

    /// \brief Only usable when `Dim == Dynamic`
    DirichletDistribution(std::size_t dim, const double *alpha)
        : param_(dim, alpha)
    {
        reset();
    }

    explicit DirichletDistribution(const param_type &param) : param_(param)
    {
        reset();
    }

    explicit DirichletDistribution(param_type &&param)
        : param_(std::move(param))
    {
        reset();
    }

    void min(result_type *x) const { std::fill_n(x, dim(), 0); }

    void max(result_type *x) const { std::fill_n(x, dim(), 1); }

    void reset() {}

    std::size_t dim() const { return param_.dim(); }

    const result_type *alpha() const { return param_.alpha(); }

    const param_type &param() const { return param_; }

    void param(const param_type &param)
    {
        param_ = param;
        reset();
    }

    void param(param_type &&param)
    {
        param_ = std::move(param);
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
        if (param.is_scalar_)
            dirichlet_distribution(rng, n, r, param.dim(), param.alpha()[0]);
        else
            dirichlet_distribution(rng, n, r, param.dim(), param.alpha());
    }

    friend bool operator==(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        if (dist1.param_ != dist2.param_)
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
        if (!os)
            return os;

        os << dist.param_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)
    {
        if (!is)
            return is;

        param_type param;
        is >> std::ws >> param;
        if (is)
            dist.param_ = std::move(param);

        return is;
    }

    private:
    param_type param_;

    template <typename RNGType>
    void generate(RNGType &rng, result_type *r, const param_type &param)
    {
        const std::size_t n = param.dim();
        if (param.is_scalar_) {
            GammaDistribution<result_type> gamma(param.alpha()[0]);
            gamma(rng, n, r);
        } else {
            for (std::size_t i = 0; i != n; ++i) {
                GammaDistribution<result_type> gamma(param.alpha()[i]);
                r[i] = gamma(rng);
            }
        }
        result_type s = std::accumulate(r, r + n, static_cast<result_type>(0));
        mul(n, 1 / s, r, r);
    }
}; // class DirichletDistribution

namespace internal
{

template <typename RealType>
inline void dirichlet_distribution_avg(
    std::size_t n, std::size_t dim, RealType *r)
{
    for (std::size_t i = 0; i != n; ++i, r += dim) {
        RealType s = std::accumulate(r, r + dim, static_cast<RealType>(0));
        mul(dim, 1 / s, r, r);
    }
}

} // namespace internal

/// \brief Generating Dirichlet random varaites
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void dirichlet_distribution(RNGType &rng, std::size_t n, RealType *r,
    std::size_t dim, const RealType alpha)
{
    gamma_distribution(rng, n * dim, r, alpha, static_cast<RealType>(1));
    internal::dirichlet_distribution_avg(n, dim, r);
}

/// \brief Generating Dirichlet random varaites
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void dirichlet_distribution(RNGType &rng, std::size_t n, RealType *r,
    std::size_t dim, const RealType *alpha)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**dirichlet_distribution** USED WITH RealType OTHER THAN FLOATING "
        "POINT TYPES");

    if (n * dim == 0)
        return;

    Vector<RealType> s(n);
    for (std::size_t i = 0; i != dim; ++i) {
        gamma_distribution(
            rng, n, s.data(), alpha[i], static_cast<RealType>(1));
        RealType *t = r + i;
        for (std::size_t j = 0; j != n; ++j, t += dim)
            *t = s[i];
    }
    internal::dirichlet_distribution_avg(n, dim, r);
}

template <typename RealType, std::size_t Dim, typename RNGType>
inline void dirichlet_distribution(RNGType &rng, std::size_t n, RealType *r,
    const typename DirichletDistribution<RealType, Dim>::param_type &param)
{
    DirichletDistribution<RealType, Dim> dist(param);
    dist(rng, n, r);
}

template <typename RealType, std::size_t Dim, typename RNGType>
inline void rand(RNGType &rng, DirichletDistribution<RealType, Dim> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_DIRICHLET_DISTRIBUTION_HPP
