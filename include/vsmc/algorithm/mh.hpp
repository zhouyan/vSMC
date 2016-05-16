//============================================================================
// vSMC/include/vsmc/algorithm/mh.hpp
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

#ifndef VSMC_ALGORITHM_MH_HPP
#define VSMC_ALGORITHM_MH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>
#include <vsmc/rng/normal_mv_distribution.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

/// \brief Metropolis-Hastings MCMC update
/// \ingroup MH
template <typename ResultType = double, std::size_t Dim = Dynamic>
class MH
{
    public:
    using result_type = ResultType;

    /// \brief Only usable when `Dim != Dynamic`
    MH()
    {
        static_assert(
            Dim != Dynamic, "**MH** object declared with dynamic dimension");
    }

    /// \brief Only usable when `Dim == Dynamic`
    MH(std::size_t dim) : x_(dim), y_(dim)
    {
        static_assert(
            Dim == Dynamic, "**MH** object declared with fixed dimension");
    }

    std::size_t dim() const { return x_.size(); }

    /// \brief One-step Metropolis-Hastings update
    ///
    /// \param rng RNG engine
    /// \param x The current state value. It will be updated to the new value
    /// after the Metropolis-Hastings move if it is accepted, and left
    /// unchanged otherwise.
    /// \param ltx If it is a non-null pointer, then it points to the value of
    /// the \f$\log\gamma(x)\f$. It will be updated to the new value if the
    /// Metropolis-Hastings move is accepted, and left unchanged otherwise. If
    /// it is a null pointer, then it is ignored. Use this pointer to save
    /// \f$\log\gamma(x)\f$ between updates if it is expensive to calculate.
    /// \param log_target The log-target fucntion
    /// ~~~{.cpp}
    /// ReturnType log_target(const result_type *x);
    /// ~~~
    /// where `ReturnType` needs to be convertible to `double`. It accepts a
    /// pointer to the states and returns the value of log-target function
    /// \f$\log\gamma(x)\f$.
    /// \param proposal The proposal function
    /// ~~~{.cpp}
    /// ReturnType proposal(
    ///     RNGType &rng, const result_type *x, result_type *y);
    /// ~~~
    /// where `ReturnType` needs to be convertible to `double`. It accepts a
    /// pointer to the states and returns the value of the proposal new states
    /// in `y` and the value of \f$\log(q(y, x) / q(x, y))\f$.
    ///
    /// \return Acceptance count
    template <typename RNGType, typename LogTarget, typename Proposal>
    std::size_t operator()(RNGType &rng, result_type *x, double *ltx,
        LogTarget &&log_target, Proposal &&proposal)
    {
        U01Distribution<double> u01;
        double s = ltx == nullptr ? static_cast<double>(log_target(x)) : *ltx;
        double q = static_cast<double>(proposal(rng, x, y_.data()));
        double t = static_cast<double>(log_target(y_.data()));
        double p = t - s + q;
        double u = std::log(u01(rng));

        if (u < p) {
            std::copy(y_.begin(), y_.end(), x);
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        return 0;
    }

    /// \brief Multi-step Metropolis-Hastings update
    template <typename RNGType, typename LogTarget, typename Proposal>
    std::size_t operator()(std::size_t n, RNGType &rng, result_type *x,
        double *ltx, LogTarget &&log_target, Proposal &&proposal)
    {
        if (n == 0)
            return 0;

        if (n == 1) {
            return operator()(rng, x, ltx, std::forward<LogTarget>(log_target),
                std::forward<Proposal>(proposal));
        }

        U01Distribution<double> u01;
        std::copy_n(x, dim(), x_.data());
        std::size_t accept = 0;
        double s = ltx == nullptr ? static_cast<double>(log_target(x)) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            double q =
                static_cast<double>(proposal(rng, x_.data(), y_.data()));
            double t = static_cast<double>(log_target(y_.data()));
            double p = t - s + q;
            double u = std::log(u01(rng));
            if (u < p) {
                ++accept;
                s = t;
                x_.swap(y_);
            }
        }
        if (accept != 0) {
            std::copy(x_.begin(), x_.end(), x);
            if (ltx != nullptr)
                *ltx = s;
        }

        return accept;
    }

    private:
    internal::StaticVector<ResultType, Dim> x_;
    internal::StaticVector<ResultType, Dim> y_;
}; // class MH

namespace internal
{

template <typename RealType>
inline bool normal_proposal_check_param(RealType a, RealType b)
{
    return a < b;
}

template <typename RealType>
inline bool normal_mv_proposal_check_param(
    std::size_t dim, const RealType *a, const RealType *b)
{
    for (std::size_t i = 0; i != dim; ++i)
        if (a[i] >= b[i])
            return false;
    return true;
}

template <typename RealType>
inline RealType normal_proposal_q(RealType x, RealType &y, RealType z)
{
    y = x + z;

    return 0;
}

template <typename RealType>
inline RealType normal_proposal_qa(
    RealType x, RealType &y, RealType z, RealType a)
{
    y = a + (x - a) * std::exp(z);

    return z;
}

template <typename RealType>
inline RealType normal_proposal_qb(
    RealType x, RealType &y, RealType z, RealType b)
{
    y = b - (b - x) * std::exp(z);

    return z;
}

template <typename RealType>
inline RealType normal_proposal_qab(
    RealType x, RealType &y, RealType z, RealType a, RealType b)
{
    RealType r = std::exp(z) * (x - a) / (b - x);
    y = (a + b * r) / (1 + r);

    return std::log(((y - a) / (x - a)) * ((b - y) / (b - x)));
}

} // namespace vsmc::internal

/// \brief Normal proposal
/// \ingroup MH
template <typename RealType = double>
class NormalProposal
{
    public:
    using result_type = RealType;

    /// \brief Construct a Normal proposal
    ///
    /// \param stddev The standard deviation (scale) of the proposal
    /// \param a The lower bound of the support of the target distribution
    /// \param b The upper bound of the support of the target distribution
    NormalProposal(result_type stddev, result_type a, result_type b)
        : normal_(const_zero<result_type>(), stddev), a_(a), b_(b), flag_(0)
    {
        runtime_assert(internal::normal_proposal_check_param(a, b),
            "**NormalProposal** constructed with invalid arguments");

        unsigned lower = std::isfinite(a) ? 1 : 0;
        unsigned upper = std::isfinite(b) ? 1 : 0;
        flag_ = (lower << 1) + upper;
    }

    std::size_t dim() const { return 1; }

    /// \brief Propose new value `y` and return \f$\log(q(y, x) / q(x, y))\f$.
    template <typename RNGType>
    result_type operator()(RNGType &rng, const result_type *x, result_type *y)
    {
        result_type z = normal_(rng);
        switch (flag_) {
            case 0: return internal::normal_proposal_q(*x, *y, z);
            case 1: return internal::normal_proposal_qb(*x, *y, z, b_);
            case 2: return internal::normal_proposal_qa(*x, *y, z, a_);
            case 3: return internal::normal_proposal_qab(*x, *y, z, a_, b_);
            default: return 0;
        }
    }

    private:
    NormalDistribution<RealType> normal_;
    result_type a_;
    result_type b_;
    unsigned flag_;
}; // class NormalProposal

/// \brief Multivariate Normal proposal
/// \ingroup MH
template <typename RealType = double, std::size_t Dim = Dynamic>
class NormalMVProposal
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(result_type chol, result_type a, result_type b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(result_type chol, result_type a, const result_type *b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(result_type chol, const result_type *a, result_type b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(
        result_type chol, const result_type *a, const result_type *b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(const result_type *chol, result_type a, result_type b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(
        const result_type *chol, result_type a, const result_type *b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(
        const result_type *chol, const result_type *a, result_type b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim != Dynamic`
    NormalMVProposal(
        const result_type *chol, const result_type *a, const result_type *b)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim != Dynamic,
            "**NormalMVProposal** object declared with dynamic dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(
        std::size_t dim, result_type chol, result_type a, result_type b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(
        std::size_t dim, result_type chol, result_type a, const result_type *b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(
        std::size_t dim, result_type chol, const result_type *a, result_type b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(std::size_t dim, result_type chol, const result_type *a,
        const result_type *b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(
        std::size_t dim, const result_type *chol, result_type a, result_type b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(std::size_t dim, const result_type *chol, result_type a,
        const result_type *b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(std::size_t dim, const result_type *chol,
        const result_type *a, result_type b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVProposal(std::size_t dim, const result_type *chol,
        const result_type *a, const result_type *b)
        : normal_mv_(dim, const_zero<result_type>(), chol)
        , a_(dim)
        , b_(dim)
        , z_(dim)
        , flag_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVProposal** object declared with fixed dimension");
        init_a(a);
        init_b(b);
        init_flag();
    }

    std::size_t dim() const { return normal_mv_.dim(); }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const result_type *x, result_type *y)
    {
        normal_mv_(rng, z_.data());
        result_type q = 0;
        for (std::size_t i = 0; i != dim(); ++i) {
            switch (flag_[i]) {
                case 0:
                    q += internal::normal_proposal_q(x[i], y[i], z_[i]);
                    break;
                case 1:
                    q +=
                        internal::normal_proposal_qb(x[i], y[i], z_[i], b_[i]);
                    break;
                case 2:
                    q +=
                        internal::normal_proposal_qa(x[i], y[i], z_[i], a_[i]);
                    break;
                case 3:
                    q += internal::normal_proposal_qab(
                        x[i], y[i], z_[i], a_[i], b_[i]);
                    break;
                default: break;
            }
        }

        return q;
    }

    private:
    NormalMVDistribution<RealType, Dim> normal_mv_;
    internal::StaticVector<RealType, Dim> a_;
    internal::StaticVector<RealType, Dim> b_;
    internal::StaticVector<RealType, Dim> z_;
    internal::StaticVector<unsigned, Dim> flag_;

    void init_a(result_type a) { std::fill(a_.begin(), a_.end(), a); }

    void init_a(const result_type *a) { std::copy_n(a, dim(), a_.begin()); }

    void init_b(result_type b) { std::fill(b_.begin(), b_.end(), b); }

    void init_b(const result_type *b) { std::copy_n(b, dim(), b_.begin()); }

    void init_flag()
    {
        runtime_assert(internal::normal_mv_proposal_check_param(
                           dim(), a_.data(), b_.data()),
            "**NormalMVProposal** constructed with invalid arguments");

        for (std::size_t i = 0; i != dim(); ++i) {
            unsigned lower = std::isfinite(a_[i]) ? 1 : 0;
            unsigned upper = std::isfinite(b_[i]) ? 1 : 0;
            flag_[i] = (lower << 1) + upper;
        }
    }
}; // class NormalMVProposal

/// \brief Multivariate Normal proposal on logit scale
/// \ingroup MH
template <typename RealType = double, std::size_t Dim = Dynamic>
class NormalMVLogitProposal
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `Dim > 1`
    NormalMVLogitProposal(result_type chol)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim > 1, "**NormalMVLogitProposal** object declared "
                               "with dimension less than 2");
    }

    /// \brief Only usable when `Dim > 1`
    NormalMVLogitProposal(const result_type *chol)
        : normal_mv_(const_zero<result_type>(), chol)
    {
        static_assert(Dim > 1, "**NormalMVLogitProposal** object declared "
                               "with dimension less than 2");
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVLogitProposal(std::size_t dim, result_type chol)
        : normal_mv_(dim, const_zero<result_type>(), chol), z_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVLogitProposal** object declared with fixed dimension");

        runtime_assert(dim > 1, "**NormalMVLogitProposal** constructed with "
                                "dimension less than 2");
    }

    /// \brief Only usable when `Dim == Dynamic`
    NormalMVLogitProposal(std::size_t dim, const result_type *chol)
        : normal_mv_(dim, const_zero<result_type>(), chol), z_(dim)
    {
        static_assert(Dim == Dynamic,
            "**NormalMVLogitProposal** object declared with fixed dimension");

        runtime_assert(dim > 1, "**NormalMVLogitProposal** constructed with "
                                "dimension less than 2");
    }

    std::size_t dim() const { return normal_mv_.dim(); }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const result_type *x, result_type *y)
    {
        const std::size_t d = dim();
        normal_mv_(rng, z_.data());
        exp(d - 1, z_.data(), y);
        mul(d - 1, x, y, y);
        mul(d - 1, 1 / x[d - 1], y, y);
        y[d - 1] = 1;
        mul(d, 1 / std::accumulate(y, y + d, 0.0), y, y);

        return q(d, y) - q(d, x);
    }

    private:
    NormalMVDistribution<RealType, Dim> normal_mv_;
    internal::StaticVector<RealType, Dim> z_;

    result_type q(std::size_t d, const result_type *x)
    {
        result_type slw = 1;
        result_type sllw = 0;
        const result_type w = x[d - 1];
        for (std::size_t i = 0; i != d - 1; ++i) {
            result_type v = x[i] / w;
            slw += v;
            sllw += std::log(v);
        }
        sllw -= d * std::log(slw);

        return sllw;
    }
}; // class NormalMVLogitProposal

} // namespace vsmc

#endif // VSMC_ALGORITHM_MH_HPP
