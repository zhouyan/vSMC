//============================================================================
// vSMC/include/vsmc/rng/random_walk.hpp
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

#ifndef VSMC_RNG_RANDOM_WALK_HPP
#define VSMC_RNG_RANDOM_WALK_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>
#include <vsmc/rng/normal_mv_distribution.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_FIXED_DIM(Dim, Name)               \
    VSMC_STATIC_ASSERT((Dim != Dynamic),                                      \
        "**" #Name "** OBJECT DECLARED WITH DYNAMIC DIMENSION")

#define VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_DYNAMIC_DIM(Dim, Name)             \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                      \
        "**" #Name "** OBJECT DECLARED WITH FIXED DIMENSION")

#define VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PROPOSAL_PARAM(flag, Name)        \
    VSMC_RUNTIME_ASSERT(                                                      \
        (flag), "**" #Name "Proposal** CONSTRUCTED WITH INVALID PARAMETERS")

namespace vsmc
{

/// \brief Random walk MCMC update
/// \ingroup RandomWalk
template <typename RealType, std::size_t Dim>
class RandomWalk
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `Dim > 0`
    RandomWalk()
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_FIXED_DIM(Dim, RandomWalk);
    }

    /// \brief Only usable when `Dim == Dynamic`
    RandomWalk(std::size_t dim) : x_(dim), y_(dim)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_DYNAMIC_DIM(Dim, RandomWalk);
    }

    std::size_t dim() const { return x_.size(); }

    /// \brief One-step random walk update
    ///
    /// \param rng RNG engine
    /// \param x The current state value. It will be updated to the new value
    /// after the MCMC move.
    /// \param ltx If it is a non-null pointer, then it points to the value of
    /// the \f$\log\gamma(x)\f$. It will be updated to the new value if the
    /// MCMC move is accepted and left unchanged otherwise. If it is a null
    /// pointer, then it is ignored. Use this pointer to save
    /// \f$\log\gamma(x)\f$ between updates if it is expensive to calculate.
    /// \param log_target The log-target fucntion
    /// ~~~{.cpp}
    /// result_type log_target(std::size_t dim_x, const result_type *x );
    /// ~~~
    /// It accepts the lengths of the state vector and a pointer to the storage
    /// of the state value. It return the value of log-target function
    /// \f$\log\gamma(x)\f$.
    /// \param proposal The proposal function. It takes the form,
    /// ~~~{.cpp}
    /// result_type proposal(RNGType &rng, std::size_t dim,
    ///     const result_type *x, result_type *y);
    /// ~~~
    /// After the call, the function return the proposed value in `y` and
    /// return the value \f$\log(q(y, x) / q(x, y))\f$.
    ///
    /// \return Acceptance count
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, result_type *x, result_type *ltx,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        result_type q = proposal(rng, dim(), x, y_.data());
        result_type s = ltx == nullptr ? log_target(dim(), x) : *ltx;
        result_type t = log_target(dim(), y_.data());
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            std::copy(y_.begin(), y_.end(), x);
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        return 0;
    }

    /// \brief One-step random walk update of a block of elements within a
    /// vector state
    ///
    /// \details
    /// With this operator, it is assumed that the length of input/output state
    /// vector `x` is of length `m` instead of `dim_x()`. A sub-vector of
    /// length `dim_x()`, starting at index `idx` will be updated. The
    /// log-target function will be called with `m` as its first argument and a
    /// length `m` vector will be passed as its second argument as well.
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, std::size_t m, std::size_t idx,
        result_type *x, result_type *ltx, LogTargetType &&log_target,
        ProposalType &&proposal)
    {
        std::copy_n(x + idx, dim(), x_.begin());
        result_type q = proposal(rng, dim(), x_.data(), y_.data());
        result_type s = ltx == nullptr ? log_target(m, x) : *ltx;
        std::copy(y_.begin(), y_.end(), x + idx);
        result_type t = log_target(m, x);
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        std::copy(x_.begin(), x_.end(), x + idx);

        return 0;
    }

    /// \brief Multi-step random walk update
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, result_type *x,
        result_type *ltx, LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s = ltx == nullptr ? log_target(dim(), x) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, x, &s,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    /// \brief Multi-step random walk update of a block of element within a
    /// vector state
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, std::size_t m,
        std::size_t idx, result_type *x, result_type *ltx,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s = ltx == nullptr ? log_target(m, x) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, m, idx, x, &s,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    private:
    U01Distribution<RealType> runif_;
    internal::Array<RealType, Dim> x_;
    internal::Array<RealType, Dim> y_;
}; // class RandomWalk

/// \brief Random walk MCMC update with test function
/// \ingroup RandomWalk
template <typename RealType, std::size_t DimX, std::size_t DimG>
class RandomWalkG
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `DimX > 0` and `DimG > 0`
    RandomWalkG()
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_FIXED_DIM(DimX, RandomWalkG);
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_FIXED_DIM(DimG, RandomWalkG);
    }

    /// \brief Only usable when `DimX == Dynamic` and `DimG == Dynamic`
    RandomWalkG(std::size_t dim_x, std::size_t dim_g)
        : x_(dim_x), y_(dim_x), g_(dim_g)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_DYNAMIC_DIM(DimX, RandomWalkG);
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_DYNAMIC_DIM(DimG, RandomWalkG);
    }

    std::size_t dim_x() const { return x_.size(); }
    std::size_t dim_g() const { return g_.size(); }

    /// \brief One-step random walk update
    ///
    /// \param rng RNG engine
    /// \param x The current state value. It will be updated to the new value
    /// after the MCMC move.
    /// \param ltx If it is a non-null pointer, then it points to the value of
    /// the \f$\log\gamma(x)\f$. It will be updated to the new value if the
    /// MCMC move is accepted and left unchanged otherwise. If it is a null
    /// pointer, then it is ignored. Use this pointer to save
    /// \f$\log\gamma(x)\f$ between updates if it is expensive to calculate.
    /// \param g If it is a non-null pointer, then it is used to save the value
    /// of the test function \f$g(x)\f$ if the MCMC move is accepted and left
    /// unchanged otherwise. If it is a null pointer, then it is ignored.
    /// \param log_target The log-target fucntion
    /// ~~~{.cpp}
    /// result_type log_target(std::size_t dim_x, std::size_t dim_g,
    ///     const result_type *x, result_type *g);
    /// ~~~
    /// It accepts the lengths of the state vector and test function value
    /// vector, and pointers to the storage of the state value and test
    /// function value. It return the value of log-target function
    /// \f$\log\gamma(x)\f$. Note that this fucntion shall be able to handle
    /// its argument `g` as a null pointer.
    /// \param proposal The proposal function. It takes the form,
    /// ~~~{.cpp}
    /// result_type proposal(RNGType &rng, std::size_t dim,
    ///     const result_type *x, result_type *y);
    /// ~~~
    /// After the call, the function return the proposed value in `y` and
    /// return the value \f$\log(q(y, x) / q(x, y))\f$.
    ///
    /// \return Acceptance count
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, result_type *x, result_type *ltx,
        result_type *g, LogTargetType &&log_target, ProposalType &&proposal)
    {
        result_type q = proposal(rng, dim_x(), x, y_.data());
        result_type s =
            ltx == nullptr ? log_target(dim_x(), dim_g(), x, nullptr) : *ltx;
        result_type t = log_target(
            dim_x(), dim_g(), y_.data(), (g == nullptr ? nullptr : g_.data()));
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            std::copy(y_.begin(), y_.end(), x);
            if (ltx != nullptr)
                *ltx = t;
            if (g != nullptr)
                std::copy(g_.begin(), g_.end(), g);
            return 1;
        }
        return 0;
    }

    /// \brief One-step random walk update of a block of elements within a
    /// vector state
    ///
    /// \details
    /// With this operator, it is assumed that the length of input/output state
    /// vector `x` is of length `m` instead of `dim_x()`. A sub-vector of
    /// length `dim_x()`, starting at index `idx` will be updated. The
    /// log-target function will be called with `m` as its first argument and a
    /// length `m` vector will be passed as its third argument as well.
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, std::size_t m, std::size_t idx,
        result_type *x, result_type *ltx, result_type *g,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::copy_n(x + idx, dim_x(), x_.begin());
        result_type q = proposal(rng, dim_x(), x_.data(), y_.data());
        result_type s =
            ltx == nullptr ? log_target(m, dim_g(), x, nullptr) : *ltx;
        std::copy(y_.begin(), y_.end(), x + idx);
        result_type t =
            log_target(m, dim_g(), x, (g == nullptr ? nullptr : g_.data()));
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            if (ltx != nullptr)
                *ltx = t;
            if (g != nullptr)
                std::copy(g_.begin(), g_.end(), g);
            return 1;
        }
        std::copy(x_.begin(), x_.end(), x + idx);

        return 0;
    }

    /// \brief Multi-step random walk update
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, result_type *x,
        result_type *ltx, result_type *g, LogTargetType &&log_target,
        ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s =
            ltx == nullptr ? log_target(dim_x(), dim_g(), x, nullptr) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, x, &s, g,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    /// \brief Multi-step random walk update of a block of element within a
    /// vector state
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, std::size_t m,
        std::size_t idx, result_type *x, result_type *ltx, result_type *g,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s =
            ltx == nullptr ? log_target(m, dim_g(), x, nullptr) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, m, idx, x, &s, g,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    private:
    U01Distribution<RealType> runif_;
    internal::Array<RealType, DimX> x_;
    internal::Array<RealType, DimX> y_;
    internal::Array<RealType, DimG> g_;
}; // class RandomWalkG

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
RealType normal_proposal_q(RealType x, RealType &y, RealType z)
{
    y = x + z;

    return 0;
}

template <typename RealType>
RealType normal_proposal_qa(RealType x, RealType &y, RealType z, RealType a)
{
    y = a + (x - a) * std::exp(z);

    return z;
}

template <typename RealType>
RealType normal_proposal_qb(RealType x, RealType &y, RealType z, RealType b)
{
    y = b - (b - x) * std::exp(z);

    return z;
}

template <typename RealType>
RealType normal_proposal_qab(
    RealType x, RealType &y, RealType z, RealType a, RealType b)
{
    RealType r = std::exp(z) * (x - a) / (b - x);
    y = (a + b * r) / (1 + r);

    return std::log((y - a) / (x - a) * (b - y) / (b - x));
}

} // namespace vsmc::internal

/// \brief Normal random walk proposal
/// \ingroup RandomWalk
template <typename RealType>
class NormalProposal
{
    public:
    using result_type = RealType;

    /// \brief Construct a Normal random walk proposal
    ///
    /// \param stddev The standard deviation (scale) of the proposal
    /// \param a The lower bound of the support of the target distribution
    /// \param b The upper bound of the support of the target distribution
    explicit NormalProposal(result_type stddev = 1,
        result_type a = -std::numeric_limits<result_type>::infinity(),
        result_type b = std::numeric_limits<result_type>::infinity())
        : rnorm_(0, stddev), a_(a), b_(b), flag_(0)
    {
        unsigned lower = std::isfinite(a) ? 1 : 0;
        unsigned upper = std::isfinite(b) ? 1 : 0;
        flag_ = (lower << 1) + upper;
        VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PROPOSAL_PARAM(
            internal::normal_proposal_check_param(a, b), Normal);
    }

    result_type a() const { return a_; }
    result_type b() const { return b_; }

    /// \brief Propose new value `y` and return \f$\log(q(y, x) / q(x, y))\f$.
    template <typename RNGType>
    result_type operator()(
        RNGType &rng, std::size_t, const result_type *x, result_type *y)
    {
        result_type z = rnorm_(rng);
        switch (flag_) {
            case 0: return internal::normal_proposal_q(*x, *y, z);
            case 1: return internal::normal_proposal_qb(*x, *y, z, b_);
            case 2: return internal::normal_proposal_qa(*x, *y, z, a_);
            case 3: return internal::normal_proposal_qab(*x, *y, z, a_, b_);
            default: return 0;
        }
    }

    private:
    NormalDistribution<RealType> rnorm_;
    result_type a_;
    result_type b_;
    unsigned flag_;
}; // class NormalProposal

/// \brief Multivariate Normal random walk proposal
/// \ingroup RandomWalk
template <typename RealType, std::size_t Dim>
class NormalMVProposal
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `Dim > 0`
    ///
    /// \param chol The lower triangular elements of the Cholesky decomposition
    /// of the covaraince matrix, packed row by row. If it is a nullpointer,
    /// then the covariance is the identicy matrix \f$I\f$
    /// \param a The lower bound of the support of the target distribuiton. It
    /// is assumed that the support is \f$\prod_{p=1}^d E_p \subset
    /// \mathbb{R}^d\f$ where \f$E_p \subset \mathbb{R}\f$.
    /// \param b The upper bound of the support of the target distribution.
    ///
    /// \details
    /// If the geometry of the support is more complex than above, then one may
    /// find a superset of the support that takes the required form, and reject
    /// proposals that lay outside the support manually.
    explicit NormalMVProposal(const result_type *chol = nullptr,
        const result_type *a = nullptr, const result_type *b = nullptr)
        : rnorm_(nullptr, chol)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_FIXED_DIM(Dim, NormalMVProposal);
        init(Dim, a, b);
    }

    explicit NormalMVProposal(std::size_t dim,
        const result_type *chol = nullptr, const result_type *a = nullptr,
        const result_type *b = nullptr)
        : rnorm_(dim, nullptr, chol), a_(dim), b_(dim), z_(dim), flag_(dim)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_DYNAMIC_DIM(Dim, NormalMVProposal);
        init(dim, a, b);
    }

    std::size_t dim() const { return rnorm_.dim(); }
    const result_type *a() const { return a_.data(); }
    const result_type *b() const { return b_.data(); }

    template <typename RNGType>
    result_type operator()(
        RNGType &rng, std::size_t, const result_type *x, result_type *y)
    {
        rnorm_(rng, z_.data());
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
    NormalMVDistribution<RealType, Dim> rnorm_;
    internal::Array<RealType, Dim> a_;
    internal::Array<RealType, Dim> b_;
    internal::Array<RealType, Dim> z_;
    internal::Array<unsigned, Dim> flag_;

    void init(std::size_t dim, const result_type *a, const result_type *b)
    {
        if (a == nullptr) {
            std::fill(a_.begin(), a_.end(),
                -std::numeric_limits<result_type>::infinity());
        } else {
            std::copy_n(a, dim, a_.begin());
        }

        if (b == nullptr) {
            std::fill(b_.begin(), b_.end(),
                std::numeric_limits<result_type>::infinity());
        } else {
            std::copy_n(b, dim, b_.begin());
        }

        for (std::size_t i = 0; i != dim; ++i) {
            unsigned lower = std::isfinite(a_[i]) ? 1 : 0;
            unsigned upper = std::isfinite(b_[i]) ? 1 : 0;
            flag_[i] = (lower << 1) + upper;
        }

        VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PROPOSAL_PARAM(
            internal::normal_mv_proposal_check_param(
                dim, a_.data(), b_.data()),
            NormalMV);
    }
}; // class NormalMVProposal

} // namespace vsmc

#endif // VSMC_RNG_RANDOM_WALK_HPP
