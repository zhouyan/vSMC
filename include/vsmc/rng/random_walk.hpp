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

#define VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_FIXED_DIM(Dim, Name)            \
    VSMC_STATIC_ASSERT((Dim != Dynamic),                                      \
        "**" #Name "** OBJECT DECLARED WITH DYNAMIC DIMENSION")

#define VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_DYNAMIC_DIM(Dim, Name)          \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                      \
        "**" #Name "** OBJECT DECLARED WITH FIXED DIMENSION")

#define VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PROPOSAL_PARAM(flag, Name)        \
    VSMC_RUNTIME_ASSERT(                                                      \
        (flag), "**" #Name "Proposal** CONSTRUCTED WITH INVALID PARAMETERS")

namespace vsmc
{

/// \brief Random walk MCMC
/// \ingroup RandomWalk
template <typename RealType>
class RandomWalk
{
    public:
    using result_type = RealType;

    /// \brief One-step random walk update
    ///
    /// \param rng RNG engine
    /// \param x The current state value. It will be updated to the new value
    /// after the MCMC move.
    /// \param ltx If it is a non-null pointer, then it points to the value of
    /// the \f$\log\gamma(x)\f$. It will be updated to the updated value.  If
    /// it is a null pointer, then it is ignored. Use this pointer to save
    /// \f$\log\gamma(x)\f$ between updates if it is expensive to calculate.
    /// \param log_target The log-target fucntion
    /// ~~~{.cpp}
    /// result_type log_target(result_type x);
    /// ~~~
    /// and return the value of \f$\log\gamma(x)\f$.
    /// \param proposal The proposal function. It takes the form,
    /// ~~~{.cpp}
    /// result_type proposal(RNGType &rng, result_type x, result_type &y);
    /// ~~~
    /// After the call, the function return the proposed value in `y` and
    /// return the value \f$\log(q(y, x) / q(x, y))\f$.
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, result_type &x, result_type *ltx,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        result_type r;
        result_type q = proposal(rng, x, r);
        result_type s = ltx == nullptr ? log_target(x) : *ltx;
        result_type t = log_target(r);
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            x = r;
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        return 0;
    }

    /// \brief One-step random walk update of one element within a vector state
    ///
    /// \param rng RNG engine
    /// \param m The length of state vector x
    /// \param idx The index of the element to be updated
    /// \param x The current state value
    /// \param ltx The current value of \f$\log\gamma(x)\f$
    /// \param log_target The log-target fucntion
    /// \param proposal The proposal function
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, std::size_t m, std::size_t idx,
        result_type *x, result_type *ltx, LogTargetType &&log_target,
        ProposalType &&proposal)
    {
        result_type xi = x[idx];
        result_type r;
        result_type q = proposal(rng, x[idx], r);
        result_type s = ltx == nullptr ? log_target(m, x) : *ltx;
        x[idx] = r;
        result_type t = log_target(m, x);
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        x[idx] = xi;

        return 0;
    }

    /// \brief Multi-step random walk update
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, result_type &x,
        result_type *ltx, LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s = ltx == nullptr ? log_target(x) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, x, &s,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    /// \brief Multi-step random walk update of one element within a vector
    /// state
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, std::size_t m,
        std::size_t index, result_type *x, result_type *ltx,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        result_type s = ltx == nullptr ? log_target(m, x) : *ltx;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, m, index, x, &s,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }
        if (ltx != nullptr)
            *ltx = s;

        return acc;
    }

    private:
    U01Distribution<RealType> runif_;
}; // class RandomWalk

/// \brief Multivariate random walk MCMC
/// \ingroup RandomWalk
template <typename RealType, std::size_t Dim>
class RandomWalkMV
{
    public:
    using result_type = RealType;

    /// \brief Only usable when `Dim > 0`
    RandomWalkMV()
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_FIXED_DIM(Dim, RandomWalkMV);
    }

    /// \brief Only usable when `Dim == Dynamic`
    RandomWalkMV(std::size_t dim) : r_(dim)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_DYNAMIC_DIM(Dim, RandomWalkMV);
    }

    std::size_t dim() const { return r_.size(); }

    /// \brief One-step random walk update
    ///
    /// \param rng RNG engine
    /// \param x The current state value. It will be updated to the new value
    /// after the MCMC move.
    /// \param ltx If it is a non-null pointer, then it points to the value of
    /// the \f$\log\gamma(x)\f$. It will be updated to the updated value.  If
    /// it is a null pointer, then it is ignored. Use this pointer to save
    /// \f$\log\gamma(x)\f$ between updates if it is expensive to calculate.
    /// \param log_target The log-target fucntion
    /// ~~~{.cpp}
    /// result_type log_target(std::size_t dim, const result_type *x);
    /// ~~~
    /// and return the value of \f$\log\gamma(x)\f$.
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
        result_type q = proposal(rng, dim(), x, r_.data());
        result_type s = ltx == nullptr ? log_target(dim(), x) : *ltx;
        result_type t = log_target(dim(), r_.data());
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            std::copy(r_.begin(), r_.end(), x);
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        return 0;
    }

    /// \brief One-step random walk update of a block of element within a
    /// vector state
    ///
    /// \param rng RNG engine
    /// \param m The length of state vector x
    /// \param idx The index of the first element within the block. The block
    /// length is `dim()`, the dimension of this RandomWalMV object
    /// \param x The current state value
    /// \param ltx The current value of \f$\log\gamma(x)\f$
    /// \param log_target The log-target fucntion
    /// \param proposal The proposal function
    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, std::size_t m, std::size_t idx,
        result_type *x, result_type *ltx, LogTargetType &&log_target,
        ProposalType &&proposal)
    {
        std::copy_n(x + idx, dim(), xi_.begin());
        result_type q = proposal(rng, dim(), xi_.data(), r_.data());
        result_type s = ltx == nullptr ? log_target(m, x) : *ltx;
        std::copy(r_.begin(), r_.end(), x + idx);
        result_type t = log_target(m, x);
        result_type p = t - s + q;
        result_type u = std::log(runif_(rng));
        if (u < p) {
            if (ltx != nullptr)
                *ltx = t;
            return 1;
        }
        std::copy(xi_.begin(), xi_.end(), x + idx);

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
    using vector_type = typename std::conditional<Dim == Dynamic,
        Vector<RealType>, std::array<RealType, Dim>>::type;

    U01Distribution<RealType> runif_;
    vector_type r_;
    vector_type xi_;
}; // class RandomwWalkMV

namespace internal
{

template <typename RealType>
inline bool proposal_normal_check_param(RealType a, RealType b)
{
    return a < b;
}

template <typename RealType>
inline bool proposal_normal_mv_check_param(
    std::size_t dim, const RealType *a, const RealType *b)
{
    for (std::size_t i = 0; i != dim; ++i)
        if (a[i] >= b[i])
            return false;
    return true;
}

template <typename RealType>
RealType proposal_normal_q(RealType x, RealType &y, RealType z)
{
    y = x + z;

    return 0;
}

template <typename RealType>
RealType proposal_normal_qa(RealType x, RealType &y, RealType z, RealType a)
{
    y = a + (x - a) * std::exp(z);

    return z;
}

template <typename RealType>
RealType proposal_normal_qb(RealType x, RealType &y, RealType z, RealType b)
{
    y = b - (b - x) * std::exp(z);

    return z;
}

template <typename RealType>
RealType proposal_normal_qab(
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
            internal::proposal_normal_check_param(a, b), Normal);
    }

    result_type a() const { return a_; }
    result_type b() const { return b_; }

    /// \brief Propose new value `y` and return \f$\log(q(y, x) / q(x, y))\f$.
    template <typename RNGType>
    result_type operator()(RNGType &rng, result_type x, result_type &y)
    {
        result_type z = rnorm_(rng);
        switch (flag_) {
            case 0: return internal::proposal_normal_q(x, y, z);
            case 1: return internal::proposal_normal_qb(x, y, z, b_);
            case 2: return internal::proposal_normal_qa(x, y, z, a_);
            case 3: return internal::proposal_normal_qab(x, y, z, a_, b_);
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
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_FIXED_DIM(Dim, NormalMVProposal);
        init(Dim, a, b);
    }

    explicit NormalMVProposal(std::size_t dim,
        const result_type *chol = nullptr, const result_type *a = nullptr,
        const result_type *b = nullptr)
        : rnorm_(dim, nullptr, chol), a_(dim), b_(dim), z_(dim), flag_(dim)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_DYNAMIC_DIM(
            Dim, NormalMVProposal);
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
                    q += internal::proposal_normal_q(x[i], y[i], z_[i]);
                    break;
                case 1:
                    q +=
                        internal::proposal_normal_qb(x[i], y[i], z_[i], b_[i]);
                    break;
                case 2:
                    q +=
                        internal::proposal_normal_qa(x[i], y[i], z_[i], a_[i]);
                    break;
                case 3:
                    q += internal::proposal_normal_qab(
                        x[i], y[i], z_[i], a_[i], b_[i]);
                    break;
                default: break;
            }
        }

        return q;
    }

    private:
    template <typename T>
    using vector_type = typename std::conditional<Dim == Dynamic,
        vsmc::Vector<T>, std::array<T, Dim>>::type;

    NormalMVDistribution<RealType, Dim> rnorm_;
    vector_type<RealType> a_;
    vector_type<RealType> b_;
    vector_type<RealType> z_;
    vector_type<unsigned> flag_;

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
            internal::proposal_normal_mv_check_param(
                dim, a_.data(), b_.data()),
            NormalMV);
    }
}; // class NormalMVProposal

} // namespace vsmc

#endif // VSMC_RNG_RANDOM_WALK_HPP
