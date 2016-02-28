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

    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, result_type x, result_type &y,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        result_type r = 0;
        result_type q = proposal(rng, x, r);
        result_type p = log_target(r) - log_target(x) + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            y = r;
            return 1;
        } else {
            y = x;
            return 0;
        }
    }

    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, result_type x,
        result_type &y, LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        y = x;
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, y, y,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }

        return acc;
    }

    private:
    U01Distribution<RealType> runif_;
}; // class RandomWalk

template <typename RealType, std::size_t Dim>
class RandomWalkMV
{
    public:
    using result_type = RealType;

    RandomWalkMV()
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_FIXED_DIM(Dim, RandomWalkMV);
    }

    RandomWalkMV(std::size_t dim) : r_(dim)
    {
        VSMC_STATIC_ASSERT_RNG_RANDOM_WALK_MV_DYNAMIC_DIM(Dim, RandomWalkMV);
    }

    std::size_t dim() const { return r_.size(); }

    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(RNGType &rng, const result_type *x, result_type *y,
        LogTargetType &&log_target, ProposalType &&proposal)
    {
        result_type q = proposal(rng, dim(), x, r_.data());
        result_type p =
            log_target(dim(), r_.data()) - log_target(dim(), x) + q;
        result_type u = std::log(runif_(rng));

        if (u < p) {
            std::copy(r_.begin(), r_.end(), y);
            return 1;
        } else {
            std::copy_n(x, dim(), y);
            return 0;
        }
    }

    template <typename RNGType, typename LogTargetType, typename ProposalType>
    std::size_t operator()(std::size_t n, RNGType &rng, const result_type *x,
        result_type *y, LogTargetType &&log_target, ProposalType &&proposal)
    {
        std::size_t acc = 0;
        std::copy_n(x, dim(), y);
        for (std::size_t i = 0; i != n; ++i) {
            acc += operator()(rng, y, y,
                std::forward<LogTargetType>(log_target),
                std::forward<ProposalType>(proposal));
        }

        return acc;
    }

    private:
    U01Distribution<RealType> runif_;
    typename std::conditional<Dim == Dynamic, Vector<RealType>,
        std::array<RealType, Dim>>::type r_;
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
        : rnorm_(dim, nullptr, chol), a_(dim), b_(dim), flag_(dim)
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
        rnorm_(rng, y);
        result_type q = 0;
        for (std::size_t i = 0; i != dim(); ++i) {
            switch (flag_[i]) {
                case 0:
                    q += internal::proposal_normal_q(x[i], y[i], y[i]);
                    break;
                case 1:
                    q += internal::proposal_normal_qb(x[i], y[i], y[i], b_[i]);
                    break;
                case 2:
                    q += internal::proposal_normal_qa(x[i], y[i], y[i], a_[i]);
                    break;
                case 3:
                    q += internal::proposal_normal_qab(
                        x[i], y[i], y[i], a_[i], b_[i]);
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
    vector_type<unsigned> flag_;

    void init(std::size_t dim, const result_type *a, const result_type *b)
    {
        if (a == nullptr)
            std::fill(a_.begin(), a_.end(), 0);
        else
            std::copy_n(a, dim, a_.begin());

        if (b == nullptr)
            std::fill(b_.begin(), b_.end(), 0);
        else
            std::copy_n(b, dim, b_.begin());

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
