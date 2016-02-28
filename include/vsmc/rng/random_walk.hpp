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

#define VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PARAM(flag, Name)                 \
    VSMC_RUNTIME_ASSERT(                                                      \
        (flag), "**RandomWal" #Name "** CONSTRUCTED WITH INVALID PARAMETERS")

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool random_walk_normal_check_param(
    RealType stddev, RealType a, RealType b)
{
    return (stddev > 0) && (a < b);
}

template <typename RealType>
inline bool random_walk_normal_mv_check_param(std::size_t dim,
    const RealType *, const std::size_t *a, const std::size_t *b)
{
    for (std::size_t i = 0; i != dim; ++i)
        if (a[i] > b[i])
            return false;
    return true;
}

template <typename RealType>
RealType random_walk_normal_q0(RealType x, RealType &y, RealType z)
{
    y = x + z;

    return 0;
}

template <typename RealType>
RealType random_walk_normal_qa(RealType x, RealType &y, RealType z, RealType a)
{
    y = a + (x - a) * std::exp(z);

    return z;
}

template <typename RealType>
RealType random_walk_normal_qb(RealType x, RealType &y, RealType z, RealType b)
{
    y = b - (b - x) * std::exp(z);

    return z;
}

template <typename RealType>
RealType random_walk_normal_qab(
    RealType x, RealType &y, RealType z, RealType a, RealType b)
{
    RealType r = std::exp(z) * (x - a) / (b - x);
    y = (a + b * r) / (1 + r);

    return std::log((y - a) / (x - a) * (b - y) / (b - x));
}

} // namespace vsmc::internal

/// \brief Random walk MCMC
/// \ingroup RandomWalk
template <typename RealType>
class RandomWalkMCMC
{
    public:
    using result_type = RealType;

    template <typename RNGType, typename LogTargetType,
        typename RandomWalkType>
    result_type operator()(RNGType &rng, result_type x,
        LogTargetType &&log_target, RandomWalkType &&random_walk)
    {
        result_type y = 0;
        result_type q = random_walk(rng, x, y);
        result_type p = log_target(y) - log_target(x) + q;
        result_type u = std::log(runif_(rng));

        return u > p ? x : y;
    }

    private:
    U01Distribution<RealType> runif_;
}; // class MCMC

/// \brief Normal random walk proposal
/// \ingroup RandomWalk
template <typename RealType>
class RandomWalkNormal
{
    public:
    using result_type = RealType;

    explicit RandomWalkNormal(result_type stddev = 1,
        result_type a = -std::numeric_limits<result_type>::infinity(),
        result_type b = std::numeric_limits<result_type>::infinity())
        : rnorm_(0, stddev), a_(a), b_(b), flag_(0)
    {
        VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PARAM(
            internal::random_walk_normal_check_param(stddev, a, b), Normal);

        unsigned lower = std::isfinite(a) ? 1 : 0;
        unsigned upper = std::isfinite(b) ? 1 : 0;
        flag_ = (lower << 1) + upper;
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, result_type x, result_type &y)
    {
        result_type z = rnorm_(rng);
        switch (flag_) {
            case 0: return internal::random_walk_normal_q0(x, y, z);
            case 1: return internal::random_walk_normal_qb(x, y, z, b_);
            case 2: return internal::random_walk_normal_qa(x, y, z, a_);
            case 3: return internal::random_walk_normal_qab(x, y, z, a_, b_);
            default: return 0;
        }
    }

    private:
    NormalDistribution<RealType> rnorm_;
    result_type a_;
    result_type b_;
    unsigned flag_;
}; // class RandomWalkNormal

/// \brief Multivariate Normal random walk proposal
/// \ingroup RandomWalk
template <typename RealType, std::size_t Dim>
class RandomWalkNormalMV
{
    public:
    using result_type = RealType;

    explicit RandomWalkNormalMV(const result_type *chol = nullptr,
        const result_type *a = nullptr, const result_type *b = nullptr)
        : rnorm_(nullptr, chol)
    {
        init(Dim, a, b);
        VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PARAM(
            internal::random_walk_normal_mv_check_param(
                Dim, chol, a_.data(), b_.data()),
            NormalMV);
    }

    explicit RandomWalkNormalMV(std::size_t dim = 1,
        const result_type *chol = nullptr, const result_type *a = nullptr,
        const result_type *b = nullptr)
        : rnorm_(dim, nullptr, chol), a_(dim), b_(dim), z_(dim), flag_(dim)
    {
        init(dim, a, b);
        std::copy_n(a, dim, a_.begin());
        std::copy_n(b, dim, b_.begin());
        VSMC_RUNTIME_ASSERT_RNG_RANDOM_WALK_PARAM(
            internal::random_walk_normal_check_param(
                dim, chol, a_.data(), b_.data()),
            NormalMV);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const result_type *x, result_type *y)
    {
        z_ = rnorm_(rng);
        result_type q = 0;
        for (std::size_t i = 0; i != dim(); ++i) {
            switch (flag_) {
                case 0:
                    q += internal::random_walk_normal_q0(x[i], y[i], z_[i]);
                    break;
                case 1:
                    q += internal::random_walk_normal_qb(
                        x[i], y[i], z_[i], b_[i]);
                    break;
                case 2:
                    q += internal::random_walk_normal_qa(
                        x[i], y[i], z_[i], a_[i]);
                    break;
                case 3:
                    q += internal::random_walk_normal_qab(
                        x[i], y[i], z_[i], a_[i], b_[i]);
                    break;
                default: break;
            }
        }

        return q;
    }

    std::size_t dim() const { return rnorm_.dim(); }

    private:
    template <typename T>
    using vtype = typename std::conditional<Dim == Dynamic, vsmc::Vector<T>,
        std::array<T, Dim>>::type;

    NormalMVDistribution<RealType, Dim> rnorm_;
    vtype<RealType> a_;
    vtype<RealType> b_;
    vtype<RealType> z_;
    vtype<unsigned> flag_;

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
    }
}; // class RandomWalkNormalMV

} // namespace vsmc

#endif // VSMC_RNG_RANDOM_WALK_HPP
