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

/// \brief Normal random walk kernel on (un)bounded support
/// \ingroup RandomWalk
template <typename RealType>
class RandomWalkNormal
{
    public:
    using result_type = RealType;

    RandomWalkNormal(result_type stddev = 1,
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
            case 0: return trans_0(x, y, z);
            case 1: return trans_b(x, y, z);
            case 2: return trans_a(x, y, z);
            case 3: return trans_ab(x, y, z);
            default: return 0;
        }
    }

    private:
    NormalDistribution<RealType> rnorm_;
    result_type a_;
    result_type b_;
    unsigned flag_;

    result_type trans_0(result_type x, result_type &y, result_type z) const
    {
        y = x + z;

        return 0;
    }

    result_type trans_a(result_type x, result_type &y, result_type z) const
    {
        y = a_ + (x - a_) * std::exp(z);

        return z;
    }

    result_type trans_b(result_type x, result_type &y, result_type z) const
    {
        y = b_ - (b_ - x) * std::exp(z);

        return z;
    }

    result_type trans_ab(result_type x, result_type &y, result_type z) const
    {
        result_type r = std::exp(z) * (x - a_) / (b_ - x);
        y = (a_ + b_ * r) / (1 + r);

        return std::log((y - a_) / (x - a_) * (b_ - y) / (b_ - x));
    }
}; // class RandomWalkNormal

} // namespace vsmc

#endif // VSMC_RNG_RANDOM_WALK_HPP
