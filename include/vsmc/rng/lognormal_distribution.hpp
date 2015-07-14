//============================================================================
// vSMC/include/vsmc/rng/lognormal_distribution.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP
#define VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>

namespace vsmc
{

namespace internal
{

inline bool lognormal_distribution_check_param(double, double s)
{
    return s > 0;
}

} // namespace vsmc::internal

/// \brief Lognormal distribution
/// \ingroup Distribution
template <typename RealType>
class LognormalDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Lognormal, lognormal, RealType, m, 0, s, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    public:
    result_type min() const { return 0; }
    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() { normal_ = NormalDistribution<RealType>(0, 1); }

    private:
    NormalDistribution<RealType> normal_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        return std::exp(param.m() + param.s() * normal_(rng));
    }
}; // class LognormalDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void lognormal_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType m, RealType s)
{
    normal_distribution(rng, n, r, m, s);
    exp(n, r, r);
}

} // namespace vsmc::internal

/// \brief Generating lognormal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void lognormal_distribution(RNGType &rng, std::size_t n, RealType *r,
    RealType logmean, RealType logstddev)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::lognormal_distribution_impl(rng, k, r, logmean, logstddev);
    internal::lognormal_distribution_impl(rng, l, r, logmean, logstddev);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, LognormalDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP
