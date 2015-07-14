//============================================================================
// vSMC/include/vsmc/rng/cauchy_distribution.hpp
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

#ifndef VSMC_RNG_CAUCHY_DISTRIBUTION_HPP
#define VSMC_RNG_CAUCHY_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

namespace internal
{

inline bool cauchy_distribution_check_param(double, double b) { return b > 0; }

} // namespace vsmc::internal

/// \brief Cauchy distribution
/// \ingroup Distribution
template <typename RealType>
class CauchyDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Cauchy, cauchy, RealType, a, 0, b, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    result_type min() const { return -max(); }
    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        U01CODistribution<RealType> runif;

        return param.a() +
            param.b() * std::tan(const_pi<result_type>() * runif(rng));
    }
}; // class CauchyDistribution

namespace internal
{

template <typename RealType, typename RNGType>
inline void cauchy_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    u01_co_distribution(rng, n, r);
    mul(n, const_pi<RealType>(), r, r);
    tan(n, r, r);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = a + b * r[i];
}

} // namespace vsmc::internal

/// \brief Generating cauchy random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void cauchy_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::cauchy_distribution_impl(rng, k, r + i * k, a, b);
    internal::cauchy_distribution_impl(rng, l, r + m * k, a, b);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, CauchyDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_CAUCHY_DISTRIBUTION_HPP
