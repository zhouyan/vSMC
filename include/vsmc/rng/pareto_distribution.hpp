//============================================================================
// vSMC/include/vsmc/rng/pareto_distribution.hpp
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

#ifndef VSMC_RNG_PARETO_DISTRIBUTION_HPP
#define VSMC_RNG_PARETO_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/exponential_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool pareto_distribution_check_param(RealType a, RealType b)
{
    return a > 0 && b > 0;
}

} // namespace vsmc::internal

/// \brief Pareto distribution
/// \ingroup Distribution
template <typename RealType>
class ParetoDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Pareto, pareto, a, 1, b, 1)

    public:
    result_type min() const { return a(); }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        U01Distribution<RealType> runif;

        return param.b() * std::exp(-std::log(runif(rng)) / param.a());
    }
}; // class ParetoDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void pareto_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    exponential_distribution(rng, n, r, a);
    exp(n, r, r);
    mul(n, b, r, r);
}

} // namespace vsmc::internal

/// \brief Generating pareto random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void pareto_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**pareto_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::pareto_distribution_impl<k>(rng, k, r + i * k, a, b);
    internal::pareto_distribution_impl<k>(rng, l, r + m * k, a, b);
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(Pareto, pareto, a, b)

} // namespace vsmc

#endif // VSMC_RNG_PARETO_DISTRIBUTION_HPP
