//============================================================================
// vSMC/include/vsmc/rng/levy_distribution.hpp
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

#ifndef VSMC_RNG_LEVY_DISTRIBUTION_HPP
#define VSMC_RNG_LEVY_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool levy_distribution_check_param(RealType, RealType b)
{
    return b > 0;
}

} // namespace vsmc::internal

/// \brief Levy distribution
/// \ingroup Distribution
template <typename RealType>
class LevyDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Levy, levy, a, 0, b, 1)

    public:
    result_type min() const { return a(); }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() { normal_ = NormalDistribution<RealType>(0, 1); }

    private:
    NormalDistribution<RealType> normal_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        result_type r = normal_(rng);

        return param.a() + param.b() / (r * r);
    }
}; // class LevyDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void levy_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    normal_distribution(
        rng, n, r, static_cast<RealType>(0), static_cast<RealType>(1));
    sqr(n, r, r);
    inv(n, r, r);
    fma(n, b, r, a, r);
}

} // namespace vsmc::internal

/// \brief Generating levy random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void levy_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**levy_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::levy_distribution_impl<k>(rng, k, r, a, b);
    internal::levy_distribution_impl<k>(rng, l, r, a, b);
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(Levy, levy, a, b)

} // namespace vsmc

#endif // VSMC_RNG_LEVY_DISTRIBUTION_HPP
