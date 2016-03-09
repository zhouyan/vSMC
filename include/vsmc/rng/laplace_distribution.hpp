//============================================================================
// vSMC/include/vsmc/rng/laplace_distribution.hpp
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

#ifndef VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
#define VSMC_RNG_LAPLACE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_LAPLACE_DISTRIBUTION_PARAM_CHECK(b)           \
    VSMC_RUNTIME_ASSERT((b > 0), "**LaplaceDistribution** CONSTRUCTED "       \
                                 "WITH INVALID SCALE PARAMETER VALUE")

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool laplace_distribution_check_param(RealType, RealType b)
{
    return b > 0;
}

} // namespace vsmc::internal

/// \brief Laplace distribution
/// \ingroup Distribution
template <typename RealType>
class LaplaceDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(
        Laplace, laplace, RealType, result_type, a, 0, result_type, b, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    public:
    result_type min() const
    {
        return std::numeric_limits<result_type>::lowest();
    }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        U01Distribution<RealType> runif;
        result_type u = runif(rng) - static_cast<result_type>(0.5);

        return u > 0 ? param.a() - param.b() * std::log(1 - 2 * u) :
                       param.a() + param.b() * std::log(1 + 2 * u);
    }
}; // class LaplaceDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void laplace_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    RealType s[K];
    u01_distribution(rng, n, r);
    sub(n, r, static_cast<RealType>(0.5), r);
    for (std::size_t i = 0; i != n; ++i) {
        if (r[i] > 0) {
            r[i] = 1 - 2 * r[i];
            s[i] = -b;
        } else {
            r[i] = 1 + 2 * r[i];
            s[i] = b;
        }
    }
    log(n, r, r);
    fma(n, s, r, a, r);
}

} // namespace vsmc::internal

/// \brief Generating laplace random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void laplace_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::laplace_distribution_impl<k>(rng, k, r + i * k, a, b);
    internal::laplace_distribution_impl<k>(rng, l, r + m * k, a, b);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, LaplaceDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
