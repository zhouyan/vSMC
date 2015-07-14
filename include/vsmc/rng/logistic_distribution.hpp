//============================================================================
// vSMC/include/vsmc/rng/logistic_distribution.hpp
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

#ifndef VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP
#define VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

namespace internal
{

inline bool logistic_distribution_check_param(double, double b)
{
    return b > 0;
}

} // namespace vsmc::internal

/// \brief Logistic distribution
/// \ingroup Distribution
template <typename RealType>
class LogisticDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(Logistic, logistic, RealType, a, 0, b, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    public:
    result_type min() const { return -max(); }
    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        U01OODistribution<RealType> runif;
        result_type u = runif(rng);

        return param.a() + param.b() * std::log(u / (1 - u));
    }
}; // class LogisticDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void logistic_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    RealType s[K];
    u01_oo_distribution(rng, n, r);
    sub(n, static_cast<RealType>(1), r, s);
    div(n, r, s, r);
    log(n, r, r);
    fma(n, b, r, a, r);
}

} // namespace vsmc::internal

/// \brief Generating logistic random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void logistic_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::logistic_distribution_impl<k>(rng, k, r + i * k, a, b);
    internal::logistic_distribution_impl<k>(rng, l, r + m * k, a, b);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, LogisticDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_LOGISTIC_DISTRIBUTION_HPP