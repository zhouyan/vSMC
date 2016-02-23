//============================================================================
// vSMC/include/vsmc/rng/bernoulli_distribution.hpp
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

#ifndef VSMC_RNG_BERNOULLI_DISTRIBUTION_HPP
#define VSMC_RNG_BERNOULLI_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

namespace internal
{

inline bool bernoulli_distribution_check_param(double p)
{
    return p >= 0 && p <= 1;
}

} // namespace vsmc::internal

/// \brief Bernoulli distribution
/// \ingroup Distribution
template <typename IntType>
class BernoulliDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_1(
        Bernoulli, bernoulli, IntType, double, p, 0.5)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    public:
    result_type min VSMC_MNE() const { return static_cast<result_type>(0); }
    result_type max VSMC_MNE() const { return static_cast<result_type>(1); }
    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        U01CODistribution<double> runif;
        double u = runif(rng);

        return generate(u, param.p(), static_cast<result_type *>(nullptr));
    }

    static bool generate(double u, double p, bool *) { return u < p; }

    template <typename U>
    static U generate(double u, double p, U *)
    {
        return u < p ? 1 : 0;
    }
}; // class BernoulliDistribution

namespace internal
{

template <std::size_t K, typename IntType, typename RNGType>
inline void bernoulli_distribution_impl(
    RNGType &rng, std::size_t n, IntType *r, double p)
{
    double u[K];
    u01_co_distribution(rng, n, u);
    std::memset(r, 0, sizeof(IntType) * n);
    for (std::size_t i = 0; i != n; ++i)
        if (u[i] < p)
            r[i] = 1;
}

} // namespace vsmc::internal

/// \brief Generating bernoulli random variates
/// \ingroup Distribution
template <typename IntType, typename RNGType>
inline void bernoulli_distribution(
    RNGType &rng, std::size_t n, IntType *r, IntType p)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::bernoulli_distribution_impl<k>(rng, k, r + i * k, p);
    internal::bernoulli_distribution_impl<k>(rng, l, r + m * k, p);
}

template <typename IntType, typename RNGType>
inline void rng_rand(RNGType &rng, BernoulliDistribution<IntType> &dist,
    std::size_t n, IntType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_BERNOULLI_DISTRIBUTION_HPP
