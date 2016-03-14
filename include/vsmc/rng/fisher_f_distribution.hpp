//============================================================================
// vSMC/include/vsmc/rng/fisher_f_distribution.hpp
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

#ifndef VSMC_RNG_FISHER_F_DISTRIBUTION_HPP
#define VSMC_RNG_FISHER_F_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/chi_squared_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool fisher_f_distribution_check_param(RealType m, RealType n)
{
    return m > 0 && n > 0;
}

} // namespace vsmc::internal

/// \brief Fisher-F distribution
/// \ingroup Distribution
template <typename RealType>
class FisherFDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(FisherF, fisher_f, m, 1, n, 1)

    public:
    result_type min() const { return 0; }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset()
    {
        chi_squared_m_ = ChiSquaredDistribution<RealType>(m());
        chi_squared_n_ = ChiSquaredDistribution<RealType>(n());
    }

    private:
    ChiSquaredDistribution<RealType> chi_squared_m_;
    ChiSquaredDistribution<RealType> chi_squared_n_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        if (param == param_)
            return (chi_squared_m_(rng) / m()) / (chi_squared_n_(rng) / n());

        ChiSquaredDistribution<RealType> chi_squared_m(param.m());
        ChiSquaredDistribution<RealType> chi_squared_n(param.n());

        return (chi_squared_m(rng) / param.m()) /
            (chi_squared_n(rng) / param.n());
    }
}; // class FisherFDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void fisher_f_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType df1, RealType df2)
{
    RealType s[K];
    chi_squared_distribution(rng, n, s, df1);
    chi_squared_distribution(rng, n, r, df2);
    mul(n, 1 / df1, s, s);
    mul(n, 1 / df2, r, r);
    div(n, s, r, r);
}

} // namespace vsmc::internal

/// \brief Generating Fisher-F random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void fisher_f_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df1, RealType df2)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**fisher_f_distribution** USED WITH RealType OTHER THAN FLOATING "
        "POINT TYPES");

    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::fisher_f_distribution_impl<k>(rng, k, r, df1, df2);
    internal::fisher_f_distribution_impl<k>(rng, l, r, df1, df2);
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(FisherF, fisher_f, m, n)

} // namespace vsmc

#endif // VSMC_RNG_FISHER_F_DISTRIBUTION_HPP
