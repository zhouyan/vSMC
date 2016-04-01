//============================================================================
// vSMC/include/vsmc/rng/extreme_value_distribution.hpp
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

#ifndef VSMC_RNG_EXTREME_VALUE_DISTRIBUTION_HPP
#define VSMC_RNG_EXTREME_VALUE_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool extreme_value_distribution_check_param(RealType, RealType b)
{
    return b > 0;
}

} // namespace vsmc::internal

/// \brief ExtremeValue distribution
/// \ingroup Distribution
template <typename RealType>
class ExtremeValueDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_2(ExtremeValue, extreme_value, a, 0, b, 1)

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
        U01OODistribution<RealType> u01;

        return param.a() - param.b() * std::log(-std::log(u01(rng)));
    }
}; // class ExtremeValueDistribution

namespace internal
{

template <std::size_t, typename RealType, typename RNGType>
inline void extreme_value_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType a, RealType b)
{
    u01_oo_distribution(rng, n, r);
    log(n, r, r);
    mul(n, static_cast<RealType>(-1), r, r);
    log(n, r, r);
    distribution_impl_location_scale(n, r, a, -b);
}

} // namespace vsmc::internal

/// \brief Generating extreme_value random variates
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_2(extreme_value, a, b)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(ExtremeValue, extreme_value, a, b)

} // namespace vsmc

#endif // VSMC_RNG_EXTREME_VALUE_DISTRIBUTION_HPP
