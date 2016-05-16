//============================================================================
// vSMC/include/vsmc/rng/chi_squared_distribution.hpp
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

#ifndef VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP
#define VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/gamma_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool chi_squared_distribution_check_param(RealType n)
{
    return n > 0;
}

} // namespace vsmc::internal

/// \brief The \f$\chi^2\f$ distribution
/// \ingroup Distribution
template <typename RealType>
class ChiSquaredDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_1(ChiSquared, chi_squared, n, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_MEMBER_1(GammaDistribution<RealType>, gamma_)

    public:
    result_type min() const { return 0; }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() { gamma_ = GammaDistribution<RealType>(n() / 2, 2); }

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        if (param == param_)
            return gamma_(rng);

        GammaDistribution<RealType> gamma(param.n() / 2, 2);

        return gamma(rng);
    }
}; // class ChiSquaredDistribution

/// \brief Generating \f$\chi^2\f$ random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void chi_squared_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**chi_squared_distribution** used with RealType other than floating "
        "point types");

    gamma_distribution(rng, n, r, df / 2, static_cast<RealType>(2));
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_1(ChiSquared, chi_squared, n)

} // namespace vsmc

#endif // VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP
