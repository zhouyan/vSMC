//============================================================================
// vSMC/include/vsmc/rng/student_t_distribution.hpp
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

#ifndef VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP
#define VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/chi_squared_distribution.hpp>
#include <vsmc/rng/normal_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool student_t_distribution_check_param(RealType n)
{
    return n > 0;
}

} // namespace vsmc::internal

/// \brief Student-t distribution
/// \ingroup Distribution
template <typename RealType>
class StudentTDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_1(
        StudentT, student_t, RealType, result_type, n, 1)
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    public:
    result_type min VSMC_MNE() const
    {
        return -std::numeric_limits<result_type>::max VSMC_MNE();
    }

    result_type max VSMC_MNE() const
    {
        return std::numeric_limits<result_type>::max VSMC_MNE();
    }

    void reset()
    {
        chi_squared_ = ChiSquaredDistribution<RealType>(n());
        normal_ = NormalDistribution<RealType>(0, 1);
    }

    private:
    ChiSquaredDistribution<RealType> chi_squared_;
    NormalDistribution<RealType> normal_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        result_type z = normal_(rng);
        result_type u = 0;
        if (param == param_) {
            u = n() / chi_squared_(rng);
        } else {
            ChiSquaredDistribution<RealType> chi_squared(param.n());
            u = param.n() / chi_squared(rng);
        }

        return z * std::sqrt(u);
    }
}; // class StudentTDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void student_t_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType df)
{
    RealType s[K];
    chi_squared_distribution(rng, n, r, df);
    mul(n, 1 / df, r, r);
    sqrt(n, r, r);
    normal_distribution(
        rng, n, s, static_cast<RealType>(0), static_cast<RealType>(1));
    div(n, s, r, r);
}

} // namespace vsmc::internal

/// \brief Generating student-t random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void student_t_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::student_t_distribution_impl<k>(rng, k, r + i * k, df);
    internal::student_t_distribution_impl<k>(rng, l, r + m * k, df);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, StudentTDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP
