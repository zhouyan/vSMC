//============================================================================
// vSMC/include/vsmc/resample/residual_stratified.hpp
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

#ifndef VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP

#include <vsmc/resample/internal/common.hpp>
#include <vsmc/resample/transform.hpp>

namespace vsmc
{

/// \brief Residual stratified resampling
/// \ingroup Resample
class ResampleResidualStratified
{
    public:
    template <typename IntType, typename RNGType>
    void operator()(std::size_t M, std::size_t N, RNGType &rng,
        const double *weight, IntType *replication)
    {
        residual_.resize(M);
        integral_.resize(M);
        for (std::size_t i = 0; i != M; ++i)
            residual_[i] = std::modf(N * weight[i], integral_.data() + i);
        double coeff = 1 / math::asum(M, residual_.data(), 1);
        math::scal(M, coeff, residual_.data(), 1);

        IntType R = 0;
        for (std::size_t i = 0; i != M; ++i)
            R += static_cast<IntType>(integral_[i]);
        std::size_t NN = N - static_cast<std::size_t>(R);
        U01SequenceStratified<RNGType> u01seq(NN, rng);
        resample_trans_u01_rep(M, NN, residual_.data(), u01seq, replication);
        for (std::size_t i = 0; i != M; ++i)
            replication[i] += static_cast<IntType>(integral_[i]);
    }

    private:
    Vector<double> residual_;
    Vector<double> integral_;
}; // ResampleResidualStratified

/// \brief Type trait of ResidualStratified scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<ResidualStratified>
{
    public:
    using type = ResampleResidualStratified;
}; // class ResampleTypeTrait

} // namespace vsmc

#endif // VSMC_RESAMPLE_RESIDUAL_STRATIFIED_HPP
