//============================================================================
// vSMC/include/vsmc/resample/transform.hpp
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

#ifndef VSMC_RESAMPLE_TRANSFORM_HPP
#define VSMC_RESAMPLE_TRANSFORM_HPP

#include <vsmc/resample/internal/common.hpp>

namespace vsmc
{

/// \brief Transform uniform [0, 1] sequence into replication numbers
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param weight N-vector of normalized weights
/// \param u01seq M ordered uniform [0, 1] random numbers
/// \param replication N-vector of replication numbers
template <typename IntType, typename U01SeqType>
inline void resample_trans_u01_rep(std::size_t N, std::size_t M,
    const double *weight, U01SeqType &&u01seq, IntType *replication)
{
    if (N == 0)
        return;

    if (N == 1) {
        *replication = static_cast<IntType>(M);
        return;
    }

    std::memset(replication, 0, sizeof(IntType) * N);
    if (M == 0)
        return;

    double accw = 0;
    std::size_t j = 0;
    for (std::size_t i = 0; i != N - 1; ++i) {
        accw += weight[i];
        while (j != M && u01seq[j] < accw) {
            ++replication[i];
            ++j;
        }
    }
    replication[N - 1] = static_cast<IntType>(M - j);
}

/// \brief Transform replication numbers into parent indices
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param replication N-vector of replication numbers
/// \param index M-vector of parent indices
template <typename IntType1, typename IntType2>
inline void resample_trans_rep_index(
    std::size_t N, std::size_t M, const IntType1 *replication, IntType2 *index)
{
    if (N == 0 || M == 0)
        return;

    const std::size_t K = std::min(N, M);
    IntType1 time = 0;
    std::size_t src = 0;

    auto good_src = [K, replication, &time, &src]() {
        return src < K ? replication[src] > time + 1 : replication[src] > time;
    };

    auto find_src = [&time, &src, &good_src]() {
        if (!good_src()) {
            time = 0;
            do
                ++src;
            while (!good_src());
        }
    };

    for (std::size_t dst = 0; dst != K; ++dst) {
        if (replication[dst] > 0) {
            index[dst] = static_cast<IntType2>(dst);
        } else {
            find_src();
            index[dst] = static_cast<IntType2>(src);
            ++time;
        }
    }

    for (std::size_t dst = K; dst < M; ++dst) {
        find_src();
        index[dst] = static_cast<IntType2>(src);
        ++time;
    }
}

/// \brief Transform normalized weights to normalized residual and integrals,
/// and return the number of remaining elements to be resampled
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param weight N-vector of normalized weights
/// \param resid N-vector of normalized residuals
/// \param integ N-vector of integral parts
template <typename IntType>
inline std::size_t resample_trans_residual(std::size_t N, std::size_t M,
    const double *weight, double *resid, IntType *integ)
{
    double integral = 0;
    const double coeff = static_cast<double>(M);
    for (std::size_t i = 0; i != N; ++i) {
        resid[i] = std::modf(coeff * weight[i], &integral);
        integ[i] = static_cast<IntType>(integral);
    }
    mul(N, 1 / std::accumulate(resid, resid + N, 0.0), resid, resid);
    IntType R = std::accumulate(integ, integ + N, static_cast<IntType>(0));

    return M - static_cast<std::size_t>(R);
}

} // namespace vsmc

#endif // VSMC_RESAMPLE_TRANSFORM_HPP
