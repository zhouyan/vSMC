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

#include <vsmc/internal/common.hpp>

namespace vsmc
{

/// \brief Transform normalized weights to normalized residual and integrals,
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param weight N-vector of normalized weights
/// \param resid N-vector of normalized residuals
/// \param integ N-vector of integral parts
/// \return The number of remaining elements to be resampled
template <typename InputIter, typename OutputIterR, typename OutputIterI>
inline std::size_t resample_trans_residual(std::size_t N, std::size_t M,
    InputIter weight, OutputIterR resid, OutputIterI integ)
{
    using resid_type = typename std::iterator_traits<OutputIterR>::value_type;
    using integ_type = typename std::iterator_traits<OutputIterI>::value_type;

    static_assert(std::is_floating_point<resid_type>::value,
        "**resample_trans_residual** OUTPUT resid IS NOT OF FLOATING POINT "
        "TYPES");

    resid_type sum_resid = 0;
    integ_type sum_integ = 0;
    OutputIterR resid_i = resid;
    OutputIterI integ_i = integ;
    const resid_type coeff = static_cast<resid_type>(M);
    for (std::size_t i = 0; i != N; ++i, ++weight, ++resid_i, ++integ_i) {
        const resid_type w = coeff * static_cast<resid_type>(*weight);
        resid_type integral;
        *resid_i = std::modf(w, &integral);
        *integ_i = static_cast<integ_type>(integral);
        sum_resid += *resid_i;
        sum_integ += *integ_i;
    }

    const resid_type mul_resid = 1 / sum_resid;
    for (std::size_t i = 0; i != N; ++i, ++resid)
        *resid *= mul_resid;

    return M - static_cast<std::size_t>(sum_integ);
}

/// \brief Transform uniform [0, 1) sequence into replication numbers
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param weight N-vector of normalized weights
/// \param u01seq M ordered uniform [0, 1) random numbers
/// \param replication N-vector of replication numbers
template <typename InputIter, typename OutputIter, typename U01SeqType>
inline OutputIter resample_trans_u01_rep(std::size_t N, std::size_t M,
    InputIter weight, U01SeqType &&u01seq, OutputIter replication)
{
    using real_type = typename std::iterator_traits<InputIter>::value_type;
    using rep_type = typename std::iterator_traits<OutputIter>::value_type;

    if (N == 0)
        return replication;

    if (N == 1) {
        *replication++ = static_cast<rep_type>(M);
        return replication;
    }

    OutputIter rep = std::fill_n(replication, N, const_zero<rep_type>());
    if (M == 0)
        return rep;

    real_type accw = 0;
    std::size_t j = 0;
    for (std::size_t i = 0; i != N - 1; ++i, ++weight, ++replication) {
        accw += *weight;
        while (j != M && static_cast<real_type>(u01seq[j]) < accw) {
            *replication += 1;
            ++j;
        }
    }
    *replication++ = static_cast<rep_type>(M - j);

    return replication;
}

/// \brief Transform replication numbers into parent indices
/// \ingroup Resample
///
/// \param N Sample size before resampling
/// \param M Sample size after resampling
/// \param replication N-vector of replication numbers
/// \param index M-vector of parent indices
template <typename InputIter, typename OutputIter>
inline OutputIter resample_trans_rep_index(
    std::size_t N, std::size_t M, InputIter replication, OutputIter index)
{
    using rep_type = typename std::iterator_traits<InputIter>::value_type;
    using idx_type = typename std::iterator_traits<OutputIter>::value_type;

    if (N == 0 || M == 0)
        return index;

    const std::size_t K = std::min(N, M);
    rep_type time = 0;
    std::size_t src = 0;
    InputIter rep = replication;

    auto seek = [N, K, &time, &src, &rep]() {
        if (src < K && *rep < time + 2) {
            time = 0;
            do {
                ++src;
                ++rep;
            } while (src < K && *rep < time + 2);
        }
        if (src >= K && *rep < time + 1) {
            time = 0;
            do {
                ++src;
                ++rep;
            } while (src < N && *rep < time + 1);
        }
    };

    for (std::size_t dst = 0; dst != K; ++dst, ++replication, ++index) {
        if (*replication > 0) {
            *index = static_cast<idx_type>(dst);
        } else {
            seek();
            *index = static_cast<idx_type>(src);
            ++time;
        }
    }

    for (std::size_t dst = K; dst < M; ++dst, ++index) {
        seek();
        *index = static_cast<idx_type>(src);
        ++time;
    }

    return index;
}

} // namespace vsmc

#endif // VSMC_RESAMPLE_TRANSFORM_HPP
