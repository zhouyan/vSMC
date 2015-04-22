//============================================================================
// vSMC/include/vsmc/resample/common.hpp
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

#ifndef VSMC_RESAMPLE_COMMON_HPP
#define VSMC_RESAMPLE_COMMON_HPP

#include <vsmc/internal/common.hpp>
#if VSMC_HAS_AES_NI
#include <vsmc/rng/aes.hpp>
#include <vsmc/rng/ars.hpp>
#endif
#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>
#include <vsmc/rng/u01.hpp>
#include <vsmc/utility/aligned_memory.hpp>

/// \brief Default RNG type for resampling
/// \ingroup Config
#ifndef VSMC_RESAMPLE_RNG_TYPE
#define VSMC_RESAMPLE_RNG_TYPE ::vsmc::Threefry4x64
#endif

namespace vsmc
{

namespace internal
{

// Given N sorted U01 random variates
// Compute M replication numbers based on M weights
template <typename IntType, typename U01SeqType>
inline void trans_usrp(std::size_t M, std::size_t N, const double *weight,
    U01SeqType &u01seq, IntType *replication)
{
    if (M == 0)
        return;

    if (M == 1) {
        *replication = static_cast<IntType>(N);
        return;
    }

    std::memset(replication, 0, sizeof(IntType) * M);

    if (N == 0)
        return;

    std::size_t n = 0;
    double accw = 0;
    for (std::size_t i = 0; i != M - 1; ++i) {
        accw += weight[i];
        while (n != N && u01seq[n] <= accw) {
            ++replication[i];
            ++n;
        }
    }
    replication[M - 1] = N - n;
}

// Given replication numbers, transfer them to copy_from index
template <typename IntType1, typename IntType2>
inline void trans_rpcf(std::size_t M, std::size_t N,
    const IntType1 *replication, IntType2 *copy_from)
{
    if (M != N) {
        std::size_t to = 0;
        for (std::size_t from = 0; from != M; ++from) {
            const IntType1 rep = replication[from];
            for (IntType1 r = 0; r != rep; ++r)
                copy_from[to++] = static_cast<IntType2>(from);
        }
        return;
    }

    IntType1 time = 0;
    IntType2 from = 0;
    for (std::size_t to = 0; to != N; ++to) {
        if (replication[to] != 0) {
            copy_from[to] = static_cast<IntType2>(to);
        } else {
            // replication[to] has zero child, copy from elsewhere
            if (replication[from] < time + 2) {
                // only 1 child left on replication[from]
                time = 0;
                do // move from to a position with at least 2 children
                    ++from;
                while (replication[from] < 2);
            }
            copy_from[to] = from;
            ++time;
        }
    }
}

template <typename IntType, typename U01SeqType>
inline void trans_uscf(std::size_t M, std::size_t N, const double *weight,
    U01SeqType &u01seq, IntType *copy_from)
{
    if (M == 0 || N == 0)
        return;

    std::memset(copy_from, 0, sizeof(IntType) * N);

    if (M == 1)
        return;

    std::size_t n = 0;
    double accw = 0;
    for (std::size_t i = 0; i != M - 1; ++i) {
        accw += weight[i];
        while (n != N && u01seq[n] <= accw)
            copy_from[n++] = static_cast<IntType>(i);
    }
    while (n != N)
        copy_from[n++] = static_cast<IntType>(M - 1);
}

} // namespace vsmc::internal

/// \brief Resampling schemes
/// \ingroup Resample
enum ResampleScheme {
    Multinomial,        ///< Multinomial resampling
    Residual,           ///< Residual resampling
    Stratified,         ///< Stratified resampling
    Systematic,         ///< Systematic resampling
    ResidualStratified, ///< Stratified resampling on residuals
    ResidualSystematic  ///< Systematic resampling on residuals
};                      // enum ResampleScheme

/// \brief Resample forward decleration
/// \ingroup Resample
template <typename>
class Resample;

/// \brief Resampling type of the built-in schemes
/// \ingroup Resample
template <ResampleScheme Scheme>
struct ResampleType {
    typedef Resample<std::integral_constant<ResampleScheme, Scheme>> type;
};

namespace traits
{

/// \brief Particle::resample_rng_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(
    ResampleRngType, resample_rng_type, VSMC_RESAMPLE_RNG_TYPE)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RESAMPLE_COMMON_HPP
