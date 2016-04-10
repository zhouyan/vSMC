//============================================================================
// vSMC/include/vsmc/rng/aes_ni.hpp
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

#ifndef VSMC_RNG_AES_NI_HPP
#define VSMC_RNG_AES_NI_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/counter.hpp>
#include <wmmintrin.h>

namespace vsmc
{

/// \brief RNG generator using AES-NI instructions
/// \ingroup AESNIRNG
template <typename ResultType, typename KeySeqType, std::size_t Rounds,
    std::size_t Blocks>
class AESNIGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**AESNIGenerator** USED WITH ResultType OTHER THAN UNSIGNED INTEGER "
        "TYPES");

    static_assert(128 >= std::numeric_limits<ResultType>::digits &&
            128 % std::numeric_limits<ResultType>::digits == 0,
        "**AESNIGenerator** USED WITH INVALID ResultType");

    static_assert(
        Rounds != 0, "**AESNIGenerator** USED WITH ROUNDS EQUAL TO ZERO");

    static_assert(
        Blocks != 0, "**AESNIGenerator** USED WITH Blocks EQUAL TO ZERO");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, M128I<ResultType>::size()>;
    using key_type = typename KeySeqType::key_type;

    static constexpr std::size_t size()
    {
        return Blocks * M128I<ResultType>::size();
    }

    void reset(const key_type &key) { key_seq_.reset(key); }

    void operator()(ctr_type &ctr, const key_type &key,
        std::array<ResultType, size()> &buffer) const
    {
        union {
            std::array<M128I<>, Blocks> state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, size()> result;
        } buf;

        std::array<M128I<>, Rounds + 1> rk;
        key_seq_(key, rk);

        increment(ctr, buf.ctr_block);
        enc(buf.state, rk);
        buffer = buf.result;
    }

    void operator()(ctr_type &ctr, const key_type &key, std::size_t n,
        std::array<ResultType, size()> *buffer) const
    {
        union {
            std::array<M128I<>, Blocks> state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, size()> result;
        } buf;

        std::array<M128I<>, Rounds + 1> rk;
        key_seq_(key, rk);

        for (std::size_t i = 0; i != n; ++i) {
            increment(ctr, buf.ctr_block);
            enc(buf.state, rk);
            buffer[i] = buf.result;
        }
    }

    private:
    KeySeqType key_seq_;

    template <typename std::size_t K>
    void enc(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk) const
    {
        enc_first(state, rk);
        enc_round<1>(state, rk, std::integral_constant<bool, 1 < Rounds>());
        enc_last(state, rk);
    }

    template <std::size_t K>
    void enc_first(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk) const
    {
        enc_first<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K>
    void enc_first(std::array<M128I<>, K> &,
        const std::array<M128I<>, Rounds + 1> &, std::false_type) const
    {
    }

    template <std::size_t B, std::size_t K>
    void enc_first(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk, std::true_type) const
    {
        std::get<B>(state) ^= std::get<0>(rk);
        enc_first<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t, std::size_t K>
    void enc_round(std::array<M128I<>, K> &,
        const std::array<M128I<>, Rounds + 1> &, std::false_type) const
    {
    }

    template <std::size_t N, std::size_t K>
    void enc_round(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk, std::true_type) const
    {
        enc_round_block<0, N>(state, rk, std::true_type());
        enc_round<N + 1>(
            state, rk, std::integral_constant<bool, N + 1 < Rounds>());
    }

    template <std::size_t, std::size_t, std::size_t K>
    void enc_round_block(std::array<M128I<>, K> &,
        const std::array<M128I<>, Rounds + 1> &, std::false_type) const
    {
    }

    template <std::size_t B, std::size_t N, std::size_t K>
    void enc_round_block(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk, std::true_type) const

    {
        std::get<B>(state) = _mm_aesenc_si128(
            std::get<B>(state).value(), std::get<N>(rk).value());
        enc_round_block<B + 1, N>(
            state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t K>
    void enc_last(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk) const
    {
        enc_last<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K>
    void enc_last(std::array<M128I<>, K> &,
        const std::array<M128I<>, Rounds + 1> &, std::false_type) const
    {
    }

    template <std::size_t B, std::size_t K>
    void enc_last(std::array<M128I<>, K> &state,
        const std::array<M128I<>, Rounds + 1> &rk, std::true_type) const
    {
        std::get<B>(state) = _mm_aesenclast_si128(
            std::get<B>(state).value(), std::get<Rounds>(rk).value());
        enc_last<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }
}; // class AESNIGenerator

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
template <typename ResultType, typename KeySeqType, std::size_t Rounds,
    std::size_t Blocks>
using AESNIEngine =
    CounterEngine<AESNIGenerator<ResultType, KeySeqType, Rounds, Blocks>>;

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
