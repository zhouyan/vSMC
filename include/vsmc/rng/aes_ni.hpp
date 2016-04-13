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
template <typename KeySeqType, std::size_t Rounds, std::size_t Blocks>
class AESNIGenerator
{
    static_assert(
        Rounds != 0, "**AESNIGenerator** USED WITH ROUNDS EQUAL TO ZERO");

    static_assert(
        Blocks != 0, "**AESNIGenerator** USED WITH Blocks EQUAL TO ZERO");

    public:
    using ctr_type = std::array<std::uint64_t, 2>;
    using key_type = typename KeySeqType::key_type;

    static constexpr std::size_t size() { return Blocks * sizeof(__m128i); }

    void reset(const key_type &key) { key_seq_.reset(key); }

    void enc(const ctr_type &ctr, ctr_type &buffer) const
    {
        union {
            std::array<__m128i, 1> state;
            ctr_type result;
        } buf;

        std::array<__m128i, Rounds + 1> rk_tmp;
        const std::array<__m128i, Rounds + 1> &rk = key_seq_(rk_tmp);

        buf.result = ctr;
        enc(buf.state, rk);
        buffer = buf.result;
    }

    template <typename ResultType>
    void operator()(ctr_type &ctr,
        std::array<ResultType, size() / sizeof(ResultType)> &buffer) const
    {
        union {
            std::array<__m128i, Blocks> state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, size() / sizeof(ResultType)> result;
        } buf;

        std::array<__m128i, Rounds + 1> rk_tmp;
        const std::array<__m128i, Rounds + 1> &rk = key_seq_(rk_tmp);

        increment(ctr, buf.ctr_block);
        enc(buf.state, rk);
        buffer = buf.result;
    }

    template <typename ResultType>
    void operator()(ctr_type &ctr, std::size_t n,
        std::array<ResultType, size() / sizeof(ResultType)> *buffer) const
    {
        union {
            std::array<__m128i, Blocks> state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, size() / sizeof(ResultType)> result;
        } buf;

        std::array<__m128i, Rounds + 1> rk_tmp;
        const std::array<__m128i, Rounds + 1> &rk = key_seq_(rk_tmp);

        for (std::size_t i = 0; i != n; ++i) {
            increment(ctr, buf.ctr_block);
            enc(buf.state, rk);
            buffer[i] = buf.result;
        }
    }

    friend bool operator==(
        const AESNIGenerator<KeySeqType, Rounds, Blocks> &gen1,
        const AESNIGenerator<KeySeqType, Rounds, Blocks> &gen2)
    {
        return gen1.key_seq_ == gen2.key_seq_;
    }

    friend bool operator!=(
        const AESNIGenerator<KeySeqType, Rounds, Blocks> &gen1,
        const AESNIGenerator<KeySeqType, Rounds, Blocks> &gen2)
    {
        return !(gen1 == gen2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIGenerator<KeySeqType, Rounds, Blocks> &gen)
    {
        if (!os)
            return os;

        os << gen.key_seq_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIGenerator<KeySeqType, Rounds, Blocks> &gen)
    {
        if (!is)
            return is;

        AESNIGenerator<KeySeqType, Rounds, Blocks> gen_tmp;
        is >> std::ws >> gen_tmp.key_seq_;

        if (static_cast<bool>(is))
            gen = std::move(gen_tmp);

        return is;
    }

    private:
    KeySeqType key_seq_;

    template <std::size_t K, std::size_t Rp1>
    void enc(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk) const
    {
        enc_first(state, rk);
        enc_round<1>(state, rk, std::integral_constant<bool, 2 < Rp1>());
        enc_last(state, rk);
    }

    template <std::size_t K, std::size_t Rp1>
    void enc_first(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk) const
    {
        enc_first<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K, std::size_t Rp1>
    void enc_first(std::array<__m128i, K> &, const std::array<__m128i, Rp1> &,
        std::false_type) const
    {
    }

    template <std::size_t B, std::size_t K, std::size_t Rp1>
    void enc_first(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk, std::true_type) const
    {
        std::get<B>(state) =
            _mm_xor_si128(std::get<B>(state), std::get<0>(rk));
        enc_first<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t, std::size_t K, std::size_t Rp1>
    void enc_round(std::array<__m128i, K> &, const std::array<__m128i, Rp1> &,
        std::false_type) const
    {
    }

    template <std::size_t N, std::size_t K, std::size_t Rp1>
    void enc_round(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk, std::true_type) const
    {
        enc_round_block<0, N>(state, rk, std::true_type());
        enc_round<N + 1>(
            state, rk, std::integral_constant<bool, N + 2 < Rp1>());
    }

    template <std::size_t, std::size_t, std::size_t K, std::size_t Rp1>
    void enc_round_block(std::array<__m128i, K> &,
        const std::array<__m128i, Rp1> &, std::false_type) const
    {
    }

    template <std::size_t B, std::size_t N, std::size_t K, std::size_t Rp1>
    void enc_round_block(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk, std::true_type) const

    {
        std::get<B>(state) =
            _mm_aesenc_si128(std::get<B>(state), std::get<N>(rk));
        enc_round_block<B + 1, N>(
            state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t K, std::size_t Rp1>
    void enc_last(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk) const
    {
        enc_last<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K, std::size_t Rp1>
    void enc_last(std::array<__m128i, K> &, const std::array<__m128i, Rp1> &,
        std::false_type) const
    {
    }

    template <std::size_t B, std::size_t K, std::size_t Rp1>
    void enc_last(std::array<__m128i, K> &state,
        const std::array<__m128i, Rp1> &rk, std::true_type) const
    {
        std::get<B>(state) =
            _mm_aesenclast_si128(std::get<B>(state), std::get<Rp1 - 1>(rk));
        enc_last<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }
}; // class AESNIGenerator

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
template <typename ResultType, typename KeySeqType, std::size_t Rounds,
    std::size_t Blocks>
using AESNIEngine =
    CounterEngine<ResultType, AESNIGenerator<KeySeqType, Rounds, Blocks>>;

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
