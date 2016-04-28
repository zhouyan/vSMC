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

#ifdef VSMC_GCC
#if __GNUC__ >= 6
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif
#endif

#define VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(N, val)                            \
    template <>                                                               \
    inline __m128i AESKeyGenAssist<N>(const __m128i &xmm)                     \
    {                                                                         \
        return _mm_aeskeygenassist_si128(xmm, val);                           \
    }

/// \brief AES-128 default rounds
#ifndef VSMC_RNG_AES128_ROUNDS
#define VSMC_RNG_AES128_ROUNDS 10
#endif

/// \brief AES-192 default rounds
#ifndef VSMC_RNG_AES192_ROUNDS
#define VSMC_RNG_AES192_ROUNDS 12
#endif

/// \brief AES-256 default rounds
#ifndef VSMC_RNG_AES256_ROUNDS
#define VSMC_RNG_AES256_ROUNDS 14
#endif

/// \brief ARSEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_ARS_ROUNDS
#define VSMC_RNG_ARS_ROUNDS 5
#endif

/// \brief AESEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_AES_NI_BLOCKS
#define VSMC_RNG_AES_NI_BLOCKS 8
#endif

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

        if (is)
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

namespace internal
{

template <std::size_t>
inline __m128i AESKeyGenAssist(const __m128i &);

VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x00, 0x8D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x01, 0x01)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x02, 0x02)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x03, 0x04)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x04, 0x08)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x05, 0x10)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x06, 0x20)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x07, 0x40)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x08, 0x80)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x09, 0x1B)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0A, 0x36)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0B, 0x6C)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0C, 0xD8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0D, 0xAB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0E, 0x4D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x0F, 0x9A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x10, 0x2F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x11, 0x5E)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x12, 0xBC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x13, 0x63)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x14, 0xC6)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x15, 0x97)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x16, 0x35)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x17, 0x6A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x18, 0xD4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x19, 0xB3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1A, 0x7D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1B, 0xFA)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1C, 0xEF)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1D, 0xC5)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1E, 0x91)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x1F, 0x39)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x20, 0x72)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x21, 0xE4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x22, 0xD3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x23, 0xBD)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x24, 0x61)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x25, 0xC2)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x26, 0x9F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x27, 0x25)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x28, 0x4A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x29, 0x94)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2A, 0x33)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2B, 0x66)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2C, 0xCC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2D, 0x83)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2E, 0x1D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x2F, 0x3A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x30, 0x74)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x31, 0xE8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x32, 0xCB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x33, 0x8D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x34, 0x01)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x35, 0x02)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x36, 0x04)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x37, 0x08)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x38, 0x10)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x39, 0x20)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3A, 0x40)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3B, 0x80)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3C, 0x1B)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3D, 0x36)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3E, 0x6C)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x3F, 0xD8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x40, 0xAB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x41, 0x4D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x42, 0x9A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x43, 0x2F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x44, 0x5E)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x45, 0xBC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x46, 0x63)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x47, 0xC6)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x48, 0x97)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x49, 0x35)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4A, 0x6A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4B, 0xD4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4C, 0xB3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4D, 0x7D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4E, 0xFA)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x4F, 0xEF)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x50, 0xC5)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x51, 0x91)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x52, 0x39)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x53, 0x72)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x54, 0xE4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x55, 0xD3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x56, 0xBD)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x57, 0x61)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x58, 0xC2)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x59, 0x9F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5A, 0x25)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5B, 0x4A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5C, 0x94)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5D, 0x33)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5E, 0x66)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x5F, 0xCC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x60, 0x83)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x61, 0x1D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x62, 0x3A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x63, 0x74)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x64, 0xE8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x65, 0xCB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x66, 0x8D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x67, 0x01)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x68, 0x02)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x69, 0x04)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6A, 0x08)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6B, 0x10)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6C, 0x20)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6D, 0x40)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6E, 0x80)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x6F, 0x1B)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x70, 0x36)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x71, 0x6C)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x72, 0xD8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x73, 0xAB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x74, 0x4D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x75, 0x9A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x76, 0x2F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x77, 0x5E)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x78, 0xBC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x79, 0x63)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7A, 0xC6)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7B, 0x97)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7C, 0x35)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7D, 0x6A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7E, 0xD4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x7F, 0xB3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x80, 0x7D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x81, 0xFA)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x82, 0xEF)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x83, 0xC5)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x84, 0x91)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x85, 0x39)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x86, 0x72)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x87, 0xE4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x88, 0xD3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x89, 0xBD)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8A, 0x61)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8B, 0xC2)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8C, 0x9F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8D, 0x25)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8E, 0x4A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x8F, 0x94)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x90, 0x33)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x91, 0x66)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x92, 0xCC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x93, 0x83)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x94, 0x1D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x95, 0x3A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x96, 0x74)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x97, 0xE8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x98, 0xCB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x99, 0x8D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9A, 0x01)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9B, 0x02)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9C, 0x04)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9D, 0x08)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9E, 0x10)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0x9F, 0x20)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA0, 0x40)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA1, 0x80)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA2, 0x1B)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA3, 0x36)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA4, 0x6C)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA5, 0xD8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA6, 0xAB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA7, 0x4D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA8, 0x9A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xA9, 0x2F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAA, 0x5E)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAB, 0xBC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAC, 0x63)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAD, 0xC6)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAE, 0x97)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xAF, 0x35)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB0, 0x6A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB1, 0xD4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB2, 0xB3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB3, 0x7D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB4, 0xFA)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB5, 0xEF)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB6, 0xC5)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB7, 0x91)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB8, 0x39)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xB9, 0x72)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBA, 0xE4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBB, 0xD3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBC, 0xBD)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBD, 0x61)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBE, 0xC2)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xBF, 0x9F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC0, 0x25)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC1, 0x4A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC2, 0x94)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC3, 0x33)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC4, 0x66)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC5, 0xCC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC6, 0x83)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC7, 0x1D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC8, 0x3A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xC9, 0x74)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCA, 0xE8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCB, 0xCB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCC, 0x8D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCD, 0x01)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCE, 0x02)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xCF, 0x04)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD0, 0x08)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD1, 0x10)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD2, 0x20)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD3, 0x40)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD4, 0x80)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD5, 0x1B)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD6, 0x36)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD7, 0x6C)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD8, 0xD8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xD9, 0xAB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDA, 0x4D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDB, 0x9A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDC, 0x2F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDD, 0x5E)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDE, 0xBC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xDF, 0x63)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE0, 0xC6)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE1, 0x97)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE2, 0x35)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE3, 0x6A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE4, 0xD4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE5, 0xB3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE6, 0x7D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE7, 0xFA)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE8, 0xEF)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xE9, 0xC5)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xEA, 0x91)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xEB, 0x39)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xEC, 0x72)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xED, 0xE4)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xEE, 0xD3)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xEF, 0xBD)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF0, 0x61)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF1, 0xC2)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF2, 0x9F)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF3, 0x25)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF4, 0x4A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF5, 0x94)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF6, 0x33)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF7, 0x66)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF8, 0xCC)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xF9, 0x83)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFA, 0x1D)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFB, 0x3A)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFC, 0x74)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFD, 0xE8)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFE, 0xCB)
VSMC_DEFINE_RNG_AES_KEY_GEN_ASSIST(0xFF, 0x8D)

template <std::size_t>
class ARSWeylConstant;

template <>
class ARSWeylConstant<0> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0x9E3779B97F4A7C15)>
{
}; // class ARSWeylConstant

template <>
class ARSWeylConstant<1> : public std::integral_constant<std::uint64_t,
                               UINT64_C(0xBB67AE8584CAA73B)>
{
}; // class ARSWeylConstant

} // namespace internal

/// \brief Default ARS constants
/// \ingroup AESNIRNG
class ARSConstants
{
    public:
    /// \brief Weyl constant of the I-th 64-bit element of the key
    template <std::size_t I>
    using weyl = internal::ARSWeylConstant<I>;
}; // class ARSConstants

namespace internal
{

template <std::size_t Rounds, typename KeySeqGenerator>
class AESKeySeq
{
    public:
    using key_type = typename KeySeqGenerator::key_type;

    void reset(const key_type &key)
    {
        KeySeqGenerator generator;
        generator(key, key_seq_);
    }

    const std::array<__m128i, Rounds + 1> &operator()(
        std::array<__m128i, Rounds + 1> &) const
    {
        return key_seq_;
    }

    friend bool operator==(const AESKeySeq<Rounds, KeySeqGenerator> &seq1,
        const AESKeySeq<Rounds, KeySeqGenerator> &seq2)
    {
        alignas(16) std::array<std::uint64_t, 2 * (Rounds + 1)> ks1;
        alignas(16) std::array<std::uint64_t, 2 * (Rounds + 1)> ks2;
        std::memcpy(ks1.data(), seq1.key_seq_.data(), 16 * (Rounds + 1));
        std::memcpy(ks2.data(), seq2.key_seq_.data(), 16 * (Rounds + 1));

        return ks1 == ks2;
    }

    friend bool operator!=(const AESKeySeq<Rounds, KeySeqGenerator> &seq1,
        const AESKeySeq<Rounds, KeySeqGenerator> &seq2)
    {
        return !(seq1 == seq2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESKeySeq<Rounds, KeySeqGenerator> &seq)
    {
        if (!os)
            return os;

        alignas(16) std::array<std::uint64_t, 2 * (Rounds + 1)> ks;
        std::memcpy(ks.data(), seq.key_seq_.data(), 16 * (Rounds + 1));
        os << ks;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESKeySeq<Rounds, KeySeqGenerator> &seq)
    {
        if (!is)
            return is;

        alignas(16) std::array<std::uint64_t, 2 * (Rounds + 1)> ks;
        is >> ks;
        if (is)
            std::memcpy(seq.key_seq_.data(), ks.data(), 16 * (Rounds + 1));

        return is;
    }

    private:
    std::array<__m128i, Rounds + 1> key_seq_;
}; // class AESKeySeq

class AES128KeySeqGenerator
{
    public:
    using key_type = std::array<std::uint64_t, 2>;

    template <std::size_t Rp1>
    void operator()(const key_type &key, std::array<__m128i, Rp1> &rk)
    {
        xmm1_ = _mm_loadu_si128(reinterpret_cast<const __m128i *>(key.data()));
        std::get<0>(rk) = xmm1_;
        generate_seq<1>(rk, std::integral_constant<bool, 1 < Rp1>());
    }

    private:
    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;

    template <std::size_t, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &, std::false_type)
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &rk, std::true_type)
    {
        xmm2_ = AESKeyGenAssist<N % 256>(xmm1_);
        expand_key();
        std::get<N>(rk) = xmm1_;
        generate_seq<N + 1>(rk, std::integral_constant<bool, N + 1 < Rp1>());
    }

    void expand_key()
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF); // pshufd xmm2, xmm2, 0xFF
        xmm3_ = _mm_slli_si128(xmm1_, 0x04);    // pslldq xmm3, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm3_);    // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128(xmm3_, 0x04);    // pslldq xmm3, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm3_);    // pxor   xmm1, xmm3
        xmm3_ = _mm_slli_si128(xmm3_, 0x04);    // pslldq xmm3, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm3_);    // pxor   xmm1, xmm3
        xmm1_ = _mm_xor_si128(xmm1_, xmm2_);    // pxor   xmm1, xmm2
    }
}; // class AES128KeySeq

class AES192KeySeqGenerator
{
    public:
    using key_type = std::array<std::uint64_t, 3>;

    template <std::size_t Rp1>
    void operator()(const key_type &key, std::array<__m128i, Rp1> &rk)
    {
        xmm1_ = _mm_loadu_si128(reinterpret_cast<const __m128i *>(key.data()));
        xmm7_ = _mm_set_epi64x(static_cast<VSMC_INT64>(0),
            static_cast<VSMC_INT64>(std::get<2>(key)));
        std::get<0>(rk) = xmm1_;
        std::get<1>(rk) = xmm7_;

        xmm3_ = _mm_setzero_si128();
        xmm6_ = _mm_setzero_si128();
        xmm4_ = _mm_shuffle_epi32(xmm7_, 0x4F); // pshufd xmm4, xmm7, 0x4F

        std::array<unsigned char, Rp1 * 16 + 16> rk_tmp;
        generate_seq<1, Rp1>(
            rk_tmp.data(), std::integral_constant<bool, 24 < Rp1 * 16>());
        copy_key(
            rk, rk_tmp.data(), std::integral_constant<bool, 24 < Rp1 * 16>());
    }

    private:
    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;
    __m128i xmm4_;
    __m128i xmm5_;
    __m128i xmm6_;
    __m128i xmm7_;

    template <std::size_t, std::size_t>
    void generate_seq(unsigned char *, std::false_type)
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_seq(unsigned char *rk_ptr, std::true_type)
    {
        generate_key<N>(rk_ptr);
        complete_key<N>(
            rk_ptr, std::integral_constant<bool, N * 24 + 16 < Rp1 * 16>());
        generate_seq<N + 1, Rp1>(
            rk_ptr, std::integral_constant<bool, N * 24 + 24 < Rp1 * 16>());
    }

    template <std::size_t N>
    void generate_key(unsigned char *rk_ptr)
    {
        // In entry, N * 24 < Rp1 * 16
        // Required Storage: N * 24 + 16;

        xmm2_ = AESKeyGenAssist<N % 256>(xmm4_);
        generate_key_expansion();
        _mm_storeu_si128(reinterpret_cast<__m128i *>(rk_ptr + N * 24), xmm1_);
    }

    template <std::size_t>
    void complete_key(unsigned char *, std::false_type)
    {
    }

    template <std::size_t N>
    void complete_key(unsigned char *rk_ptr, std::true_type)
    {
        // In entry, N * 24 + 16 < Rp1 * 16
        // Required storage: N * 24 + 32

        complete_key_expansion();
        _mm_storeu_si128(
            reinterpret_cast<__m128i *>(rk_ptr + N * 24 + 16), xmm7_);
    }

    void generate_key_expansion()
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF); // pshufd xmm2, xmm2, 0xFF
        xmm3_ = _mm_castps_si128(               // shufps xmm3, xmm1, 0x10
            _mm_shuffle_ps(
                _mm_castsi128_ps(xmm3_), _mm_castsi128_ps(xmm1_), 0x10));
        xmm1_ = _mm_xor_si128(xmm1_, xmm3_);     // pxor   xmm1, xmm3
        xmm3_ = _mm_castps_si128(_mm_shuffle_ps( // shufps xmm3, xmm1, 0x10
            _mm_castsi128_ps(xmm3_), _mm_castsi128_ps(xmm1_), 0x8C));
        xmm1_ = _mm_xor_si128(xmm1_, xmm3_); // pxor   xmm1, xmm3
        xmm1_ = _mm_xor_si128(xmm1_, xmm2_); // pxor   xmm1, xmm2
    }

    void complete_key_expansion()
    {
        xmm5_ = xmm4_;                           // movdqa xmm5, xmm4
        xmm5_ = _mm_slli_si128(xmm5_, 0x04);     // pslldq xmm5, 0x04
        xmm6_ = _mm_castps_si128(_mm_shuffle_ps( // shufps xmm6, xmm1, 0x10
            _mm_castsi128_ps(xmm6_), _mm_castsi128_ps(xmm1_), 0xF0));
        xmm6_ = _mm_xor_si128(xmm6_, xmm5_);    // pxor   xmm6, xmm5
        xmm4_ = _mm_xor_si128(xmm4_, xmm6_);    // pxor   xmm4, xmm6
        xmm7_ = _mm_shuffle_epi32(xmm4_, 0x0E); // pshufd xmm7, xmm4, 0x0E
    }

    template <std::size_t Rp1>
    void copy_key(
        std::array<__m128i, Rp1> &, const unsigned char *, std::false_type)
    {
    }

    template <std::size_t Rp1>
    void copy_key(std::array<__m128i, Rp1> &rk, const unsigned char *rk_ptr,
        std::true_type)
    {
        unsigned char *dst = reinterpret_cast<unsigned char *>(rk.data());
        std::memcpy(dst + 24, rk_ptr + 24, Rp1 * 16 - 24);
    }
}; // class AES192KeySeq

class AES256KeySeqGenerator
{
    public:
    using key_type = std::array<std::uint64_t, 4>;

    template <std::size_t Rp1>
    void operator()(const key_type &key, std::array<__m128i, Rp1> &rk)
    {
        xmm1_ = _mm_loadu_si128(reinterpret_cast<const __m128i *>(key.data()));
        xmm3_ =
            _mm_loadu_si128(reinterpret_cast<const __m128i *>(key.data() + 2));
        std::get<0>(rk) = xmm1_;
        std::get<1>(rk) = xmm3_;
        generate_seq<2>(rk, std::integral_constant<bool, 2 < Rp1>());
    }

    private:
    __m128i xmm1_;
    __m128i xmm2_;
    __m128i xmm3_;
    __m128i xmm4_;

    template <std::size_t, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &, std::false_type)
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_seq(std::array<__m128i, Rp1> &rk, std::true_type)
    {
        generate_key<N>(rk, std::integral_constant<bool, N % 2 == 0>());
        generate_seq<N + 1>(rk, std::integral_constant<bool, N + 1 < Rp1>());
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_key(std::array<__m128i, Rp1> &rk, std::true_type)
    {
        xmm2_ = AESKeyGenAssist<(N / 2) % 256>(xmm3_);
        expand_key(std::true_type());
        std::get<N>(rk) = xmm1_;
    }

    template <std::size_t N, std::size_t Rp1>
    void generate_key(std::array<__m128i, Rp1> &rk, std::false_type)
    {
        xmm4_ = _mm_aeskeygenassist_si128(xmm1_, 0);
        expand_key(std::false_type());
        std::get<N>(rk) = xmm3_;
    }

    void expand_key(std::true_type)
    {
        xmm2_ = _mm_shuffle_epi32(xmm2_, 0xFF); // pshufd xmm2, xmm2, 0xFF
        xmm4_ = _mm_slli_si128(xmm1_, 0x04);    // pslldq xmm4, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm4_);    // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128(xmm4_, 0x04);    // pslldq xmm4, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm4_);    // pxor   xmm1, xmm4
        xmm4_ = _mm_slli_si128(xmm4_, 0x04);    // pslldq xmm4, 0x04
        xmm1_ = _mm_xor_si128(xmm1_, xmm4_);    // pxor   xmm1, xmm4
        xmm1_ = _mm_xor_si128(xmm1_, xmm2_);    // pxor   xmm1, xmm2
    }

    void expand_key(std::false_type)
    {
        xmm2_ = _mm_shuffle_epi32(xmm4_, 0xAA); // pshufd xmm2, xmm4, 0xAA
        xmm4_ = _mm_slli_si128(xmm3_, 0x04);    // pslldq xmm4, 0x04
        xmm3_ = _mm_xor_si128(xmm3_, xmm4_);    // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128(xmm4_, 0x04);    // pslldq xmm4, 0x04
        xmm3_ = _mm_xor_si128(xmm3_, xmm4_);    // pxor   xmm3, xmm4
        xmm4_ = _mm_slli_si128(xmm4_, 0x04);    // pslldq xmm4, 0x04
        xmm3_ = _mm_xor_si128(xmm3_, xmm4_);    // pxor   xmm3, xmm4
        xmm3_ = _mm_xor_si128(xmm3_, xmm2_);    // pxor   xmm1, xmm2
    }
}; // class AESKey256

template <typename Constants>
class ARSKeySeqImpl
{
    template <std::size_t I>
    using weyl = typename Constants::template weyl<I>;

    public:
    using key_type = std::array<std::uint64_t, 2>;

    ARSKeySeqImpl() : key_(_mm_setzero_si128()) {}

    void reset(const key_type &key)
    {
        key_ = _mm_loadu_si128(reinterpret_cast<const __m128i *>(key.data()));
    }

    template <std::size_t Rp1>
    const std::array<__m128i, Rp1> &operator()(
        std::array<__m128i, Rp1> &rk) const
    {
        __m128i w = _mm_set_epi64x(static_cast<VSMC_INT64>(weyl<1>::value),
            static_cast<VSMC_INT64>(weyl<0>::value));
        std::get<0>(rk) = key_;
        generate<1>(rk, w, std::integral_constant<bool, 1 < Rp1>());

        return rk;
    }

    friend bool operator==(
        const ARSKeySeqImpl &seq1, const ARSKeySeqImpl &seq2)
    {
        alignas(16) key_type k1;
        alignas(16) key_type k2;
        _mm_store_si128(reinterpret_cast<__m128i *>(k1.data()), seq1.key_);
        _mm_store_si128(reinterpret_cast<__m128i *>(k2.data()), seq2.key_);

        return k1 == k2;
    }

    friend bool operator!=(
        const ARSKeySeqImpl &seq1, const ARSKeySeqImpl &seq2)
    {
        return !(seq1 == seq2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const ARSKeySeqImpl &seq)
    {
        if (!os)
            return os;

        alignas(16) key_type k;
        _mm_store_si128(reinterpret_cast<__m128i *>(k.data()), seq.key_);
        os << k;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, ARSKeySeqImpl &seq)
    {
        if (!is)
            return is;

        alignas(16) key_type k = {{0}};
        is >> k;
        if (is) {
            seq.key_ =
                _mm_load_si128(reinterpret_cast<const __m128i *>(k.data()));
        }

        return is;
    }

    private:
    template <std::size_t, std::size_t Rp1>
    void generate(
        std::array<__m128i, Rp1> &, const __m128i &, std::false_type) const
    {
    }

    template <std::size_t N, std::size_t Rp1>
    void generate(
        std::array<__m128i, Rp1> &rk, const __m128i &w, std::true_type) const
    {
        std::get<N>(rk) = _mm_add_epi64(std::get<N - 1>(rk), w);
        generate<N + 1>(rk, w, std::integral_constant<bool, N + 1 < Rp1>());
    }

    private:
    __m128i key_;
}; // class ARSKeySeqImpl

} // namespace vsmc::internal

/// \brief AES128Engine key sequence generator
/// \ingroup AESNIRNG
template <std::size_t Rounds>
using AES128KeySeq =
    internal::AESKeySeq<Rounds, internal::AES128KeySeqGenerator>;

/// \brief AES192Engine key sequence generator
/// \ingroup AESNIRNG
template <std::size_t Rounds>
using AES192KeySeq =
    internal::AESKeySeq<Rounds, internal::AES192KeySeqGenerator>;

/// \brief AES256Engine key sequence generator
/// \ingroup AESNIRNG
template <std::size_t Rounds>
using AES256KeySeq =
    internal::AESKeySeq<Rounds, internal::AES256KeySeqGenerator>;

/// \brief Default ARSEngine key sequence generator
/// \ingroup AESNIRNG
///
/// \tparam Constants A trait class that defines algorithm constants, see
/// ARSConstants
///
/// \details
/// This generator implement the ARS algorithm in
/// [Random123](http://www.deshawresearch.com/resources_random123.html),
/// developed John K. Salmon, Mark A. Moraes, Ron O. Dror, and David E. Shaw.
template <typename Constants = ARSConstants>
using ARSKeySeq = internal::ARSKeySeqImpl<Constants>;

/// \brief AES-128 RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_AES128_ROUNDS,
    std::size_t Blocks = VSMC_RNG_AES_NI_BLOCKS>
using AES128Engine =
    AESNIEngine<ResultType, AES128KeySeq<Rounds>, Rounds, Blocks>;

/// \brief AES-192 RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_AES192_ROUNDS,
    std::size_t Blocks = VSMC_RNG_AES_NI_BLOCKS>
using AES192Engine =
    AESNIEngine<ResultType, AES192KeySeq<Rounds>, Rounds, Blocks>;

/// \brief AES-256 RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_AES256_ROUNDS,
    std::size_t Blocks = VSMC_RNG_AES_NI_BLOCKS>
using AES256Engine =
    AESNIEngine<ResultType, AES256KeySeq<Rounds>, Rounds, Blocks>;

/// \brief ARS RNG engine
/// \ingroup AESNIRNG
template <typename ResultType, std::size_t Rounds = VSMC_RNG_ARS_ROUNDS,
    std::size_t Blocks = VSMC_RNG_AES_NI_BLOCKS,
    typename Constants = ARSConstants>
using ARSEngine =
    AESNIEngine<ResultType, ARSKeySeq<Constants>, Rounds, Blocks>;

/// \brief AES-128 RNG engine with 32-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES128x1 = AES128Engine<std::uint32_t, VSMC_RNG_AES128_ROUNDS, 1>;

/// \brief AES-128 RNG engine with 32-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES128x2 = AES128Engine<std::uint32_t, VSMC_RNG_AES128_ROUNDS, 2>;

/// \brief AES-128 RNG engine with 32-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES128x4 = AES128Engine<std::uint32_t, VSMC_RNG_AES128_ROUNDS, 4>;

/// \brief AES-128 RNG engine with 32-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES128x8 = AES128Engine<std::uint32_t, VSMC_RNG_AES128_ROUNDS, 8>;

/// \brief AES-128 RNG engine with 64-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES128x1_64 = AES128Engine<std::uint64_t, VSMC_RNG_AES128_ROUNDS, 1>;

/// \brief AES-128 RNG engine with 64-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES128x2_64 = AES128Engine<std::uint64_t, VSMC_RNG_AES128_ROUNDS, 2>;

/// \brief AES-128 RNG engine with 64-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES128x4_64 = AES128Engine<std::uint64_t, VSMC_RNG_AES128_ROUNDS, 4>;

/// \brief AES-128 RNG engine with 64-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES128x8_64 = AES128Engine<std::uint64_t, VSMC_RNG_AES128_ROUNDS, 8>;

/// \brief AES-128 RNG engine with 32-bit integers output
using AES128 = AES128Engine<std::uint32_t>;

/// \brief AES-128 RNG engine with 64-bit integers output
/// \ingroup AESNIRNG
using AES128_64 = AES128Engine<std::uint64_t>;

/// \brief AES-192 RNG engine with 32-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES192x1 = AES192Engine<std::uint32_t, VSMC_RNG_AES192_ROUNDS, 1>;

/// \brief AES-192 RNG engine with 32-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES192x2 = AES192Engine<std::uint32_t, VSMC_RNG_AES192_ROUNDS, 2>;

/// \brief AES-192 RNG engine with 32-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES192x4 = AES192Engine<std::uint32_t, VSMC_RNG_AES192_ROUNDS, 4>;

/// \brief AES-192 RNG engine with 32-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES192x8 = AES192Engine<std::uint32_t, VSMC_RNG_AES192_ROUNDS, 8>;

/// \brief AES-192 RNG engine with 64-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES192x1_64 = AES192Engine<std::uint64_t, VSMC_RNG_AES192_ROUNDS, 1>;

/// \brief AES-192 RNG engine with 64-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES192x2_64 = AES192Engine<std::uint64_t, VSMC_RNG_AES192_ROUNDS, 2>;

/// \brief AES-192 RNG engine with 64-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES192x4_64 = AES192Engine<std::uint64_t, VSMC_RNG_AES192_ROUNDS, 4>;

/// \brief AES-192 RNG engine with 64-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES192x8_64 = AES192Engine<std::uint64_t, VSMC_RNG_AES192_ROUNDS, 8>;

/// \brief AES-192 RNG engine with 32-bit integers output
/// \ingroup AESNIRNG
using AES192 = AES192Engine<std::uint32_t>;

/// \brief AES-192 RNG engine with 64-bit integers output
/// \ingroup AESNIRNG
using AES192_64 = AES192Engine<std::uint64_t>;

/// \brief AES-256 RNG engine with 32-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES256x1 = AES256Engine<std::uint32_t, VSMC_RNG_AES256_ROUNDS, 1>;

/// \brief AES-256 RNG engine with 32-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES256x2 = AES256Engine<std::uint32_t, VSMC_RNG_AES256_ROUNDS, 2>;

/// \brief AES-256 RNG engine with 32-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES256x4 = AES256Engine<std::uint32_t, VSMC_RNG_AES256_ROUNDS, 4>;

/// \brief AES-256 RNG engine with 32-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES256x8 = AES256Engine<std::uint32_t, VSMC_RNG_AES256_ROUNDS, 8>;

/// \brief AES-256 RNG engine with 64-bit integers output, 1 block
/// \ingroup AESNIRNG
using AES256x1_64 = AES256Engine<std::uint64_t, VSMC_RNG_AES256_ROUNDS, 1>;

/// \brief AES-256 RNG engine with 64-bit integers output, 2 blocks
/// \ingroup AESNIRNG
using AES256x2_64 = AES256Engine<std::uint64_t, VSMC_RNG_AES256_ROUNDS, 2>;

/// \brief AES-256 RNG engine with 64-bit integers output, 4 blocks
/// \ingroup AESNIRNG
using AES256x4_64 = AES256Engine<std::uint64_t, VSMC_RNG_AES256_ROUNDS, 4>;

/// \brief AES-256 RNG engine with 64-bit integers output, 8 blocks
/// \ingroup AESNIRNG
using AES256x8_64 = AES256Engine<std::uint64_t, VSMC_RNG_AES256_ROUNDS, 8>;

/// \brief AES-256 RNG engine with 32-bit integers output
/// \ingroup AESNIRNG
using AES256 = AES256Engine<std::uint32_t>;

/// \brief AES-256 RNG engine with 64-bit integers output
/// \ingroup AESNIRNG
using AES256_64 = AES256Engine<std::uint64_t>;

/// \brief ARS RNG engine with 32-bit integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARSx1 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 32-bit integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx2 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 32-bit integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx4 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 32-bit integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx8 = ARSEngine<std::uint32_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 64-bit integers output, 1 block and default
/// rounds
/// \ingroup AESNIRNG
using ARSx1_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 1>;

/// \brief ARS RNG engine with 64-bit integers output, 2 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx2_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 2>;

/// \brief ARS RNG engine with 64-bit integers output, 4 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx4_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 4>;

/// \brief ARS RNG engine with 64-bit integers output, 8 blocks and default
/// rounds
/// \ingroup AESNIRNG
using ARSx8_64 = ARSEngine<std::uint64_t, VSMC_RNG_ARS_ROUNDS, 8>;

/// \brief ARS RNG engine with 32-bit integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS = ARSEngine<std::uint32_t>;

/// \brief ARS RNG engine with 64-bit integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
using ARS_64 = ARSEngine<std::uint64_t>;

} // namespace vsmc

#ifdef VSMC_GCC
#if __GNUC__ >= 6
#pragma GCC diagnostic pop
#endif
#endif

#endif // VSMC_RNG_AES_NI_HPP
