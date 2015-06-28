//============================================================================
// vSMC/include/vsmc/rng/aes_ni.hpp
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

#ifndef VSMC_RNG_AES_NI_HPP
#define VSMC_RNG_AES_NI_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rngc/u01.h>
#include <vsmc/utility/simd.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_AES_NI_BLOCKS(Blocks)                          \
    VSMC_STATIC_ASSERT((Blocks > 0), "**AESNIEngine** USED WITH ZERO BLOCKS")

#define VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType)                 \
    VSMC_STATIC_ASSERT((std::is_unsigned<ResultType>::value),                 \
        "**AESNIEngine USED WITH ResultType NOT AN UNSIGNED INTEGER")

#define VSMC_STATIC_ASSERT_RNG_AES_NI                                         \
    VSMC_STATIC_ASSERT_RNG_AES_NI_BLOCKS(Blocks);                             \
    VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType);

namespace vsmc
{

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
template <typename ResultType, typename KeySeqType, std::size_t Rounds,
    std::size_t Blocks>
class AESNIEngine
{
    public:
    using result_type = ResultType;
    using key_type = typename KeySeqType::key_type;
    using ctr_type = std::array<ResultType, M128I<ResultType>::size()>;

    explicit AESNIEngine(result_type s = 0) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type,
            AESNIEngine<ResultType, KeySeqType, Rounds, Blocks>>::value>::type
            * = nullptr)
        : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(seq);
    }

    AESNIEngine(const key_type &k) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(k);
    }

    void seed(result_type s)
    {
        key_type k;
        k.fill(0);
        k.front() = s;
        seed(k);
    }

    template <typename SeedSeq>
    void seed(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          result_type, key_type>::value>::type * = nullptr)
    {
        key_type k;
        seq.generate(k.begin(), k.end());
        seed(k);
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        key_seq_.key(k);
        index_ = M_;
    }

    ctr_type ctr() const
    {
        ctr_type c;
        std::memcpy(c.data(), ctr_.data(), sizeof(ctr_type));

        return c;
    }

    key_type key() const { return key_seq_.key(); }

    void ctr(const ctr_type &c)
    {
        std::memcpy(ctr_.data(), c.data(), sizeof(ctr_type));
        index_ = M_;
    }

    void key(const key_type &k)
    {
        key_seq_.key(k);
        index_ = M_;
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generate_buffer();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void operator()(std::size_t n, result_type *r) { generate_buffer(n, r); }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= M_) {
            index_ += n;
            return;
        }

        n -= M_ - index_;
        if (n <= M_) {
            index_ = M_;
            operator()();
            index_ = n;
            return;
        }

        internal::increment(ctr_, static_cast<result_type>(n / M_));
        index_ = M_;
        operator()();
        index_ = n % M_;
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng1,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng2)
    {
        if (eng1.buffer_ != eng2.buffer_)
            return false;
        if (eng1.key_seq_ != eng2.key_seq_)
            return false;
        if (eng1.ctr_ != eng2.ctr_)
            return false;
        if (eng1.index_ != eng2.index_)
            return false;
        return true;
    }

    friend bool operator!=(
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng1,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng)
    {
        if (!os.good())
            return os;

        os << eng.buffer_ << ' ';
        os << eng.key_seq_ << ' ';
        os << eng.ctr_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &eng)
    {
        if (!is.good())
            return is;

        AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> eng_tmp;
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.key_seq_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    using rk_type = std::array<__m128i, Rounds + 1>;

    static constexpr std::size_t M_ = Blocks * M128I<ResultType>::size();

    alignas(16) std::array<ResultType, M_> buffer_;
    KeySeqType key_seq_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        union {
            std::array<__m128i, Blocks> state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, M_> result;
        } buf;

        rk_type rk;
        round_key(rk);
        internal::increment(ctr_, buf.ctr_block);
        enc_first(buf.state, rk);
        enc_round<1>(
            buf.state, rk, std::integral_constant<bool, 1 < Rounds>());
        enc_last(buf.state, rk);
        buffer_ = buf.result;
    }

    void generate_buffer(std::size_t n, result_type *r)
    {
        if (n * sizeof(result_type) <= 32) {
            for (std::size_t i = 0; i != n; ++i)
                r[i] = operator()();
            return;
        }

        std::size_t p = 32 -
            static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(r) % 32);
        if (p % sizeof(result_type) == 0) {
            p /= sizeof(result_type);
            for (std::size_t i = 0; i != p; ++i)
                r[i] = operator()();
            n -= p;
            r += p;
        }

        rk_type rk;
        round_key(rk);
        const std::size_t K = 8;
        const std::size_t M = K * M128I<ResultType>::size();
        const std::size_t m = n / M;
        ctr_type *c = reinterpret_cast<ctr_type *>(r);
        std::array<__m128i, K> *s =
            reinterpret_cast<std::array<__m128i, K> *>(r);
        internal::increment(m * K, ctr_, c);
        for (std::size_t i = 0; i != m; ++i) {
            enc_first(s[i], rk);
            enc_round<1>(s[i], rk, std::integral_constant<bool, 1 < Rounds>());
            enc_last(s[i], rk);
        }
        n -= m * M;
        r += m * M;

        for (std::size_t i = 0; i != n; ++i)
            r[i] = operator()();
    }

    template <std::size_t K>
    void enc_first(std::array<__m128i, K> &state, const rk_type &rk)
    {
        enc_first<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K>
    void enc_first(std::array<__m128i, K> &, const rk_type &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t K>
    void enc_first(
        std::array<__m128i, K> &state, const rk_type &rk, std::true_type)
    {
        std::get<B>(state) =
            _mm_xor_si128(std::get<B>(state), std::get<0>(rk));
        enc_first<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t, std::size_t K>
    void enc_round(std::array<__m128i, K> &, const rk_type &, std::false_type)
    {
    }

    template <std::size_t N, std::size_t K>
    void enc_round(
        std::array<__m128i, K> &state, const rk_type &rk, std::true_type)
    {
        enc_round_block<0, N>(state, rk, std::true_type());
        enc_round<N + 1>(
            state, rk, std::integral_constant<bool, N + 1 < Rounds>());
    }

    template <std::size_t, std::size_t, std::size_t K>
    void enc_round_block(
        std::array<__m128i, K> &, const rk_type &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t N, std::size_t K>
    void enc_round_block(
        std::array<__m128i, K> &state, const rk_type &rk, std::true_type)
    {
        std::get<B>(state) =
            _mm_aesenc_si128(std::get<B>(state), std::get<N>(rk));
        enc_round_block<B + 1, N>(
            state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <std::size_t K>
    void enc_last(std::array<__m128i, K> &state, const rk_type &rk)
    {
        enc_last<0>(state, rk, std::true_type());
    }

    template <std::size_t, std::size_t K>
    void enc_last(std::array<__m128i, K> &, const rk_type &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t K>
    void enc_last(
        std::array<__m128i, K> &state, const rk_type &rk, std::true_type)
    {
        std::get<B>(state) =
            _mm_aesenclast_si128(std::get<B>(state), std::get<Rounds>(rk));
        enc_last<B + 1>(state, rk, std::integral_constant<bool, B + 1 < K>());
    }

    template <size_t N>
    void round_key(rk_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void round_key(rk_type &rk, std::true_type)
    {
        std::get<N>(rk) =
            key_seq_.get(std::integral_constant<std::size_t, N>());
        round_key<N + 1>(rk, std::integral_constant < bool, N<Rounds>());
    }

    void round_key(rk_type &rk) { round_key<0>(rk, std::true_type()); }
}; // class AESNIEngine

template <typename ResultType, typename KeySeqType, std::size_t Rounds,
    std::size_t Blocks>
inline void rng_rand(AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng,
    std::size_t n, ResultType *r)
{
    rng(n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
