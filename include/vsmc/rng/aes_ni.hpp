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

    ctr_type ctr() const { return ctr_; }

    key_type key() const { return key_seq_.key(); }

    void ctr(const ctr_type &c)
    {
        ctr_ = c;
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
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng1,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng2)
    {
        if (rng1.buffer_ != rng2.buffer_)
            return false;
        if (rng1.key_seq_ != rng2.key_seq_)
            return false;
        if (rng1.ctr_ != rng2.ctr_)
            return false;
        if (rng1.index_ != rng2.index_)
            return false;
        return true;
    }

    friend bool operator!=(
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng1,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng2)
    {
        return !(rng1 == rng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng)
    {
        if (!os.good())
            return os;

        os << rng.buffer_ << ' ';
        os << rng.key_seq_ << ' ';
        os << rng.ctr_ << ' ';
        os << rng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> &rng)
    {
        if (!is.good())
            return is;

        AESNIEngine<ResultType, KeySeqType, Rounds, Blocks> rng_tmp;
        is >> std::ws >> rng_tmp.buffer_;
        is >> std::ws >> rng_tmp.key_seq_;
        is >> std::ws >> rng_tmp.ctr_;
        is >> std::ws >> rng_tmp.index_;

        if (is.good())
            rng = std::move(rng_tmp);

        return is;
    }

    private:
    using state_type = std::array<M128I<>, Blocks>;

    static constexpr std::size_t M_ = Blocks * M128I<ResultType>::size();

    alignas(16) std::array<ResultType, M_> buffer_;
    KeySeqType key_seq_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        union {
            state_type state;
            std::array<ctr_type, Blocks> ctr_block;
            std::array<ResultType, M_> result;
        } buf;
        internal::increment(ctr_, buf.ctr_block);
        enc_first(buf.state);
        enc_round<1>(buf.state, std::integral_constant<bool, 1 < Rounds>());
        enc_last(buf.state);
        buffer_ = buf.result;
    }

    void enc_first(state_type &state)
    {
        __m128i round_key =
            key_seq_.get(std::integral_constant<std::size_t, 0>());
        enc_first<0>(state, round_key, std::true_type());
    }

    template <std::size_t>
    void enc_first(state_type &, const __m128i &, std::false_type)
    {
    }

    template <std::size_t B>
    void enc_first(state_type &state, const __m128i &round_key, std::true_type)
    {
        std::get<B>(state).value() =
            _mm_xor_si128(std::get<B>(state).value(), round_key);
        enc_first<B + 1>(
            state, round_key, std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_round(state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void enc_round(state_type &state, std::true_type)
    {
        __m128i round_key =
            key_seq_.get(std::integral_constant<std::size_t, N>());
        enc_round<0, N>(state, round_key, std::true_type());
        enc_round<N + 1>(
            state, std::integral_constant<bool, N + 1 < Rounds>());
    }

    template <std::size_t, std::size_t>
    void enc_round(state_type &, const __m128i &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t N>
    void enc_round(state_type &state, const __m128i &round_key, std::true_type)
    {
        std::get<B>(state).value() =
            _mm_aesenc_si128(std::get<B>(state).value(), round_key);
        enc_round<B + 1, N>(
            state, round_key, std::integral_constant<bool, B + 1 < Blocks>());
    }

    void enc_last(state_type &state)
    {
        __m128i round_key =
            key_seq_.get(std::integral_constant<std::size_t, Rounds>());
        enc_last<0>(state, round_key, std::true_type());
    }

    template <std::size_t>
    void enc_last(state_type &, const __m128i &, std::false_type)
    {
    }

    template <std::size_t B>
    void enc_last(state_type &state, const __m128i &round_key, std::true_type)
    {
        std::get<B>(state).value() =
            _mm_aesenclast_si128(std::get<B>(state).value(), round_key);
        enc_last<B + 1>(
            state, round_key, std::integral_constant<bool, B + 1 < Blocks>());
    }
}; // class AESNIEngine

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
