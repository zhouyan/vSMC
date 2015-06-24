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

namespace internal
{

template <typename KeySeq, bool KeySeqInit, std::size_t Rounds>
class AESNIKeySeqStorage;

template <typename KeySeq, std::size_t Rounds>
class AESNIKeySeqStorage<KeySeq, true, Rounds>
{
    public:
    using key_type = typename KeySeq::key_type;
    using key_seq_type = std::array<M128I<>, Rounds + 1>;

    key_seq_type get(const key_type &) const { return key_seq_; }

    void set(const key_type &k)
    {
        KeySeq seq;
        seq.generate(k, key_seq_);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        if (!os.good())
            return os;

        for (std::size_t i = 0; i != Rounds + 1; ++i)
            os << ks.key_seq_[i] << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        if (!is.good())
            return is;

        AESNIKeySeqStorage<KeySeq, true, Rounds> ks_tmp;
        for (std::size_t i = 0; i != Rounds + 1; ++i)
            is >> std::ws >> ks_tmp.key_seq_[i];

        if (is.good())
            ks = std::move(ks_tmp);

        return is;
    }

    private:
    key_seq_type key_seq_;
}; // class AESNIKeySeqStorage

template <typename KeySeq, std::size_t Rounds>
class AESNIKeySeqStorage<KeySeq, false, Rounds>
{
    public:
    using key_type = typename KeySeq::key_type;
    using key_seq_type = std::array<M128I<>, Rounds + 1>;

    key_seq_type get(const key_type &k) const
    {
        key_seq_type ks;
        KeySeq seq;
        seq.generate(k, ks);

        return ks;
    }

    void set(const key_type &) {}

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIKeySeqStorage<KeySeq, false, Rounds> &)
    {
        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIKeySeqStorage<KeySeq, false, Rounds> &)
    {
        return is;
    }
}; // class AESNIKeySeqStorage

} // namespace vsmc::internal

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
template <typename ResultType, typename KeySeq, bool KeySeqInit,
    std::size_t Rounds, std::size_t Blocks>
class AESNIEngine
{
    public:
    using result_type = ResultType;
    using key_type = typename KeySeq::key_type;
    using key_seq_type = std::array<M128I<>, Rounds + 1>;
    using ctr_type = std::array<ResultType, M128I<ResultType>::size()>;

    explicit AESNIEngine(result_type s = 0) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine(SeedSeq &seq,
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, result_type, key_type,
                AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds,
                                      Blocks>>::value>::type * = nullptr)
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
        ctr_.fill(0);
        key_.fill(0);
        key_.front() = s;
        key_seq_.set(key_);
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          result_type, key_type>::value>::type * = nullptr)
    {
        ctr_.fill(0);
        seq.generate(key_.begin(), key_.end());
        key_seq_.set(key_);
        index_ = M_;
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        key_ = k;
        key_seq_.set(k);
        index_ = M_;
    }

    ctr_type ctr() const { return ctr_; }

    key_type key() const { return key_; }

    key_seq_type key_seq() const { return key_seq_.get(key_); }

    void ctr(const ctr_type &c)
    {
        ctr_ = c;
        index_ = M_;
    }

    void key(const key_type &k)
    {
        key_ = k;
        key_seq_.set(k);
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

    friend bool operator==(const AESNIEngine<ResultType, KeySeq, KeySeqInit,
                               Rounds, Blocks> &eng1,
        const AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks>
            &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.key_ == eng2.key_ &&
            eng1.ctr_ == eng2.ctr_;
    }

    friend bool operator!=(const AESNIEngine<ResultType, KeySeq, KeySeqInit,
                               Rounds, Blocks> &eng1,
        const AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks>
            &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng)
    {
        if (!os.good())
            return os;

        os << eng.key_seq_ << ' ';
        os << eng.buffer_ << ' ';
        os << eng.key_ << ' ';
        os << eng.ctr_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng)
    {
        if (!is.good())
            return is;

        AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> eng_tmp;
        is >> std::ws >> eng_tmp.key_seq_;
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.key_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    using state_type = std::array<M128I<>, Blocks>;

    static constexpr std::size_t M_ = Blocks * M128I<ResultType>::size();

    internal::AESNIKeySeqStorage<KeySeq, KeySeqInit, Rounds> key_seq_;
    std::array<ResultType, M_> buffer_;
    key_type key_;
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
        const key_seq_type ks(key_seq_.get(key_));
        enc_first<0>(ks, buf.state, std::true_type());
        enc_round<1>(
            ks, buf.state, std::integral_constant<bool, 1 < Rounds>());
        enc_last<0>(ks, buf.state, std::true_type());
        buffer_ = buf.result;
    }

    template <std::size_t>
    void enc_first(const key_seq_type &, state_type &, std::false_type) const
    {
    }

    template <std::size_t B>
    void enc_first(
        const key_seq_type &ks, state_type &state, std::true_type) const
    {
        std::get<B>(state).value() =
            _mm_xor_si128(std::get<B>(state).value(), ks.front().value());
        enc_first<B + 1>(
            ks, state, std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_round(const key_seq_type &, state_type &, std::false_type) const
    {
    }

    template <std::size_t N>
    void enc_round(
        const key_seq_type &ks, state_type &state, std::true_type) const
    {
        enc_round_block<0, N>(ks, state, std::true_type());
        enc_round<N + 1>(
            ks, state, std::integral_constant<bool, N + 1 < Rounds>());
    }

    template <std::size_t, std::size_t>
    void enc_round_block(
        const key_seq_type &, state_type &, std::false_type) const
    {
    }

    template <std::size_t B, std::size_t N>
    void enc_round_block(
        const key_seq_type &ks, state_type &state, std::true_type) const
    {
        std::get<B>(state).value() = _mm_aesenc_si128(
            std::get<B>(state).value(), std::get<N>(ks).value());
        enc_round_block<B + 1, N>(
            ks, state, std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_last(const key_seq_type &, state_type &, std::false_type) const
    {
    }

    template <std::size_t B>
    void enc_last(
        const key_seq_type &ks, state_type &state, std::true_type) const
    {
        std::get<B>(state).value() = _mm_aesenclast_si128(
            std::get<B>(state).value(), ks.back().value());
        enc_last<B + 1>(
            ks, state, std::integral_constant<bool, B + 1 < Blocks>());
    }
}; // class AESNIEngine

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
