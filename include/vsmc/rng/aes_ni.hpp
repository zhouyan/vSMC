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
#include <vsmc/rng/m128i.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_AES_NI_BLOCKS(Blocks) \
    VSMC_STATIC_ASSERT((Blocks > 0), USE_AESNIEngine_WITH_ZERO_BLOCKS)

#define VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((std::is_unsigned<ResultType>::value),              \
            USE_AESNIEngine_WITH_RESULT_TYPE_NOT_AN_UNSIGNED_INTEGER)

#define VSMC_STATIC_ASSERT_RNG_AES_NI \
    VSMC_STATIC_ASSERT_RNG_AES_NI_BLOCKS(Blocks);                            \
    VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType);

namespace vsmc {

namespace internal {

template <typename KeySeq, bool KeySeqInit, std::size_t Rounds>
class AESNIKeySeqStorage;

template <typename KeySeq, std::size_t Rounds>
class AESNIKeySeqStorage<KeySeq, true, Rounds>
{
    public :

    typedef typename KeySeq::key_type key_type;
    typedef Array<__m128i, Rounds + 1> key_seq_type;

    key_seq_type get (const key_type &) const {return key_seq_;}

    void set (const key_type &k)
    {
        KeySeq seq;
        seq.generate(k, key_seq_);
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        if (!os.good())
            return os;

        for (std::size_t i = 0; i != Rounds + 1; ++i) {
            m128i_output(os, ks.key_seq_[i]);
            os << ' ';
        }

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        if (!is.good())
            return is;

        AESNIKeySeqStorage<KeySeq, true, Rounds> ks_tmp;
        for (std::size_t i = 0; i != Rounds + 1; ++i)
            m128i_input(is, ks_tmp.key_seq_[i]);

        if (is.good()) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            ks = std::move(ks_tmp);
#else
            ks = ks_tmp;
#endif
        }

        return is;
    }

    private :

    key_seq_type key_seq_;
}; // struct AESNIKeySeqStorage

template <typename KeySeq, std::size_t Rounds>
class AESNIKeySeqStorage<KeySeq, false, Rounds>
{
    public :

    typedef typename KeySeq::key_type key_type;
    typedef Array<__m128i, Rounds + 1> key_seq_type;

    key_seq_type get (const key_type &k) const
    {
        key_seq_type ks;
        KeySeq seq;
        seq.generate(k, ks);

        return ks;
    }

    void set (const key_type &) {}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &) {return os;}

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIKeySeqStorage<KeySeq, false, Rounds> &) {return is;}
}; // struct AESNIKeySeqStorage

} // namespace vsmc::internal

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
///
/// \details
/// Two dervied class AES128Engine and ARSEngine behave exactly the same as
/// AES and ARS RNG engines as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib], when used with `uint32_t` as ResultType. The first
/// \f$2^{32}\f$ iterations will be exactly the same as
/// `r123::Engine<r123:AESNI4x32>` and `r123::Engine<r123:ARS4x32_R<10> >`.
/// (Note, they could be simple template alias in C++11, but to support C++98
/// we had to derive from it. Since the derived classes contains nothing and
/// does nothing, there is no performance cost).
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// This implementation is more flexible than the original. First, it allows
/// using any unsigned integers as output. Second, it allows user defined key
/// schedule (the second template argument). The two derived classed merely use
/// two special key schedule to reproduce the original behavior.
///
/// The terminology used in this engine is slightly different than that in
/// [Random123][r123lib]. In the later, there is a distinction between
/// `key_type` and `ukey_type`. They are `key_type` and `key_seq_type` in this
/// class, respectively. In other [Random123][r123lib] engines, `key_type` and
/// `ukey_type` are usually the same. And in ThreefryEngine and PhiloxEngine,
/// vSMC does not define `ukey_type`. In this engine, the term `key` (`ukey`,
/// short for unique key, in [Rnadom123][r123lib]) usually refer to the 128-,
/// 192-, or 256-bits input key in the context of the AES algorithm. And
/// `key_seq` (`key` in [Random123][r123lib]) refers to the key schedule in the
/// same context. In the context of C++11, `key_seq` is also sort of like the
/// `seed_seq` for other RNG (basically both expand a single input value into a
/// sequence such that the sequence provides extra entropy even the inputs are
/// not random at all). Therefore vSMC's terminology shall be both familiar to
/// people familiar with block ciphers (of which AES is one), and people
/// familiar with C++11, but at the risk of confusing people already familiar
/// with [Random123[r123lib]. Of course, if only used as a standard C++11
/// engine, then none of these matters since users only provides seeds or a
/// seed sequence.
///
/// \tparam ResultType The output type of `operator()`
/// \tparam KeySeq Using other key schedule can lead to other rng. The `KeySeq`
/// template parameter needs to has a member function of the form,
/// ~~~{.cpp}
/// void generate (const key_type &key, AESNIEngine::key_seq_type &key_seq)
/// ~~~
/// which is similar to that of C++11 `seed_seq`. Given a unique key, a
/// sequence of round keys shall be generated and filled into `key_seq`. The
/// `KeySeq` type also needs to have a member type `key_type`.
/// \tparam KeySeqInit The key sequence can be computed when the engine is
/// constructed or seeded, or computed each time it is needed. Prepare the key
/// when seeding increase the size of the engine considerably. But in some
/// cases such as AES128Engine, etc, it increase the throughput significantly.
/// \tparam Rounds The third template argument is the rounds of the algorithm.
/// AES requires 10 rounds when using a 128-bits key. With reduced strength,
/// any number of round below 10 can be used.
/// \tparam Blocks The fourth template argument specifies how many blocks shall
/// be used. The AES-NI instructions have noticeable latency but can be started
/// every two cycles. By allowing generating multiple blocks at once, and
/// interleaving the instructions, the throughput can be increased at the cost
/// of space.
template <typename ResultType, typename KeySeq, bool KeySeqInit,
         std::size_t Rounds, std::size_t Blocks>
class AESNIEngine
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType) * Blocks;

    public :

    typedef ResultType result_type;
    typedef Array<__m128i, Blocks> buffer_type;
    typedef Array<ResultType, sizeof(__m128i) / sizeof(ResultType)> ctr_type;
    typedef Array<ctr_type, Blocks> ctr_block_type;
    typedef typename KeySeq::key_type key_type;
    typedef Array<__m128i, Rounds + 1> key_seq_type;

    private :

    typedef Counter<ctr_type> counter;

    public :

    explicit AESNIEngine (result_type s = 0) : index_(K_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine (SeedSeq &seq, typename std::enable_if<
            internal::is_seed_seq<SeedSeq, result_type, key_type,
            AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks>
            >::value>::type * = VSMC_NULLPTR) : index_(K_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(seq);
    }

    AESNIEngine (const key_type &k) : index_(K_)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(k);
    }

    void seed (result_type s)
    {
        counter::reset(ctr_block_);
        key_.fill(0);
        key_.front() = s;
        key_seq_.set(key_);
        index_ = K_;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<
            SeedSeq, result_type, key_type>:: value>::type * = VSMC_NULLPTR)
    {
        counter::reset(ctr_block_);
        seq.generate(key_.begin(), key_.end());
        key_seq_.set(key_);
        index_ = K_;
    }

    void seed (const key_type &k)
    {
        counter::reset(ctr_block_);
        key_ = k;
        key_seq_.set(k);
        index_ = K_;
    }

    template <std::size_t B>
    ctr_type ctr () const {return ctr_block_[Position<B>()];}

    ctr_block_type ctr_block () const {return ctr_block_;}

    key_type key () const {return key_;}

    key_seq_type key_seq () const {return key_seq_.get(key_);}

    void ctr (const ctr_type &c)
    {
        counter::set(ctr_block_, c);
        index_ = K_;
    }

    void key (const key_type &k)
    {
        key_ = k;
        key_seq_.set(k);
        index_ = K_;
    }

    result_type operator() ()
    {
        if (index_ == K_) {
            counter::increment(ctr_block_);
            generate_buffer(ctr_block_, buffer_);
            index_ = 0;
        }

        return reinterpret_cast<const result_type *>(buffer_.data())[index_++];
    }

    /// \brief Generate a buffer of random bits given a counter using the
    /// current key
    buffer_type operator() (const ctr_type &c) const
    {
        ctr_block_type cb;
        counter::set(cb, c);
        buffer_type buf;
        generate_buffer(cb, buf);

        return buf;
    }

    /// \brief Generate a buffer of random bits given a block of counters using
    /// the current key
    buffer_type operator() (const ctr_block_type &cb) const
    {
        buffer_type buf;
        generate_buffer(cb, buf);

        return buf;
    }

    /// \brief Generate random bits in a pre-allocated buffer given a counter
    /// using the current key
    void operator() (const ctr_type &c, buffer_type &buf) const
    {
        ctr_block_type cb;
        counter::set(cb, c);
        generate_buffer(cb, buf);
    }

    /// \brief Generate ranodm bits in a pre-allocated buffer given a block of
    /// counters using the current key
    void operator() (const ctr_block_type &cb, buffer_type &buf) const
    {generate_buffer(cb, buf);}

    void discard (result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= K_) {
            index_ += n;
            return;
        }

        n -= K_ - index_;
        if (n <= K_) {
            index_ = K_;
            operator()();
            index_ = n;
            return;
        }

        counter::increment(ctr_block_, static_cast<result_type>(n / K_));
        index_ = K_;
        operator()();
        index_ = n % K_;
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const AESNIEngine<
            ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng1,
            const AESNIEngine<
            ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng2)
    {
        return eng1.index_ == eng2.index_ &&
            eng1.key_ == eng2.key_ &&
            eng1.ctr_block_ == eng2.ctr_block_;
    }

    friend inline bool operator!= (
            const AESNIEngine<
            ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng1,
            const AESNIEngine<
            ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIEngine<
            ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng)
    {
        if (!os.good())
            return os;

        for (std::size_t i = 0; i != Blocks; ++i) {
            m128i_output(os, eng.buffer_[i]);
            os << ' ';
        }
        os << eng.ctr_block_ << ' ';
        os << eng.key_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng)
    {
        if (!is.good())
            return is;

        AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> eng_tmp;
        for (std::size_t i = 0; i != Blocks; ++i)
            m128i_input(is, eng_tmp.buffer_[i]);
        is >> std::ws >> eng_tmp.ctr_block_;
        is >> std::ws >> eng_tmp.key_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good()) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng = std::move(eng_tmp);
#else
            eng = eng_tmp;
#endif
        }

        return is;
    }

    private :

    // FIXME
    // buffer_ is automatically 16 bytes aligned
    // Thus, we assume that ctr_block_ and buffer_ will also be 16 bytes
    // alinged

    buffer_type buffer_;
    internal::AESNIKeySeqStorage<KeySeq, KeySeqInit, Rounds> key_seq_;
    ctr_block_type ctr_block_;
    key_type key_;
    std::size_t index_;

    void generate_buffer (const ctr_block_type &cb,
            buffer_type &buf) const
    {
        const key_seq_type ks(key_seq_.get(key_));
        pack(cb, buf);
        enc_first<0>(ks, buf, std::true_type());
        enc_round<1>(ks, buf, std::integral_constant<bool, 1 < Rounds>());
        enc_last <0>(ks, buf, std::true_type());
    }

    template <std::size_t>
    void enc_first (const key_seq_type &, buffer_type &,
            std::false_type) const {}

    template <std::size_t B>
    void enc_first (const key_seq_type &ks, buffer_type &buf,
            std::true_type) const
    {
        buf[Position<B>()] = _mm_xor_si128(buf[Position<B>()], ks.front());
        enc_first<B + 1>(ks, buf,
                std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_round (const key_seq_type &, buffer_type &,
            std::false_type) const {}

    template <std::size_t N>
    void enc_round (const key_seq_type &ks, buffer_type &buf,
            std::true_type) const
    {
        enc_round_block<0, N>(ks, buf, std::true_type());
        enc_round<N + 1>(ks, buf,
                std::integral_constant<bool, N  + 1 < Rounds>());
    }

    template <std::size_t, std::size_t>
    void enc_round_block (const key_seq_type &, buffer_type &,
            std::false_type) const {}

    template <std::size_t B, std::size_t N>
    void enc_round_block (const key_seq_type &ks, buffer_type &buf,
            std::true_type) const
    {
        buf[Position<B>()] = _mm_aesenc_si128(
                buf[Position<B>()], ks[Position<N>()]);
        enc_round_block<B + 1, N>(ks, buf,
                std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_last (const key_seq_type &, buffer_type &,
            std::false_type) const {}

    template <std::size_t B>
    void enc_last (const key_seq_type &ks, buffer_type &buf,
            std::true_type) const
    {
        buf[Position<B>()] = _mm_aesenclast_si128(
                buf[Position<B>()], ks.back());
        enc_last<B + 1>(ks, buf,
                std::integral_constant<bool, B + 1 < Blocks>());
    }

    void pack (const ctr_block_type &cb, buffer_type &buf) const
    {pack_ctr<0>(cb, buf, std::true_type());}

    template <std::size_t>
    void pack_ctr (const ctr_block_type &, buffer_type &,
            std::false_type) const {}

    template <std::size_t B>
    void pack_ctr (const ctr_block_type &cb, buffer_type &buf,
            std::true_type) const
    {
        m128i_pack<0>(cb[Position<B>()], buf[Position<B>()]);
        pack_ctr<B + 1>(cb, buf,
                std::integral_constant<bool, B + 1 < Blocks>());
    }
}; // class AESNIEngine

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
