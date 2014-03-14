#ifndef VSMC_RNG_AES_NI_HPP
#define VSMC_RNG_AES_NI_HPP

#include <vsmc/rng/m128i.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_AES_NI_BLOCKS(Blocks) \
    VSMC_STATIC_ASSERT((Blocks > 0), USE_AESNIEngine_WITH_ZERO_BLOCKS)

#define VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((cxx11::is_unsigned<ResultType>::value),              \
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
    typedef StaticVector<__m128i, Rounds + 1> key_seq_type;

    key_seq_type get () const {return key_seq_;}

    void set (const key_type &k)
    {
        KeySeq seq;
        key_ = k;
        seq.generate(key_, key_seq_);
    }

    key_type key () const {return key_;}

    friend inline bool operator== (
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks1,
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks2)
    {return ks1.key_ == ks2.key_;}

    friend inline bool operator!= (
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks1,
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks2)
    {return !(ks1 == ks2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        if (os) os << ks.key_; if (os) os << ' ';
        for (std::size_t i = 0; i != Rounds + 1; ++i) {
            m128i_output(os, ks.key_seq_[i]);
            if (os) os << ' ';
        }

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIKeySeqStorage<KeySeq, true, Rounds> &ks)
    {
        AESNIKeySeqStorage<KeySeq, true, Rounds> ks_tmp;
        if (is) is >> std::ws >> ks_tmp.key_;
        for (std::size_t i = 0; i != Rounds + 1; ++i)
            m128i_input(is, ks_tmp.key_seq_[i]);
        if (is) ks = ks_tmp;

        return is;
    }

    private :

    key_type key_;
    key_seq_type key_seq_;
}; // struct AESNIKeySeqStorage

template <typename KeySeq, std::size_t Rounds>
class AESNIKeySeqStorage<KeySeq, false, Rounds>
{
    public :

    typedef typename KeySeq::key_type key_type;
    typedef StaticVector<__m128i, Rounds + 1> key_seq_type;

    key_seq_type get () const
    {
        key_seq_type ks;
        KeySeq seq;
        seq.generate(key_, ks);

        return ks;
    }

    void set (const key_type &k) {key_ = k;}

    key_type key () const {return key_;}

    friend inline bool operator== (
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &ks1,
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &ks2)
    {return ks1.key_ == ks2.key_;}

    friend inline bool operator!= (
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &ks1,
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &ks2)
    {return !(ks1 == ks2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIKeySeqStorage<KeySeq, false, Rounds> &ks)
    {
        if (os) os << ks.key_; if (os) os << ' ';

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIKeySeqStorage<KeySeq, false, Rounds> &ks)
    {
        AESNIKeySeqStorage<KeySeq, false, Rounds> ks_tmp;
        if (is) is >> std::ws >> ks_tmp.key_;
        if (is) ks = ks_tmp;

        return is;
    }

    private :

    key_type key_;
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
/// \tparam ResultType The output type of `operator()`. Unlike ThreefryEngine
/// and PhiloxEngine, where the `ResultType` and the size of the counter are
/// both specified by the user, this engine only allows specification of the
/// type of output. Internally, it may use any type of counters whose total
/// width is 128-bits. Therefore, increment a counter of type
/// `AESNIEngine::ctr_type` manually is not the same as having the engine
/// increment it internally. The engine only guarantee that each 128-bits
/// output are generated using different counters (until all \f$2^128\f$
/// possible counters are used). The `ctr_type` and `key_type` are only
/// convenient way for user to specify the memory input when generating exact
/// results using the `generate` member functions. For example, one may specify
/// the counter as,
/// ~~~{.cpp}
/// unsigned char input = {
///     0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
///     0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
/// };
/// typedef AES128Engine<unsigned char> eng_type; // A derived class
/// eng_type::ctr_type ctr;
/// eng_type::key_type key;
/// std::memcpy(ctr.data(), input, 16);
/// std::memcpy(key.data(), input, 16);
/// ~~~
/// or
/// ~~~{.cpp}
/// typedef AES128Engine<uint64_t> eng_type;
/// eng_type::ctr_type ctr;
/// eng_type::ctr_type key;
/// ctr[0] = 0x7766554433221100; // Assuming little-endian such as x86
/// ctr[1] = 0xFFEEDDCCBBAA9988;
/// key[0] = 0x7766554433221100;
/// key[1] = 0xFFEEDDCCBBAA9988;
/// ~~~
/// In either case, the following
/// ~~~{.cpp}
/// eng_type eng;
/// eng.key(key);
/// eng_type::buffer_type buffer(eng(ctr));
/// unsigned char output[16];
/// std::memcpy(output, buffer.data(), 16);
/// ~~~
/// will give exactly the same `output` array.
///
/// \tparam KeySeq Using other key schedule can lead to other rng. The `KeySeq`
/// template parameter needs to has a member function of the form,
/// ~~~{.cpp}
/// void generate (const key_type &key, AESNIEngine::key_seq_type &key_seq)
/// ~~~
/// which is similar to that of C++11 `seed_seq`. Given a unique key, a
/// sequence of round keys shall be generated and filled into `key_seq`. The
/// `KeySeq` type also needs to have a member type `key_type`
///
/// \tparam KeySeqInit The key sequence can be computed when the engine is
/// constructed or seeded, or computed each time it is needed. Prepare the key
/// when seeding increase the size of the engine considerably. But in some
/// cases such as AES128Engine, etc, it increase the throughput significantly.
///
/// \tparam Rounds The third template argument is the rounds of the algorithm.
/// AES requires 10 rounds when using a 128-bits key. With reduced strength,
/// any number of round below 10 can be used.
///
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
        sizeof(__m128i) / sizeof(ResultType);

    static VSMC_CONSTEXPR const std::size_t buffer_size_ = K_ * Blocks;

    public :

    typedef ResultType result_type;
    typedef StaticVector<__m128i, Blocks> buffer_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef StaticVector<ctr_type, Blocks> ctr_block_type;
    typedef typename KeySeq::key_type key_type;
    typedef StaticVector<__m128i, Rounds + 1> key_seq_type;

    private :

    typedef StaticVector<uint64_t, 2> ctype;
    typedef StaticVector<ctype, Blocks> cbtype;
    typedef StaticCounter<ctype> counter;

    public :

    explicit AESNIEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        seed(seq);
    }

    AESNIEngine (const ctr_type &c, const key_type &k) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AES_NI;
        ctr_block_type tmp;
        StaticCounter<ctr_type>::set(tmp, c);
        std::memcpy(ctr_block_.data(), tmp.data(), 16 * Blocks);
        key_seq_.set(k);
    }

    void seed (result_type s)
    {
        counter::reset(ctr_block_);
        key_type k;
        k.fill(0);
        k.front() = s;
        key_seq_.set(k);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        counter::reset(ctr_block_);
        key_type k;
        seq.generate(k.begin(), k.end());
        key_seq_.set(k);
        remain_ = 0;
    }

    template <std::size_t B>
    ctr_type ctr () const
    {
        ctr_type tmp;
        std::memcpy(tmp.data(), ctr_block_[Position<B>()].data(), 16);
        return tmp;
    }

    ctr_block_type ctr_block () const
    {
        ctr_block_type tmp;
        std::memcpy(tmp.data(), ctr_block_.data(), 16 * Blocks);
        return tmp;
    }

    key_type key () const {return key_seq_.key();}

    key_seq_type key_seq () const {return key_seq_.get();}

    void ctr (const ctr_type &c)
    {
        ctr_block_type tmp;
        StaticCounter<ctr_type>::set(tmp, c);
        std::memcpy(ctr_block_.data(), tmp.data(), 16 * Blocks);
        remain_ = 0;
    }

    /// \brief Set the block of counters
    ///
    /// \details
    /// Behavior is undefined if `cb.data()` points some place within the
    /// destination object. The class itself does not have any member functions
    /// that return references or pointers to its internal, so unless someone
    /// does something nasty such as the following,
    /// ~~~{.cpp}
    /// typedef /* engine type */ eng_type;
    /// eng_type eng;
    /// const eng_type::ctr_block_type *src = reinterpret_cast<const eng_type::ctr_block_type *>(&eng);
    /// eng.ctr_block(*src);
    /// ~~~
    /// which a sane person should not do, this shall not be a problem.
    void ctr_block (const ctr_block_type &cb)
    {
        std::memcpy(ctr_block_.data(), cb.data(), 16 * Blocks);
        remain_ = 0;
    }

    void key (const key_type &k)
    {
        key_seq_.set(k);
        remain_ = 0;
    }

    /// \brief After reset, next call to `operator()` will always increase the
    /// counter and refresh the buffer
    void reset ()
    {
        counter::reset(ctr_block_);
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            counter::increment(ctr_block_);
            generate_buffer(ctr_block_, buffer_);
            remain_ = buffer_size_;
        }
        --remain_;

        return reinterpret_cast<const ResultType *>(&buffer_)[remain_];
    }

    /// \brief Generate a buffer of random bits given a counter and using the
    /// current key
    buffer_type operator() (const ctr_type &c) const
    {
        ctr_block_type cb;
        StaticCounter<ctr_type>::set(cb, c);
        buffer_type buf;
        generate_buffer(*(reinterpret_cast<const cbtype *>(&cb)), buf);

        return buf;
    }

    /// \brief Generate a buffer of random bits given preloaded buffer and
    /// using the current key
    buffer_type operator() (const ctr_block_type &cb) const
    {
        buffer_type buf;
        generate_buffer(*(reinterpret_cast<const cbtype *>(&cb)), buf);

        return buf;
    }

    void discard (result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (n <= remain_) {
            remain_ -= n;
            return;
        }

        n -= remain_;
        if (n <= buffer_size_) {
            remain_ = 0;
            operator()();
            remain_ = buffer_size_ - n;
            return;
        }

        remain_ = 0;
        counter::increment(ctr_block_,
                static_cast<result_type>(n / buffer_size_));
        operator()();
        remain_ = buffer_size_ - n % buffer_size_;
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
        return
            eng1.ctr_block_ == eng2.ctr_block_ &&
            eng1.buffer_ == eng2.buffer_ &&
            eng1.key_seq_ == eng2.key_seq_ &&
            eng1.remain_ == eng2.remain_;
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
        for (std::size_t i = 0; i != Blocks; ++i) {
            m128i_output(os, eng.buffer_[i]);
            if (os) os << ' ';
        }
        if (os) os << eng.ctr_block_ << ' ';
        if (os) os << eng.key_seq_ << ' ';
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> &eng)
    {
        AESNIEngine<ResultType, KeySeq, KeySeqInit, Rounds, Blocks> eng_tmp;
        for (std::size_t i = 0; i != Blocks; ++i)
            m128i_input(is, eng_tmp.buffer_[i]);
        if (is) is >> std::ws >> eng_tmp.ctr_block_;
        if (is) is >> std::ws >> eng_tmp.key_seq_;
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    private :

    // FIXME
    // buffer_ is automatically 16 bytes aligned
    // Thus, we assume that ctr_block_ and buffer_ will also be 16 bytes
    // alinged

    buffer_type buffer_;
    cbtype ctr_block_;
    internal::AESNIKeySeqStorage<KeySeq, KeySeqInit, Rounds> key_seq_;
    std::size_t remain_;

    void generate_buffer (const cbtype &cb, buffer_type &buf) const
    {
        key_seq_type ks(key_seq_.get());
        pack(cb, buf);
        enc_first<0>(ks, buf, cxx11::true_type());
        enc_round<1>(ks, buf, cxx11::integral_constant<bool, 1 < Rounds>());
        enc_last <0>(ks, buf, cxx11::true_type());
    }

    template <std::size_t>
    void enc_first (const key_seq_type &, buffer_type &,
            cxx11::false_type) const {}

    template <std::size_t B>
    void enc_first (const key_seq_type &ks, buffer_type &buf,
            cxx11::true_type) const
    {
        buf[Position<B>()] = _mm_xor_si128(buf[Position<B>()], ks.front());
        enc_first<B + 1>(ks, buf,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_round (const key_seq_type &, buffer_type &,
            cxx11::false_type) const {}

    template <std::size_t N>
    void enc_round (const key_seq_type &ks, buffer_type &buf,
            cxx11::true_type) const
    {
        enc_round_block<0, N>(ks, buf, cxx11::true_type());
        enc_round<N + 1>(ks, buf,
                cxx11::integral_constant<bool, N  + 1 < Rounds>());
    }

    template <std::size_t, std::size_t>
    void enc_round_block (const key_seq_type &, buffer_type &,
            cxx11::false_type) const {}

    template <std::size_t B, std::size_t N>
    void enc_round_block (const key_seq_type &ks, buffer_type &buf,
            cxx11::true_type) const
    {
        buf[Position<B>()] = _mm_aesenc_si128(
                buf[Position<B>()], ks[Position<N>()]);
        enc_round_block<B + 1, N>(ks, buf,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void enc_last (const key_seq_type &, buffer_type &,
            cxx11::false_type) const {}

    template <std::size_t B>
    void enc_last (const key_seq_type &ks, buffer_type &buf,
            cxx11::true_type) const
    {
        buf[Position<B>()] = _mm_aesenclast_si128(
                buf[Position<B>()], ks.back());
        enc_last<B + 1>(ks, buf,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    void pack (const cbtype &cb, buffer_type &buf) const
    {
        is_m128_aligned(cb.data()) ?
            pack_a<0>(cb, buf, cxx11::true_type()):
            pack_u<0>(cb, buf, cxx11::true_type());
    }

    template <std::size_t>
    void pack_a (const cbtype &, buffer_type &, cxx11::false_type) const {}

    template <std::size_t B>
    void pack_a (const cbtype &cb, buffer_type &buf, cxx11::true_type) const
    {
        m128i_pack_a<0>(cb[Position<B>()], buf[Position<B>()]);
        pack_a<B + 1>(cb, buf,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void pack_u (const cbtype &, buffer_type &, cxx11::false_type) const {}

    template <std::size_t B>
    void pack_u (const cbtype &cb, buffer_type &buf, cxx11::true_type) const
    {
        m128i_pack_u<0>(cb[Position<B>()], buf[Position<B>()]);
        pack_u<B + 1>(cb, buf,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }
}; // class AESNIEngine

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
