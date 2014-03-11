#ifndef VSMC_RNG_AES_NI_HPP
#define VSMC_RNG_AES_NI_HPP

#include <vsmc/rng/m128i.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_AESNIEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_AES_NI \
    VSMC_STATIC_ASSERT_RNG_AES_NI_RESULT_TYPE(ResultType);

namespace vsmc {

/// \brief RNG engine using AES-NI instructions
/// \ingroup AESNIRNG
///
/// \details
/// Two dervied class AESEngine and ARSEngine behaves exactly the same as AES
/// and ARS RNG engines as desribed in
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
/// using 64-bits integers as output. Second, it allows user defined key
/// schedule (the second template argument). The two derived classed merely use
/// two special key schedule to reproduce the original behavior.
///
/// Using other key schedule can lead to other rng. The `KeySeq` template
/// parameter only need to has a memeber function of the form,
/// ~~~{.cpp}
/// void generate (const key_type &key, ARSEngine::key_seq_type &key_seq)
/// ~~~
/// which is similar to that of C++11 `seed_seq`. Given a unique key, a
/// sequence of keys shall be generated. The default, `ARSKeySeq` use a Weyl
/// sequence.
///
/// The third template argument, `Blocks` specify how many blocks shall be
/// used. The AES-NI instructions have noticeable latency but can be started
/// every two cycles. By allowing generating multiple blocks at once, and
/// interleaving the instructions, the throughput can be increased at the cost
/// of space. The default blocks, as in `ARS4x32` is defined by the macro
/// `VSMC_RNG_ARS_BLOCKS`
///
/// The fourth template argument, `R` is the rounds of the algorithm. AES
/// requires 10 rounds when using a 128-bits key. With reduced strength, any
/// number of round below 10 can be used.
template <typename ResultType, typename KeySeq,
         std::size_t R, std::size_t Blocks, typename KeyType =
             StaticVector<ResultType, sizeof(__m128i) / sizeof(ResultType)>
         >
class AESNIEngine
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType);

    static VSMC_CONSTEXPR const std::size_t buffer_size_ = K_ * Blocks;

    typedef internal::RngCounter<ResultType> counter;

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef StaticVector<ResultType, K_> key_type;
    typedef StaticVector<__m128i, R + 1> key_seq_type;

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

    void seed (result_type s)
    {
        counter::reset(ctr_);
        key_.fill(0);
        key_.front() = s;
        key_seq_init();
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        counter::reset(ctr_);
        seq.generate(key_.begin(), key_.end());
        key_seq_init();
        remain_ = 0;
    }

    const ctr_type &ctr () const {return counter::get(ctr_);}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        counter::set(ctr_, c);
        remain_ = 0;
    }

    void key (key_type k)
    {
        key_ = k;
        key_seq_init();
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            counter::increment(ctr_);
            pack();
            generate<1>(cxx11::integral_constant<bool, 1 < R>());
            unpack();
            remain_ = buffer_size_;
        }
        --remain_;

        return buffer_[remain_];
    }

    void discard (std::size_t nskip)
    {
        if (nskip <= remain_) {
            remain_ -= nskip;
            return;
        }

        nskip -= remain_;
        if (nskip <= buffer_size_) {
            remain_ = 0;
            operator()();
            remain_ = buffer_size_ - nskip;
            return;
        }

        remain_ = 0;
        counter::increment(ctr_, nskip / buffer_size_);
        operator()();
        remain_ = buffer_size_ - nskip % buffer_size_;
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const AESNIEngine<ResultType, KeySeq, R, Blocks> &eng1,
            const AESNIEngine<ResultType, KeySeq, R, Blocks> &eng2)
    {
        if (eng1.ctr_ != eng2.ctr_)
            return false;

        if (eng1.buffer_ != eng2.buffer_)
            return false;

        for (std::size_t i = 0; i != key_type::size(); ++i)
            if (!m128i_is_equal(eng1.key_[i], eng2.key_[i]))
                return false;

        return eng1.remain_ == eng2.remain_;
    }

    friend inline bool operator!= (
            const AESNIEngine<ResultType, KeySeq, R, Blocks> &eng1,
            const AESNIEngine<ResultType, KeySeq, R, Blocks> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIEngine<ResultType, KeySeq, R, Blocks> &eng)
    {
        if (os) os << eng.ctr_ << ' ';
        if (os) os << eng.buffer_ << ' ';
        if (os) os << eng.key_ << ' ';
        for (std::size_t i = 0; i != key_seq_type::size(); ++i) {
            m128i_output(os, eng.key_seq_[i]);
            if (os) os << ' ';
        }
        for (std::size_t i = 0; i != Blocks; ++i) {
            m128i_output(os, eng.pac_[i]);
            if (os) os << ' ';
        }
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIEngine<ResultType, KeySeq, R, Blocks> &eng)
    {
        AESNIEngine eng_tmp;
        if (is) is >> std::ws >> eng_tmp.ctr_;
        if (is) is >> std::ws >> eng_tmp.buffer_;
        if (is) is >> std::ws >> eng_tmp.key_;
        for (std::size_t i = 0; i != key_seq_type::size(); ++i)
            m128i_input(is, eng_tmp.key_seq_[i]);
        for (std::size_t i = 0; i != Blocks; ++i)
            m128i_input(is, eng_tmp.pac_[i]);
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    private :

    StaticVector<ctr_type, Blocks> ctr_;
    StaticVector<ResultType, buffer_size_> buffer_;
    key_type key_;
    StaticVector<__m128i, Blocks> pac_;
    key_seq_type key_seq_;
    std::size_t remain_;

    void key_seq_init ()
    {
        KeySeq seq;
        seq.generate(key_, key_seq_);
    }

    void pack ()
    {
        m128i_pack<0>(ctr_.front(), pac_.front());
        pack<1>(cxx11::integral_constant<bool, 1 < Blocks>());

        pac_.front() = _mm_xor_si128(pac_.front(), key_seq_.front());
        pac_xor<1>(cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t> void pack (cxx11::false_type) {}

    template <std::size_t B>
    void pack (cxx11::true_type)
    {
        m128i_pack<0>(ctr_[Position<B>()], pac_[Position<B>()]);
        pack<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t> void pac_xor (cxx11::false_type) {}

    template <std::size_t B>
    void pac_xor (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_xor_si128(
                pac_[Position<B>()], key_seq_.front());
        pac_xor<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t> void unpack (cxx11::false_type) {}

    void unpack () {unpack<0>(cxx11::integral_constant<bool, 0 < Blocks>());}

    template <std::size_t B>
    void unpack (cxx11::true_type)
    {
        m128i_unpack<B * K_>(pac_[Position<B>()], buffer_);
        unpack<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    void generate (cxx11::false_type)
    {generate_last<0>(cxx11::integral_constant<bool, 0 < Blocks>());}

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        generate_step<0, N>(cxx11::integral_constant<bool, 0 < Blocks>());
        generate<N + 1>(cxx11::integral_constant<bool, N  + 1 < R>());
    }

    template <std::size_t, std::size_t>
    void generate_step (cxx11::false_type) {}

    template <std::size_t B, std::size_t N>
    void generate_step (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_aesenc_si128(
                pac_[Position<B>()], key_seq_[Position<N>()]);
        generate_step<B + 1, N>(
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t> void generate_last (cxx11::false_type) {}

    template <std::size_t B>
    void generate_last (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_aesenclast_si128(
                pac_[Position<B>()], key_seq_.back());
        generate_last<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }
}; // class AESNIEngine

} // namespace vsmc

#endif // VSMC_RNG_AES_NI_HPP
