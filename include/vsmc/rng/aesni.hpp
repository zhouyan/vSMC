#ifndef VSMC_RNG_AESNI_HPP
#define VSMC_RNG_AESNI_HPP

#include <vsmc/rng/m128i.hpp>

#define VSMC_STATIC_ASSERT_RNG_AESNI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_AESNIEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_AESNI_BLOCKS(Blocks) \
    VSMC_STATIC_ASSERT((Blocks > 0), USE_AESNIEngine_WITH_ZERO_BLOCKS)

#define VSMC_STATIC_ASSERT_RNG_AESNI \
    VSMC_STATIC_ASSERT_RNG_AESNI_RESULT_TYPE(ResultType);                    \
    VSMC_STATIC_ASSERT_RNG_AESNI_BLOCKS(Blocks);

#define VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(N, val) \
    template <> struct AESNIRoundConstant< N > :                             \
        public cxx11::integral_constant<int, val > {};

namespace vsmc {

namespace internal {

template <std::size_t N> struct AESNIRoundConstant;

VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(0, 0x01)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(1, 0x02)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(2, 0x04)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(3, 0x08)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(4, 0x10)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(5, 0x20)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(6, 0x40)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(7, 0x80)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(8, 0x1B)
VSMC_DEFINE_RNG_AESNI_ROUND_CONSTANT(9, 0x36)

} // namespace vsmc::internal

/// \brief AESNI RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm AES as described in [Parallel
/// Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// The implementation is almost identical to the original. Compared to
/// `r123:Engine<r123::AESNI4x32>`, when using the default constructor or the
/// one with a single seed, the output shall be exactly the same for the first
/// \f$2^32\f$ iterations. Further iterations may produce different results, as
/// vSMC increment the counter slightly differently, but it still cover the
/// same range and has the same period as the original. In addition, this
/// engine allows output of 64-bits integers.
template <typename ResultType, std::size_t Blocks = 1>
class AESNIEngine
{
    static VSMC_CONSTEXPR const std::size_t R_ = 10;
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType);

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef StaticVector<__m128i, R_ + 1> key_type;
    typedef StaticVector<ResultType, K_> ukey_type;

    explicit AESNIEngine (result_type s = 0) :
        tmp0_(), tmp1_(), tmp2_(), remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AESNI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : tmp0_(), tmp1_(), tmp2_(), remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AESNI;
        seed(seq);
    }

    void seed (result_type s)
    {
        ctr_type c;
        c.fill(0);
        ctr_.fill(c);
        ukey_type uk;
        uk.fill(0);
        uk.front() = s;
        ukey(uk);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        ctr_type c;
        c.fill(0);
        ctr_.fill(c);
        ukey_type uk;
        seq.generate(uk.begin(), uk.end());
        ukey(uk);
        remain_ = 0;
    }

    const ctr_type &ctr () const {return ctr_;}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        ctr_.back() = c;
        remain_ = 0;
    }

    void key (const key_type &k)
    {
        key_ = k;
        remain_ = 0;
    }

    void ukey (const ukey_type &uk)
    {
        internal::pack(uk, pac_.front());
        init_key(pac_.front());
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ == 0)
            generate();
        --remain_;

        return result<Blocks - 1>(
                cxx11::integral_constant<bool, 1 < Blocks>());
    }

    /// \brief Discard results
    ///
    /// \details
    /// The the behavior is slightly different from that in C++11 standard.
    /// Calling `discard(nskip)` is not equivalent to call `operator()` `nskip`
    /// times. Instead, it ensures that at least `nskip` results are discarded.
    /// There may be a few more than `nskip` also discarded.
    void discard (std::size_t nskip)
    {
        if (nskip == 0)
            return;

        remain_ = 0;
        if (nksip < Blocks)
            return;

        nskip -= Blocks;
        internal::RngCounter<ResultType, K_>::increment(ctr_.front(), nskip);
        increment();
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const AESNIEngine<ResultType> &eng1,
            const AESNIEngine<ResultType> &eng2)
    {
        if (eng1.ctr_ != eng2.ctr_)
            return false;

        if (eng1.res_ != eng2.res_)
            return false;

        for (std::size_t i = 0; i != key_type::size(); ++i)
            if (!internal::is_equal(eng1.key_[i], eng2.key_[i]))
                return false;

        return eng1.remain_ == eng2.remain_;
    }

    friend inline bool operator!= (
            const AESNIEngine<ResultType> &eng1,
            const AESNIEngine<ResultType> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const AESNIEngine<ResultType> &eng)
    {
        if (os) os << eng.ctr_ << ' ';
        if (os) os << eng.res_ << ' ';
        for (std::size_t i = 0; i != key_type::size(); ++i) {
            internal::output_m128i(os, eng.key_[i]);
            if (os) os << ' ';
        }
        internal::output_m128i(os, eng.pac_);  if (os) os << ' ';
        internal::output_m128i(os, eng.tmp0_); if (os) os << ' ';
        internal::output_m128i(os, eng.tmp1_); if (os) os << ' ';
        internal::output_m128i(os, eng.tmp2_); if (os) os << ' ';
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            AESNIEngine<ResultType> &eng)
    {
        AESNIEngine eng_tmp;
        if (is) is >> std::ws >> eng_tmp.ctr_;
        if (is) is >> std::ws >> eng_tmp.res_;
        for (std::size_t i = 0; i != key_type::size(); ++i)
            internal::input_m128i(is, eng_tmp.key_[i]);
        internal::input_m128i(is, eng_tmp.pac_);
        internal::input_m128i(is, eng_tmp.tmp0_);
        internal::input_m128i(is, eng_tmp.tmp1_);
        internal::input_m128i(is, eng_tmp.tmp2_);
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    private :

    StaticVector<ctr_type, Blocks> ctr_;
    StaticVector<ctr_type, Blocks> res_;
    key_type key_;
    StaticVector<__m128i, Blocks> pac_;
    __m128i tmp0_;
    __m128i tmp1_;
    __m128i tmp2_;
    std::size_t remain_;

    void increment ()
    {
        ctr_.front() = ctr_.back();
        internal::RngCounter<ResultType, K_>::increment(ctr_.front());
        increment<1>(cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t> void increment (cxx11::false_type) {}

    template <std::size_t B>
    void increment (cxx11::true_type)
    {
        ctr_[Position<B>()] = ctr_[Position<B - 1>()];
        internal::RngCounter<ResultType, K_>::increment(ctr_[Position<B>()]);
        increment<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    void pack ()
    {
        internal::pack(ctr_.front(), pac_.front());
        pack<1>(cxx11::integral_constant<bool, 1 < Blocks>());
        pac_.front() = _mm_xor_si128(pac_.front(), key_.front());
        pac_xor<1>(cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t> void pack (cxx11::false_type) {}

    template <std::size_t B>
    void pack (cxx11::true_type)
    {
        internal::pack(ctr_[Position<B>()], pac_[Position<B>()]);
        pack<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t> void pac_xor (cxx11::false_type) {}

    template <std::size_t B>
    void pac_xor (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_xor_si128(pac_[Position<B>()], key_.front());
        pac_xor<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t> void unpack (cxx11::false_type) {}

    void unpack ()
    {
        internal::unpack(pac_.front(), res_.front());
        unpack<1>(cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t B>
    void unpack (cxx11::true_type)
    {
        internal::unpack(pac_[Position<B>()], res_[Position<B>()]);
        unpack<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    void generate ()
    {
        increment();
        pack();
        generate<0>(cxx11::true_type());
        unpack();
        remain_ = K_ * Blocks;
    }

    template <std::size_t>
    void generate (cxx11::false_type)
    {
        pac_.front() = _mm_aesenclast_si128(pac_.front(), key_.back());
        generate_last<1>(cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t> void generate_last (cxx11::false_type) {}

    template <std::size_t B>
    void generate_last (cxx11::true_type)
    {
        pac_[Position<B>()] =
            _mm_aesenclast_si128(pac_[Position<B>()], key_.back());
        generate_last<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        pac_.front() = _mm_aesenc_si128(pac_.front(), key_[Position<N + 1>()]);
        generate_step<1, N>(cxx11::integral_constant<bool, 1 < Blocks>());
        generate<N + 1>(cxx11::integral_constant<bool, N + 2 < R_>());
    }

    template <std::size_t, std::size_t>
    void generate_step (cxx11::false_type) {}

    template <std::size_t B, std::size_t N>
    void generate_step (cxx11::true_type)
    {
        pac_[Position<B>()] =
            _mm_aesenc_si128(pac_[Position<B>()], key_[Position<N + 1>()]);
        generate_step<B + 1, N>(
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t>
    result_type result (cxx11::false_type)
    {return res_.front()[remain_];}

    template <std::size_t R>
    result_type result (cxx11::true_type)
    {
        if (remain_ > R * K_)
            return res_[Position<R>()][remain_ - R * K_];
        return result<R - 1>(cxx11::integral_constant<bool, 0 < R>());
    }

    void init_key(__m128i k)
    {
        tmp0_ = k;
        init_key<0>(cxx11::true_type());
    }

    template <std::size_t>
    void init_key (cxx11::false_type) {key_.back() = tmp0_;}

    template <std::size_t N>
    void init_key (cxx11::true_type)
    {
        key_[Position<N>()] = tmp0_;
        tmp1_ = _mm_aeskeygenassist_si128(tmp0_,
                internal::AESNIRoundConstant<N>::value);
        init_key_assit();
        init_key<N + 1>(cxx11::integral_constant<bool, N + 1 < R_>());
    }

    void init_key_assit ()
    {
        tmp1_ = _mm_shuffle_epi32 (tmp1_ ,0xFF);
        tmp2_ = _mm_slli_si128    (tmp0_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp2_ = _mm_slli_si128    (tmp2_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp2_ = _mm_slli_si128    (tmp2_, 0x04);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp2_);
        tmp0_ = _mm_xor_si128     (tmp0_, tmp1_);
    }
}; // class AESNIEngine

/// \brief AESNI RNG engine returning 32-bits integers with default blocks
/// \ingroup R123RNG
typedef AESNIEngine<uint32_t> AESNI4x32;

/// \brief AESNI RNG engine returning 64-bits integers with default blocks
/// \ingroup R123RNG
typedef AESNIEngine<uint64_t> AESNI2x64;

/// \brief AESNI RNG engine returning 128-bits integers with default blocks
/// \ingroup R123RNG
typedef AESNIEngine<__m128i>  AESNI1x128;

/// \brief AESNI RNG engine returning 32-bits integers with 1 block
/// \ingroup R123RNG
typedef AESNIEngine<uint32_t, 1> AESNI4x32_1;

/// \brief AESNI RNG engine returning 64-bits integers with 1 block
/// \ingroup R123RNG
typedef AESNIEngine<uint64_t, 1> AESNI2x64_1;

/// \brief AESNI RNG engine returning 128-bits integers with 1 block
/// \ingroup R123RNG
typedef AESNIEngine<__m128i, 1>  AESNI1x128_1;

/// \brief AESNI RNG engine returning 32-bits integers with 2 block
/// \ingroup R123RNG
typedef AESNIEngine<uint32_t, 2> AESNI4x32_2;

/// \brief AESNI RNG engine returning 64-bits integers with 2 block
/// \ingroup R123RNG
typedef AESNIEngine<uint64_t, 2> AESNI2x64_2;

/// \brief AESNI RNG engine returning 128-bits integers with 2 block
/// \ingroup R123RNG
typedef AESNIEngine<__m128i, 2>  AESNI1x128_2;

/// \brief AESNI RNG engine returning 32-bits integers with 4 block
/// \ingroup R123RNG
typedef AESNIEngine<uint32_t, 4> AESNI4x32_4;

/// \brief AESNI RNG engine returning 64-bits integers with 4 block
/// \ingroup R123RNG
typedef AESNIEngine<uint64_t, 4> AESNI2x64_4;

/// \brief AESNI RNG engine returning 128-bits integers with 4 block
/// \ingroup R123RNG
typedef AESNIEngine<__m128i, 4>  AESNI1x128_4;

} // namespace vsmc

#endif // VSMC_RNG_AESNI_HPP
