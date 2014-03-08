#ifndef VSMC_RNG_AESNI_HPP
#define VSMC_RNG_AESNI_HPP

#include <vsmc/rng/m128i.hpp>

#define VSMC_STATIC_ASSERT_RNG_AESNI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_AESNIEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_AESNI \
    VSMC_STATIC_ASSERT_RNG_AESNI_RESULT_TYPE(ResultType);

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
template <typename ResultType>
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
        pac_(_mm_setzero_si128()),
        tmp0_(pac_), tmp1_(pac_), tmp2_(pac_), remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AESNI;
        seed(s);
    }

    template <typename SeedSeq>
    explicit AESNIEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) :
        pac_(_mm_setzero_si128()),
        tmp0_(pac_), tmp1_(pac_), tmp2_(pac_), remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_AESNI;
        seed(seq);
    }

    void seed (result_type s)
    {
        ctr_.fill(0);
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
        ctr_.fill(0);
        ukey_type uk;
        seq.generate(uk.begin(), uk.end());
        ukey(uk);
        remain_ = 0;
    }

    const ctr_type &ctr () const {return ctr_;}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        ctr_ = c;
        remain_ = 0;
    }

    void key (const key_type &k)
    {
        key_ = k;
        remain_ = 0;
    }

    void ukey (const ukey_type &uk)
    {
        internal::pack(uk, pac_);
        init_key(pac_);
        remain_ = 0;
    }

    /// \brief Same as operator() but return the __m128i type
    __m128i generate ()
    {
        internal::RngCounter<ResultType, K_>::increment(ctr_);
        pack();
        generate<0>(cxx11::true_type());
        remain_ = 0;

        return pac_;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            generate();
            internal::unpack(pac_, res_);
            remain_ = K_;
        }

        return res_[--remain_];
    }

    void discard (std::size_t nskip)
    {
        if (nskip == 0)
            return;

        --nskip;
        internal::RngCounter<ResultType, K_>::increment(ctr_.data(), nskip);
        remain_ = 0;
        operator()();
        nskip = nskip % K_;
        if (remain_ >= nskip) {
            remain_ -= nskip;
            return;
        }

        nskip -= remain_;
        remain_ = K_ - nskip;
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

    ctr_type ctr_;
    ctr_type res_;
    key_type key_;
    __m128i pac_;
    __m128i tmp0_;
    __m128i tmp1_;
    __m128i tmp2_;
    std::size_t remain_;

    void pack ()
    {
        internal::pack(ctr_, pac_);
        pac_ = _mm_xor_si128(pac_, key_.front());
    }

    template <std::size_t>
    void generate (cxx11::false_type)
    {pac_ = _mm_aesenclast_si128(pac_, key_.back());}

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        pac_ = _mm_aesenc_si128(pac_, key_[N + 1]);
        generate<N + 1>(cxx11::integral_constant<bool, N + 2 < R_>());
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
        key_[N] = tmp0_;
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

/// \brief AESNI RNG engine returning 32-bits integers
/// \ingroup R123RNG
typedef AESNIEngine<uint32_t> AESNI4x32;

/// \brief AESNI RNG engine returning 64-bits integers
/// \ingroup R123RNG
typedef AESNIEngine<uint64_t> AESNI2x64;

/// \brief AESNI RNG engine returning 128-bits integers
/// \ingroup R123RNG
typedef AESNIEngine<__m128i>  AESNI1x128;

} // namespace vsmc

#endif // VSMC_RNG_AESNI_HPP
