#ifndef VSMC_RNG_AESNI_HPP
#define VSMC_RNG_AESNI_HPP

#include <vsmc/rng/common.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_AESNI_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value ||                  \
             cxx11::is_same<ResultType, __m128i>::value),                    \
            USE_AESNIEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t_OR_m128i)

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
/// The implementation is slightly more flexible in the sense that the rounds
/// does not have to be 10. Otherwise it is almost identical. However, unlike
/// the reimplementation ThreefryEngine and PhiloxEngine, this engine won't
/// give exactly the same output as `AESNI4x32_R` etc.
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
        __m128i k = expand_seed(s);
        init_key(k);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        ctr_.fill(0);
        seq.generate(res_.begin(), res_.end());
        __m128i k = _mm_setzero_si128();
        _mm_storeu_si128(&k, *(reinterpret_cast<__m128i *>(res_.data())));
        init_key(k);
        remain_ = 0;
    }

    const key_type &key () const {return key_;}

    const ctr_type &ctr () const {return ctr_;}

    void key (__m128i k)
    {
        init_key(k);
        remain_ = 0;
    }

    void ctr (const ctr_type &c)
    {
        ctr_ = c;
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ > 0)
            return res_[--remain_];

        internal::RngCounter<ResultType, K_>::increment(ctr_.data());
        pack();
        generate<0>(cxx11::true_type());
        unpack();
        remain_ = K_ - 1;

        return res_[K_ - 1];
    }

    void discard (std::size_t nskip)
    {
        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

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
        _mm_storeu_si128(&pac_, *(reinterpret_cast<__m128i *>(ctr_.data())));
        pac_ = _mm_xor_si128(pac_, key_[0]);
    }

    void unpack ()
    {_mm_storeu_si128(reinterpret_cast<__m128i *>(res_.data()), pac_);}

    template <std::size_t>
    void generate (cxx11::false_type)
    {pac_ = _mm_aesenclast_si128(pac_, key_[R_]);}

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        pac_ = _mm_aesenc_si128(pac_, key_[N + 1]);
        generate<N + 1>(cxx11::integral_constant<bool, N + 1 < R_>());
    }

    void init_key(__m128i k)
    {
        tmp0_ = k;
        init_key<0>(cxx11::true_type());
    }

    __m128i expand_seed (uint32_t s)
    {return _mm_set1_epi32(static_cast<int32_t>(s));}

    __m128i expand_seed (uint64_t s)
    {return _mm_set1_epi64x(static_cast<int64_t>(s));}

    template <std::size_t>
    void init_key (cxx11::false_type) {key_[R_] = tmp0_;}

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
