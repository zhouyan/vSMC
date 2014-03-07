#ifndef VSMC_RNG_ARS_HPP
#define VSMC_RNG_ARS_HPP

#include <vsmc/rng/common.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_ARS_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value ||                  \
             cxx11::is_same<ResultType, __m128i>::value),                    \
            USE_ARSEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t_OR_m128i)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R) \
    VSMC_STATIC_ASSERT((R > 0), USE_ARSEngine_WITH_ZERO_ROUND)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R) \
    VSMC_STATIC_ASSERT((R <= 10), USE_ARSEngine_WITH_ROUNDS_LARGER_THAN_10)

#define VSMC_STATIC_ASSERT_RNG_ARS \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R);                                   \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R);

namespace vsmc {

/// \brief ARS RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm ARS as described in [Parallel
/// Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// The implementation is slightly more flexible in the sense that the rounds
/// does not have to be 10. Otherwise it is almost identical. However, unlike
/// the reimplementation ThreefryEngine and PhiloxEngine, this engine won't
/// give exactly the same output as `ARS4x32_R` etc.
template <typename ResultType, std::size_t R = 10>
class ARSEngine
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType);

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef __m128i key_type;

    explicit ARSEngine (result_type s = 0) :
        par_(_mm_setzero_si128()), weyl_(par_), pac_(par_), remain_(0) 
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ARSEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) :
        par_(_mm_setzero_si128()), weyl_(par_), pac_(par_), remain_(0) 
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(seq);
    }

    void seed (result_type s)
    {
        ctr_.fill(0);
        key_ = expand_seed(s);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        ctr_.fill(0);
        seq.generate(res_.begin(), res_.end());
        _mm_storeu_si128(&key_, *(reinterpret_cast<__m128i *>(res_.data())));
        remain_ = 0;
    }

    const key_type &key () const {return key_;}

    const ctr_type &ctr () const {return ctr_;}

    void key (key_type k)
    {
        key_ = k;
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
    __m128i par_;
    __m128i weyl_;
    __m128i pac_;
    std::size_t remain_;

    void pack ()
    {
        par_ = key_;
        weyl_ = _mm_set_epi64x(
                static_cast<int64_t>(UINT64_C(0xBB67AE8584CAA73B)),
                static_cast<int64_t>(UINT64_C(0x9E3779B97F4A7C15)));
        _mm_storeu_si128(&pac_, *(reinterpret_cast<__m128i *>(ctr_.data())));
        pac_ = _mm_xor_si128(pac_, par_);
    }

    void unpack ()
    {_mm_storeu_si128(reinterpret_cast<__m128i *>(res_.data()), pac_);}

    template <std::size_t>
    void generate (cxx11::false_type)
    {
        par_ = _mm_add_epi64(par_, weyl_);
        pac_ = _mm_aesenclast_si128(pac_, par_);
    }

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        par_ = _mm_add_epi64(par_, weyl_);
        pac_ = _mm_aesenc_si128(pac_, par_);
        generate<N + 1>(cxx11::integral_constant<bool, N < R>());
    }

    __m128i expand_seed (uint32_t s)
    {return _mm_set1_epi32(static_cast<int32_t>(s));}

    __m128i expand_seed (uint64_t s)
    {return _mm_set1_epi64x(static_cast<int64_t>(s));}
}; // class ARSEngine

/// \brief ARS RNG engine returning 32-bits integers
/// \ingroup R123RNG
typedef ARSEngine<uint32_t> ARS4x32;

/// \brief ARS RNG engine returning 64-bits integers
/// \ingroup R123RNG
typedef ARSEngine<uint64_t> ARS2x64;

/// \brief ARS RNG engine returning 128-bits integers
/// \ingroup R123RNG
typedef ARSEngine<__m128i>  ARS1x128;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
