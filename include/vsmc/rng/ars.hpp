#ifndef VSMC_RNG_ARS_HPP
#define VSMC_RNG_ARS_HPP

#include <vsmc/rng/m128i.hpp>

#define VSMC_STATIC_ASSERT_RNG_ARS_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_ARSEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R) \
    VSMC_STATIC_ASSERT((R > 0), USE_ARSEngine_WITH_ZERO_ROUND)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R) \
    VSMC_STATIC_ASSERT((R <= 10), USE_ARSEngine_WITH_ROUNDS_LARGER_THAN_10)

#define VSMC_STATIC_ASSERT_RNG_ARS \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R);                                   \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R);

namespace vsmc {

namespace internal {

template <std::size_t> struct ARSConstant;

template <> struct ARSConstant<0> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0xBB67AE8584CAA73B)> {};

template <> struct ARSConstant<1> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0x9E3779B97F4A7C15)> {};

} // namespace vsmc::internal

namespace traits {

/// \brief Traits of ARSEngine constants (Weyl sequence)
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bits or
/// the Weyl constant.
template <std::size_t N> struct ARSConstantTrait :
    public ::vsmc::internal::ARSConstant<N> {};

} // namespace vsmc::traits

/// \brief ARS RNG engine reimplemented
/// \ingroup R123RNG
///
/// \details
/// This is a reimplementation of the algorithm ARS as described in [Parallel
/// Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// The implementation is almost identical to the original. Compared to
/// `r123:Engine<r123::ARS4x32_R<10> >` etc., when using the default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^32\f$ iterations. Further iterations may produce
/// different results, as vSMC increment the counter slightly differently, but
/// it still cover the same range and has the same period as the original.
///
/// This implementation is slightly more flexible than the original. First it
/// allows using 64-bits integers as output. Second It allows setting the Weyl
/// constants through trait `vsmc::traits::ARSConstantTrait`.
template <typename ResultType, std::size_t R = 10>
class ARSEngine
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType);

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef StaticVector<ResultType, K_> key_type;

    explicit ARSEngine (result_type s = 0) :
        par_(_mm_setzero_si128()), weyl_ (_mm_set_epi64x(
                    static_cast<int64_t>(traits::ARSConstantTrait<0>::value),
                    static_cast<int64_t>(traits::ARSConstantTrait<1>::value))),
        pac_(par_), remain_(0) 
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ARSEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) :
        par_(_mm_setzero_si128()), weyl_ (_mm_set_epi64x(
                    static_cast<int64_t>(traits::ARSConstantTrait<0>::value),
                    static_cast<int64_t>(traits::ARSConstantTrait<1>::value))),
        pac_(par_), remain_(0) 
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(seq);
    }

    void seed (result_type s)
    {
        ctr_.fill(0);
        key_.fill(0);
        key_[0] = s;
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        ctr_.fill(0);
        seq.generate(key_.begin(), key_.end());
        remain_ = 0;
    }

    const ctr_type &ctr () const {return ctr_;}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        ctr_ = c;
        remain_ = 0;
    }

    void key (key_type k)
    {
        key_ = k;
        remain_ = 0;
    }

    /// \brief Same as operator() but return the __m128i type
    __m128i generate ()
    {
        internal::RngCounter<ResultType, K_>::increment(ctr_.data());
        pack();
        generate<0>(cxx11::integral_constant<bool, 1 < R>());
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
        internal::pack(ctr_, pac_);
        internal::pack(key_, par_);
        pac_ = _mm_xor_si128(pac_, par_);
    }

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
        generate<N + 1>(cxx11::integral_constant<bool, N  + 2 < R>());
    }
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
