#ifndef VSMC_RNG_ARS_HPP
#define VSMC_RNG_ARS_HPP

#include <vsmc/rng/m128i.hpp>

#define VSMC_STATIC_ASSERT_RNG_ARS_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value) ||                 \
             cxx11::is_same<ResultType, __m128i>::value)                     \
            USE_ARSEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R) \
    VSMC_STATIC_ASSERT((R > 0), USE_ARSEngine_WITH_ZERO_ROUND)

#define VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R) \
    VSMC_STATIC_ASSERT((R <= 10), USE_ARSEngine_WITH_ROUNDS_LARGER_THAN_10)

#define VSMC_STATIC_ASSERT_RNG_ARS_BLOCKS(Blocks) \
    VSMC_STATIC_ASSERT((Blocks > 0), USE_ARSEngine_WITH_ZERO_BLOCKS)

#define VSMC_STATIC_ASSERT_RNG_ARS \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND_0(R);                                   \
    VSMC_STATIC_ASSERT_RNG_ARS_ROUND(R);                                     \
    VSMC_STATIC_ASSERT_RNG_ARS_BLOCKS(Blocks);

#ifndef VSMC_RNG_ARS_BLOCKS
#define VSMC_RNG_ARS_BLOCKS 1
#endif

#ifndef VSMC_RNG_ARS_ROUNDS
#define VSMC_RNG_ARS_ROUNDS 10
#endif

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
template <typename ResultType,
         std::size_t Blocks = VSMC_RNG_ARS_BLOCKS,
         std::size_t R = VSMC_RNG_ARS_ROUNDS>
class ARSEngine
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(__m128i) / sizeof(ResultType);

    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K_> ctr_type;
    typedef StaticVector<__m128i, R + 1> key_type;
    typedef StaticVector<ResultType, K_> ukey_type;

    explicit ARSEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ARSEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_ARS;
        seed(seq);
    }

    ARSEngine (const ARSEngine<ResultType, Blocks, R> &other) :
        ctr_(other.ctr_), res_(other.res_), key_(other.key_), pac_(other.pac_),
        remain_(other.remain_) {}

    ARSEngine<ResultType, Blocks, R> &operator= (
            const ARSEngine<ResultType, Blocks, R> &other)
    {
        if (this != &other) {
            ctr_ = other.ctr_;
            res_ = other.res_;
            key_ = other.key_;
            pac_ = other.pac_;
            remain_ = other.remain_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    ARSEngine (ARSEngine<ResultType, Blocks, R> &&other) :
        ctr_(cxx11::move(other.ctr_)), res_(cxx11::move(other.res_)),
        key_(cxx11::move(other.key_)), pac_(cxx11::move(other.pac_)),
        remain_(other.remain_) {}

    ARSEngine<ResultType, Blocks, R> &operator= (
            ARSEngine<ResultType, Blocks, R> &&other)
    {
        if (this != &other) {
            ctr_ = cxx11::move(other.ctr_);
            res_ = cxx11::move(other.res_);
            key_ = cxx11::move(other.key_);
            pac_ = cxx11::move(other.pac_);
            remain_ = other.remain_;
        }

        return *this;
    }
#endif

    virtual ~ARSEngine () {}

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

    const StaticVector<ctr_type, Blocks> &ctr () const {return ctr_;}

    const key_type &key () const {return key_;}

    void ctr (const ctr_type &c)
    {
        ctr_.back() = c;
        increment();
        remain_ = 0;
    }

    void ctr (const StaticVector<ctr_type, Blocks> &c)
    {
        ctr_ = c;
        remain_ = 0;
    }

    void key (key_type k)
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

        return result();
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
        increment();
        if (nskip <= Blocks)
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
            const ARSEngine<ResultType, Blocks, R> &eng1,
            const ARSEngine<ResultType, Blocks, R> &eng2)
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
            const ARSEngine<ResultType, Blocks, R> &eng1,
            const ARSEngine<ResultType, Blocks, R> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const ARSEngine<ResultType, Blocks, R> &eng)
    {
        if (os) os << eng.ctr_ << ' ';
        if (os) os << eng.res_ << ' ';
        for (std::size_t i = 0; i != key_type::size(); ++i)
            internal::output_m128i(os, eng.key_[i]); if (os) os << ' ';
        for (std::size_t i = 0; i != Blocks; ++i)
            internal::output_m128i(os, eng.pac_[i]);  if (os) os << ' ';
        if (os) os << eng.remain_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            ARSEngine<ResultType, Blocks, R> &eng)
    {
        ARSEngine eng_tmp;
        if (is) is >> std::ws >> eng_tmp.ctr_;
        if (is) is >> std::ws >> eng_tmp.res_;
        for (std::size_t i = 0; i != Blocks; ++i)
            internal::input_m128i(is, eng_tmp.pac_[i]);
        for (std::size_t i = 0; i != Blocks; ++i)
            internal::input_m128i(is, eng_tmp.pac_[i]);
        if (is) is >> std::ws >> eng_tmp.remain_;
        if (is) eng = eng_tmp;

        return is;
    }

    protected :

    StaticVector<ctr_type, Blocks> &ctr () {return ctr_;}
    StaticVector<ctr_type, Blocks> &res () {return res_;}
    key_type &key () {return key_;}

    private :

    StaticVector<ctr_type, Blocks> ctr_;
    StaticVector<ctr_type, Blocks> res_;
    key_type key_;
    StaticVector<__m128i, Blocks> pac_;
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

    void unpack () {unpack<0>(cxx11::true_type());}

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
        generate<1>(cxx11::true_type());
        unpack();
        remain_ = K_ * Blocks;
    }

    template <std::size_t>
    void generate (cxx11::false_type) {generate_last<0>(cxx11::true_type());}

    template <std::size_t> void generate_last (cxx11::false_type) {}

    template <std::size_t B>
    void generate_last (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_aesenclast_si128(
                pac_[Position<B>()], key_.back());
        generate_last<B + 1>(cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t N>
    void generate (cxx11::true_type)
    {
        generate_step<0, N>(cxx11::true_type());
        generate<N + 1>(cxx11::integral_constant<bool, N  + 1 < R>());
    }

    template <std::size_t, std::size_t>
    void generate_step (cxx11::false_type) {}

    template <std::size_t B, std::size_t N>
    void generate_step (cxx11::true_type)
    {
        pac_[Position<B>()] = _mm_aesenc_si128(
                pac_[Position<B>()], key_[Position<N>()]);
        generate_step<B + 1, N>(
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    result_type result () {return result<Blocks - 1>(cxx11::true_type());}

    template <std::size_t>
    result_type result (cxx11::false_type) {return res_.front()[--remain_];}

    template <std::size_t I>
    result_type result (cxx11::true_type)
    {
        if (remain_ > I * K_) {
            --remain_;
            return res_[Position<I>()][remain_ - I * K_];
        }
        return result<I - 1>(cxx11::integral_constant<bool, 1 < I>());
    }

    virtual void init_key (const __m128i &k)
    {
        key_.front() = k;
        __m128i weyl = _mm_set_epi64x(
                static_cast<int64_t>(traits::ARSConstantTrait<0>::value),
                static_cast<int64_t>(traits::ARSConstantTrait<1>::value));
        init_key_seq<1>(weyl, cxx11::true_type());
    }

    template <std::size_t>
    void init_key_seq (const __m128i &, cxx11::false_type) {}

    template <std::size_t N>
    void init_key_seq (const __m128i &weyl, cxx11::true_type)
    {
        key_[Position<N>()] = _mm_add_epi64(key_[Position<N - 1>()], weyl);
        init_key_seq<N + 1>(weyl, cxx11::integral_constant<bool, N < R>());
    }
}; // class ARSEngine

/// \brief ARS RNG engine returning 32-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t> ARS4x32;

/// \brief ARS RNG engine returning 64-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t> ARS2x64;

/// \brief ARS RNG engine returning 128-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i>  ARS1x128;

/// \brief ARS RNG engine returning 32-bits integers with 1 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t, 1> ARS4x32_1;

/// \brief ARS RNG engine returning 64-bits integers with 1 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t, 1> ARS2x64_1;

/// \brief ARS RNG engine returning 128-bits integers with 1 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i, 1>  ARS1x128_1;

/// \brief ARS RNG engine returning 32-bits integers with 2 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t, 2> ARS4x32_2;

/// \brief ARS RNG engine returning 64-bits integers with 2 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t, 2> ARS2x64_2;

/// \brief ARS RNG engine returning 128-bits integers with 2 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i, 2>  ARS1x128_2;

/// \brief ARS RNG engine returning 32-bits integers with 4 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t, 4> ARS4x32_4;

/// \brief ARS RNG engine returning 64-bits integers with 4 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t, 4> ARS2x64_4;

/// \brief ARS RNG engine returning 128-bits integers with 4 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i, 4>  ARS1x128_4;

/// \brief ARS RNG engine returning 32-bits integers with 8 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t, 8> ARS4x32_8;

/// \brief ARS RNG engine returning 64-bits integers with 8 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t, 8> ARS2x64_8;

/// \brief ARS RNG engine returning 128-bits integers with 8 block and default
/// rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i, 8>  ARS1x128_8;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
