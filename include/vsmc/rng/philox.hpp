#ifndef VSMC_RNG_PHILOX_HPP
#define VSMC_RNG_PHILOX_HPP

#include <vsmc/rng/common.hpp>

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER
#include <Random123/philox.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER
#endif // VSMC_USE_RANDOM123

#define VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_same<ResultType, uint32_t>::value ||                  \
             cxx11::is_same<ResultType, uint64_t>::value),                   \
            USE_PhiloxEngine_WITH_INTEGER_TYPE_OTHER_THAN_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K) \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                   \
            USE_PhiloxEngine_WITH_SIZE_OTHER_THAN_2_OR_4)

#define VSMC_STATIC_ASSERT_RNG_PHILOX_ROUND(R) \
    VSMC_STATIC_ASSERT((R <= 16),                                            \
            USE_PhiloxEngine_WITH_ROUNDS_LARGER_THAN_16)

#define VSMC_STATIC_ASSERT_RNG_PHILOX \
        VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType);             \
        VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K);                             \
        VSMC_STATIC_ASSERT_RNG_PHILOX_ROUND(R);

#define VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(T, I, val) \
    template <> struct PhiloxBumpkConstant < T, I > :                        \
        public cxx11::integral_constant< T, val > {};

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val) \
    template <> struct PhiloxRoundConstant < T, K, I > :                     \
        public cxx11::integral_constant< T, val > {};

namespace vsmc {

namespace internal {

#if VSMC_USE_RANDOM123
template <typename, std::size_t, std::size_t> struct PhiloxR123Trait;

template <std::size_t R>
struct PhiloxR123Trait<uint32_t, 2, R>
{typedef r123::Philox2x32_R<R> type;};

template <std::size_t R>
struct PhiloxR123Trait<uint32_t, 4, R>
{typedef r123::Philox4x32_R<R> type;};

template <std::size_t R>
struct PhiloxR123Trait<uint64_t, 2, R>
{typedef r123::Philox2x64_R<R> type;};

template <std::size_t R>
struct PhiloxR123Trait<uint64_t, 4, R>
{typedef r123::Philox4x64_R<R> type;};
#endif

template <typename, std::size_t> struct PhiloxBumpkConstant;

VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint32_t, 0,
        static_cast<uint32_t>(0x9E3779B9))
VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint32_t, 1,
        static_cast<uint32_t>(0xBB67AE85))

VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint64_t, 0,
        UINT64_C(0x9E3779B97F4A7C15))
VSMC_DEFINE_RNG_PHILOX_BUMPK_CONSTANT(uint64_t, 1,
        UINT64_C(0xBB67AE8584CAA73B))

template <typename, std::size_t, std::size_t> struct PhiloxRoundConstant;

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 2, 0,
        static_cast<uint32_t>(0xd256d193))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 4, 0,
        static_cast<uint32_t>(0xD2511F53))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint32_t, 4, 1,
        static_cast<uint32_t>(0xCD9E8D57))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 2, 0,
        UINT64_C(0xD2B74407B1CE6E93))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 4, 0,
        UINT64_C(0xD2E7470EE14C6C93))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(uint64_t, 4, 1,
        UINT64_C(0xCA5A826395121157))

template <typename ResultType, std::size_t, std::size_t N, bool = (N > 1)>
struct PhiloxBumpk {static void bumpk (ResultType *) {}};

template <typename ResultType, std::size_t N>
struct PhiloxBumpk<ResultType, 2, N, true>
{
    static void bumpk (ResultType *ks)
    {ks[0] += PhiloxBumpkConstant<ResultType, 0>::value;}
};

template <typename ResultType, std::size_t N>
struct PhiloxBumpk<ResultType, 4, N, true>
{
    static void bumpk (ResultType *ks)
    {
        ks[0] += PhiloxBumpkConstant<ResultType, 0>::value;
        ks[1] += PhiloxBumpkConstant<ResultType, 1>::value;
    }
};

template <std::size_t K, std::size_t N, typename ResultType>
void philox_hilo (ResultType b, ResultType &hi, ResultType &lo)
{
    const ResultType a = PhiloxRoundConstant<ResultType, K, N>::value;
    const unsigned whalf = RngUIntBits<ResultType>::value / 2;
    const ResultType lomask = (static_cast<ResultType>(1) << whalf) - 1;

    lo = a * b;

    ResultType ahi = a >> whalf;
    ResultType alo = a & lomask;
    ResultType bhi = b >> whalf;
    ResultType blo = b & lomask;

    ResultType ahbl = ahi * blo;
    ResultType albh = alo * bhi;

    ResultType ahbl_albh = ((ahbl & lomask) + (albh & lomask));

    hi = ahi * bhi + (ahbl >> whalf) + (albh >> whalf);
    hi += ahbl_albh >> whalf;
    hi += ((lo >> whalf) < (ahbl_albh & lomask));
}

template <typename ResultType, std::size_t, std::size_t N, bool = (N > 0)>
struct PhiloxRound {static void round (ResultType *, const ResultType *) {}};

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 2, N, true>
{
    static void round (ResultType *state, const ResultType *ks)
    {
        ResultType hi = 0;
        ResultType lo = 0;
        philox_hilo<2, 0>(state[0], hi, lo);
        state[0] = hi^ks[0]^state[1];
        state[1] = lo;
    }
};

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 4, N, true>
{
    static void round (ResultType *state, const ResultType *ks)
    {
        ResultType hi0 = 0;
        ResultType hi1 = 0;
        ResultType lo0 = 0;
        ResultType lo1 = 0;
        philox_hilo<4, 0>(state[0], hi0, lo0);
        philox_hilo<4, 1>(state[2], hi1, lo1);
        state[0] = hi1^state[1]^ks[0];
        state[1] = lo1;
        state[2] = hi0^state[3]^ks[1];
        state[3] = lo0;
    }
};

} // namespace vsmc::internal

template <typename ResultType, std::size_t K, std::size_t R = 10>
class PhiloxEngine
{
    public :

    typedef ResultType result_type;
    typedef StaticVector<ResultType, K / 2> key_type;
    typedef StaticVector<ResultType, K> ctr_type;

    explicit PhiloxEngine (result_type s = 0) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(s);
    }

    template <typename SeedSeq>
    explicit PhiloxEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : remain_(0)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(seq);
    }

    PhiloxEngine (const PhiloxEngine<ResultType, K, R> &other) :
        remain_(other.remain_),
        key_(other.key_), ctr_(other.ctr_), res_(other.res_), ks_(other.ks_)
    {VSMC_STATIC_ASSERT_RNG_PHILOX;}

    PhiloxEngine<ResultType, K, R> &operator= (
            const PhiloxEngine<ResultType, K, R> &other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            key_ = other.key_;
            ctr_ = other.ctr_;
            res_ = other.res_;
            ks_ = other.ks_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    PhiloxEngine (PhiloxEngine<ResultType, K, R> &&other) :
        remain_(other.remain_),
        key_(cxx11::move(other.key_)), ctr_(cxx11::move(other.ctr_)),
        res_(cxx11::move(other.res_))
    {VSMC_STATIC_ASSERT_RNG_PHILOX;}

    PhiloxEngine<ResultType, K, R> &operator= (
            PhiloxEngine<ResultType, K, R> &&other)
    {
        if (this != &other) {
            remain_ = other.remain_;
            key_ = cxx11::move(other.key_);
            ctr_ = cxx11::move(other.ctr_);
            res_ = cxx11::move(other.res_);
            ks_ = cxx11::move(other.ks_);
        }

        return *this;
    }
#endif

    void seed (result_type s)
    {
        remain_ = 0;
        key_.fill(0);
        ctr_.fill(0);
        res_.fill(0);
        key_[0] = s;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        remain_ = 0;
        key_.fill(0);
        ctr_.fill(0);
        res_.fill(0);
        seq.generate(key_.begin(), key_.end());
    }

    const key_type &key () const {return key_;}

    const ctr_type &ctr () const {return ctr_;}

    void key (const key_type &k)
    {
        remain_ = 0;
        key_ = k;
    }

    void ctr (const ctr_type &c)
    {
        remain_ = 0;
        ctr_ = c;
    }

#if VSMC_USE_RANDOM123
    void key (const typename internal::PhiloxR123Trait<
            ResultType, K, R>::type::key_type &k)
    {
        remain_ = 0;
        for (std::size_t i = 0; i != K / 2; ++i)
            key_[i] = k.v[i];
    }

    void ctr (const typename internal::PhiloxR123Trait<
            ResultType, K, R>::type::ctr_type &c)
    {
        remain_ = 0;
        for (std::size_t i = 0; i != K; ++i)
            ctr_[i] = c.v[i];
    }
#endif

    result_type operator() ()
    {
        if (remain_ > 0)
            return res_[--remain_];

        internal::RngCounterIncrement<ResultType, K>::increment(ctr_.data());
        ks_ = key_;
        res_ = ctr_;
        generate<0>(res_.data(), ks_.data(), cxx11::true_type());
        remain_ = K - 1;

        return res_[K - 1];
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

    friend inline bool operator== (
            const PhiloxEngine<ResultType, K, R> &eng1,
            const PhiloxEngine<ResultType, K, R> &eng2)
    {
        return eng1.remain_ == eng2.remain_ &&
            eng1.key_ == eng2.key_ &&
            eng1.ctr_ == eng2.ctr_ &&
            eng1.res_ == eng2.res_;
    }

    friend inline bool operator!= (
            const PhiloxEngine<ResultType, K, R> &eng1,
            const PhiloxEngine<ResultType, K, R> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const PhiloxEngine<ResultType, K, R> &eng)
    {
        os << eng.remain_ << ' ';
        os << eng.key_ << ' ' << eng.ctr_ << ' ' << eng.res_ << ' ' << eng.ks_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            PhiloxEngine<ResultType, K, R> &eng)
    {
        PhiloxEngine eng_tmp;
        if (is) is >> eng_tmp.remain_;
        if (is) is >> eng_tmp.key_;
        if (is) is >> eng_tmp.ctr_;
        if (is) is >> eng_tmp.res_;
        if (is) is >> eng_tmp.ks_;
        if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng = cxx11::move(eng_tmp);
#else
            eng = eng_tmp;
#endif
        }

        return is;
    }

    private :

    std::size_t remain_;
    key_type key_;
    ctr_type ctr_;
    ctr_type res_;
    key_type ks_;

    template <std::size_t N>
    void generate (result_type *, result_type *, cxx11::false_type) {}

    template <std::size_t N>
    void generate (result_type *state, result_type *ks, cxx11::true_type)
    {
        internal::PhiloxBumpk<ResultType, K, N>::bumpk(ks);
        internal::PhiloxRound<ResultType, K, N>::round(state, ks);
        generate<N + 1>(state, ks, cxx11::integral_constant<bool, N < R>());
    }
}; // class PhiloxEngine

/// \brief Philox RNG engine re-implemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint32_t, 2> Philox2x32;

/// \brief Philox RNG engine re-implemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint32_t, 4> Philox4x32;

/// \brief Philox RNG engine re-implemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint64_t, 2> Philox2x64;

/// \brief Philox RNG engine re-implemented
/// \ingroup R123RNG
typedef PhiloxEngine<uint64_t, 4> Philox4x64;

} // namespace vsmc

#endif // VSMC_RNG_PHILOX_HPP
