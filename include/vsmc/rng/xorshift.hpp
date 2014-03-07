#ifndef VSMC_RNG_XORSHIFT_HPP
#define VSMC_RNG_XORSHIFT_HPP

#include <vsmc/rng/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(K) \
    VSMC_STATIC_ASSERT((K != 0), USE_XorshiftEngine_WITH_ORDER_EUQAL_TO_ZERO)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType) \
    VSMC_STATIC_ASSERT((::vsmc::cxx11::is_unsigned<ResultType>::value),      \
            USE_XorshiftEngine_WITH_A_ResultType_NOT_AN_UNSIGNED_INTEGER_TYPE)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType) \
    VSMC_STATIC_ASSERT((sizeof(ResultType) >= sizeof(uint32_t)),             \
            USE_XorshiftEngine_WITH_A_ResultType_SMALLER_THAN_32_BITS)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(I, K) \
    VSMC_STATIC_ASSERT((I != 0 || K == 1),                                   \
            USE_XorshiftEngine_WITH_INDEX_##I##_EQUAL_TO_ZERO)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX_ORDER(R, S, K) \
    VSMC_STATIC_ASSERT((R > S || K == 1),                                    \
            USE_XorshiftEngine_WITH_INDEX_##R##_NOT_LARGER_THAN_##S)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(A) \
    VSMC_STATIC_ASSERT((A != 0),                                             \
            USE_XorshiftEngine_WITH_SHIFT_BITS_##A##_EQUAL_TO_ZERO);

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_C(C, K) \
    VSMC_STATIC_ASSERT((C != 0 || K != 1),                                   \
            USE_XorshiftEngine_WITH_SHIFT_BITS_##C##_EQUAL_TO_ZERO);

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_D(D, K) \
    VSMC_STATIC_ASSERT((D != 0 || K == 1),                                   \
            USE_XorshiftEngine_WITH_SHIFT_BITS_##A##_EQUAL_TO_ZERO);

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(K);                                \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);                    \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);                   \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(R, K);                             \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(S, K);                             \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX_ORDER(R, S, K);                    \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(A);                           \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(B);                           \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_C(C, K);                      \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_D(D, K);

namespace vsmc {

namespace traits {

/// \brief Traits of XorshiftEngine
/// \ingroup Traits
template <typename ResultType>
struct XorshiftEngineTrait
{
    /// \brief Maximum number of states (e.g., 4 in Xorshift4x64) that an
    /// unrolled loop will be used
    ///
    /// \details
    /// The original implementation of Marsaglia promote each value. That is,
    /// e.g., for a state of four integers `x, y, z, w`, there is a step
    /// `x = y; y = z; z = w`. This can be generalized to the situation of
    /// any abitrary number of integers. For the number of integers smaller
    /// than or equal to this value, the original implementation will be used,
    /// equivalent to
    /// ~~~{.cpp}
    /// for (std::size_t i = 0; i != n - 1; ++i)
    ///     state[i] = state[i + 1];
    /// ~~~
    /// where `n` is the number of the integers. The loop will be unrolled at
    /// compile time. Otherwise, no loop will be performed. Instead, a circular
    /// buffer is implemented through the use of modulo operation.
    static VSMC_CONSTEXPR const std::size_t max_loop_unroll = 4;
}; // struct XorshiftEngineTrait

} // vsmc::namespace vsmc::traits

namespace internal {

template <bool, typename ResultType, unsigned>
struct XorshiftLeft
{static ResultType shift (ResultType x) {return x;}};

template <typename ResultType, unsigned A>
struct XorshiftLeft<true, ResultType, A>
{static ResultType shift (ResultType x) {return x^(x<<A);}};

template <bool, typename ResultType, unsigned>
struct XorshiftRight
{static ResultType shift (ResultType x) {return x;}};

template <typename ResultType, unsigned A>
struct XorshiftRight<true, ResultType, A>
{static ResultType shift (ResultType x) {return x^(x>>A);}};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S,
         bool  =
         (K <= traits::XorshiftEngineTrait<ResultType>::max_loop_unroll)>
struct XorshiftIndex
{
    void reset () {}

    static VSMC_CONSTEXPR std::size_t r () {return K - R;}
    static VSMC_CONSTEXPR std::size_t s () {return K - S;}
    static VSMC_CONSTEXPR std::size_t k () {return K - 1;}

    static void shift (StaticVector<ResultType, K> &state)
    {rng_array_left_shift<K, 1, false>(state);}
};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S>
struct XorshiftIndex<ResultType, K, R, S, false>
{
    XorshiftIndex () : iter_(0) {}

    void reset () {iter_ = 0;}

    std::size_t r () {return (K - R + iter_) % K;}
    std::size_t s () {return (K - S + iter_) % K;}
    std::size_t k () {return (K - 1 + iter_) % K;}

    void shift (StaticVector<ResultType, K> &)
    {iter_ = (iter_ + 1) % K;}

    private :

    std::size_t iter_;
};

template <unsigned A, unsigned B, unsigned C, unsigned,
    typename ResultType, std::size_t R, std::size_t S>
inline ResultType xorshift (StaticVector<ResultType, 1> &state,
        XorshiftIndex<ResultType, 1, R, S> &)
{
    state.front() ^= (state.front())<<A;
    state.front() ^= (state.front())>>B;
    state.front() ^= (state.front())<<C;

    return state.front();
}

template <unsigned A, unsigned B, unsigned C, unsigned D,
    typename ResultType, std::size_t K, std::size_t R, std::size_t S>
inline ResultType xorshift (StaticVector<ResultType, K> &state,
        XorshiftIndex<ResultType, K, R, S> &index)
{
    ResultType xr = state[index.r()];
    xr = XorshiftLeft <A != 0, ResultType, A>::shift(xr);
    xr = XorshiftRight<B != 0, ResultType, B>::shift(xr);

    ResultType xs = state[index.s()];
    xs = XorshiftLeft <C != 0, ResultType, C>::shift(xs);
    xs = XorshiftRight<D != 0, ResultType, D>::shift(xs);

    index.shift(state);

    return state[index.k()] = xs^xr;
}

} // namespace vsmc::internal

/// \brief Xorshift RNG engine
/// \ingroup Xorshift
///
/// \details
/// Use Marsaglia's Xorshift algorithm if `K == 1`, otherwise use Brent's
/// improvement. Marsaglia's multi-words version is equivalent to set `C = 0`,
/// `R = K`, and `S = 1`.
///
/// \tparam ResultType An unsigned 32- or 64-bits integer type
/// \tparam K Number of integers of type ResultType representing the states
/// \tparam A Bits of first left shift
/// \tparam B Bits of first right shift
/// \tparam C Bits of second left shift
/// \tparam D Bits of second right shift (unused if `K = 1`)
/// \tparam R Index of first xorshift (unused if `K = 1`)
/// \tparam S Index of second xorshift (unused if `K = 1`)
template <typename ResultType, std::size_t K,
         unsigned A, unsigned B, unsigned C, unsigned D,
         std::size_t R, std::size_t S>
class XorshiftEngine
{
    public :

    typedef ResultType result_type;

    explicit XorshiftEngine (result_type s = 123456)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(s);
    }

    template <typename SeedSeq>
    explicit XorshiftEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(seq);
    }

    void seed (result_type s)
    {
        index_.reset();
        StaticVector<uint32_t, 1> seed;
        seed.front() = static_cast<uint32_t>(s % uint32_t_max_);
        internal::XorshiftIndex<uint32_t, 1, 0, 0> index;
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = internal::xorshift<13, 17, 5, 0>(seed, index);
        discard(4 * K);
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR)
    {
        index_.reset();
        seq.generate(state_.begin(), state_.end());
        discard(4 * K);
    }

    result_type operator() ()
    {return internal::xorshift<A, B, C, D>(state_, index_);}

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
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {return eng1.state_ == eng2.state_;}

    friend inline bool operator!= (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {if (os) os << eng.state_; return os;}

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {
        StaticVector<ResultType, K> tmp;
        if (is) is >> std::ws >> tmp;
        if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng.state_ = cxx11::move(tmp);
#else
            eng.state_ = tmp;
#endif
        }

        return is;
    }

    private :

    internal::XorshiftIndex<ResultType, K, R, S> index_;
    StaticVector<ResultType, K> state_;

    static VSMC_CONSTEXPR const result_type uint32_t_max_ =
        static_cast<result_type>(static_cast<uint32_t>(
                    ~(static_cast<uint32_t>(0))));
}; // class XorshiftEngine

/// \brief Xorwow RNG engine
/// \ingroup Xorshift
///
/// \details
/// Use Marsaglia's Xorwow algorithm with an Xorshift engine.
template <typename Eng,
         typename Eng::result_type D = 362437,
         typename Eng::result_type DInit = 6615241>
class XorwowEngine
{
    public :

    typedef typename Eng::result_type result_type;
    typedef Eng engine_type;

    explicit XorwowEngine (result_type s = 123456) : eng_(s), weyl_(DInit) {}

    template <typename SeedSeq>
    explicit XorwowEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR) : eng_(seq), weyl_(DInit) {}

    void seed (result_type s)
    {
        eng_.seed(s);
        weyl_ = DInit;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR)
    {
        eng_.seed(seq);
        weyl_ = DInit;
    }

    result_type operator() ()
    {return eng_() + (weyl_ += D);}

    void discard (std::size_t nskip)
    {
        eng_.discard(nskip);
        weyl_ += D * nskip;
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const XorwowEngine<Eng, D, DInit> &eng1,
            const XorwowEngine<Eng, D, DInit> &eng2)
    {return eng1.eng_ == eng2.eng_ && eng1.weyl_ == eng2.weyl_;}

    friend inline bool operator!= (
            const XorwowEngine<Eng, D, DInit> &eng1,
            const XorwowEngine<Eng, D, DInit> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const XorwowEngine<Eng, D, DInit> &eng)
    {return (os << eng.eng_ << ' ' << eng.weyl_);}

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            XorwowEngine<Eng, D, DInit> &eng)
    {
        engine_type eng_tmp;
        result_type weyl_tmp = 0;
        if (is) is >> std::ws >> eng_tmp;
        if (is) is >> std::ws >> weyl_tmp;
        if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng.eng_ = cxx11::move(eng_tmp);
#else
            eng.eng_ = eng_tmp;
#endif
            eng.weyl_ = weyl_tmp;
        }

        return is;
    }

    private :

    Eng eng_;
    result_type weyl_;
}; // class XorwowEngine

/// \brief Xorshift RNG engine generating \f$2^32-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 1, 13, 17, 5, 0, 0, 0> Xorshift1x32;

/// \brief Xorshift RNG engine generating \f$2^64-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 1, 13, 7, 17, 0, 0, 0> Xorshift1x64;

/// \brief Xorshift RNG engine generating \f$2^64-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 2, 17, 14, 12, 19, 2, 1> Xorshift2x32;

/// \brief Xorshift RNG engine generating \f$2^128-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 4, 15, 14, 12, 17, 4, 3> Xorshift4x32;

/// \brief Xorshift RNG engine generating \f$2^256-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 8, 18, 13, 14, 15, 8, 3> Xorshift8x32;

/// \brief Xorshift RNG engine generating \f$2^512-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 16, 17, 15, 13, 14, 16, 1> Xorshift16x32;

/// \brief Xorshift RNG engine generating \f$2^1024-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 32, 19, 11, 13, 16, 32, 15> Xorshift32x32;

/// \brief Xorshift RNG engine generating \f$2^2048-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 64, 19, 12, 14, 15, 64, 59> Xorshift64x32;

/// \brief Xorshift RNG engine generating \f$2^4096-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint32_t, 128, 17, 12, 13, 15, 128, 95> Xorshift128x32;

/// \brief Xorshift RNG engine generating \f$2^128-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 2, 33, 31, 28, 29, 2, 1> Xorshift2x64;

/// \brief Xorshift RNG engine generating \f$2^256-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 4, 37, 27, 29, 33, 4, 3> Xorshift4x64;

/// \brief Xorshift RNG engine generating \f$2^512-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 8, 37, 26, 29, 34, 8, 1> Xorshift8x64;

/// \brief Xorshift RNG engine generating \f$2^1024-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 16, 34, 29, 25, 31, 16, 7> Xorshift16x64;

/// \brief Xorshift RNG engine generating \f$2^2048-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 32, 35, 27, 26, 37, 32, 1> Xorshift32x64;

/// \brief Xorshift RNG engine generating \f$2^4096-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorshiftEngine<uint64_t, 64, 33, 26, 27, 29, 64, 53> Xorshift64x64;

/// \brief THe default 32-bits Xorshift RNG engine
/// \ingroup Xorshift
typedef Xorshift128x32 Xorshift;

/// \brief THe default 64-bits Xorshift RNG engine
/// \ingroup Xorshift
typedef Xorshift64x64  Xorshift_64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^32-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift1x32> Xorwow1x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^64-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift1x64> Xorwow1x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^64-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift2x32> Xorwow2x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^128-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift4x32> Xorwow4x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^256-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift8x32> Xorwow8x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^512-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift16x32> Xorwow16x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^1024-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift32x32> Xorwow32x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^2048-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift64x32> Xorwow64x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^4096-1\f$ 32-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift128x32> Xorwow128x32;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^128-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift2x64> Xorwow2x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^256-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift4x64> Xorwow4x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^512-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift8x64> Xorwow8x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^1024-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift16x64> Xorwow16x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^2048-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift32x64> Xorwow32x64;

/// \brief Xorwow RNG engine based on Xorshfit \f$2^4096-1\f$ 64-bits integers
/// \ingroup Xorshift
typedef XorwowEngine<Xorshift64x64> Xorwow64x64;

/// \brief THe default 32-bits Xorwow RNG engine
/// \ingroup Xorshift
typedef Xorwow128x32 Xorwow;

/// \brief THe default 64-bits Xorwow RNG engine
/// \ingroup Xorshift
typedef Xorwow64x64  Xorwow_64;

} // namespace vsmc

#endif // VSMC_RNG_XORSHIFT_HPP
