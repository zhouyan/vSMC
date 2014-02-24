#ifndef VSMC_RNG_XORSHIFT_HPP
#define VSMC_RNG_XORSHIFT_HPP

#include <vsmc/rng/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R) \
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
    VSMC_STATIC_ASSERT((R > S || K == 1),                                  \
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

namespace internal {

template <bool, typename ResultType, ResultType A>
struct xorshift_left
{static ResultType get (ResultType x) {return x;}};

template <typename ResultType, ResultType A>
struct xorshift_left<true, ResultType, A>
{static ResultType get (ResultType x) {return x^(x<<A);}};

template <bool, typename ResultType, ResultType A>
struct xorshift_right
{static ResultType get (ResultType x) {return x;}};

template <typename ResultType, ResultType A>
struct xorshift_right<true, ResultType, A>
{static ResultType get (ResultType x) {return x^(x>>A);}};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S,
         bool  = (K <= 4)>
struct xorshift_index
{
    static VSMC_CONSTEXPR std::size_t r (std::size_t) {return K - R;}

    static VSMC_CONSTEXPR std::size_t s (std::size_t) {return K - S;}

    static VSMC_CONSTEXPR std::size_t k (std::size_t) {return K - 1;}

    static void shift (ResultType *state, Position<K>, std::size_t &)
    {rng_array_shift(state, Position<K>());}
};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S>
struct xorshift_index<ResultType, K, R, S, false>
{
    static std::size_t r (std::size_t iter) {return (K - R + iter) % K;}

    static std::size_t s (std::size_t iter) {return (K - S + iter) % K;}

    static std::size_t k (std::size_t iter) {return (K - 1 + iter) % K;}

    static void shift (ResultType *, Position<K>, std::size_t &iter)
    {iter = (iter + 1) % K;}
};

template <typename ResultType, std::size_t K,
         bool B = (sizeof(ResultType) * K <= 512)>
class xorshift_storage
{
    public :

    xorshift_storage () : state_() {}

    xorshift_storage (const xorshift_storage<ResultType, K, B> &other) :
        state_()
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    xorshift_storage<ResultType, K, B> &operator= (
            const xorshift_storage<ResultType, K, B> &other)
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    ResultType &operator[] (std::size_t i) {return state_[i];}
    const ResultType &operator[] (std::size_t i) const {return state_[i];}

    operator ResultType * () {return state_;}
    operator const ResultType * () const {return state_;}

    ResultType *data () {return state_;}
    const ResultType *data () const {return state_;}

    private :

    ResultType state_[K];
};

template <typename ResultType, std::size_t K>
class xorshift_storage<ResultType, K, false>
{
    public :

    xorshift_storage () : state_(new ResultType[K]) {}

    xorshift_storage (const xorshift_storage<ResultType, K, false> &other) :
        state_(new ResultType[K])
    {
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = other.state_[i];
    }

    xorshift_storage<ResultType, K, false> &operator= (
            const xorshift_storage<ResultType, K, false> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != K; ++i)
                state_[i] = other.state_[i];
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    xorshift_storage (xorshift_storage<ResultType, K, false> &&other) :
        state_(other.state_)
    {other.state_ = VSMC_NULLPTR;}

    xorshift_storage<ResultType, K, false> &operator= (
            xorshift_storage<ResultType, K, false> &&other)
    {
        if (this != &other) {
            if (state_ != VSMC_NULLPTR)
                delete [] state_;
            state_ = other.state_;
            other.state_ = VSMC_NULLPTR;
        }

        return *this;
    }
#endif

    ~xorshift_storage ()
    {
        if (state_ != VSMC_NULLPTR)
            delete [] state_;
    }

    ResultType &operator[] (std::size_t i) {return state_[i];}
    const ResultType &operator[] (std::size_t i) const {return state_[i];}

    operator ResultType * () {return state_;}
    operator const ResultType * () const {return state_;}

    ResultType *data () {return state_;}
    const ResultType *data () const {return state_;}

    private :

    ResultType *state_;
};

template <typename ResultType, ResultType A, ResultType B, ResultType C,
         ResultType, std::size_t, std::size_t>
inline ResultType xorshift (ResultType *state, Position<1>, std::size_t &)
{
    *state ^= (*state)<<A;
    *state ^= (*state)>>B;
    *state ^= (*state)<<C;

    return *state;
}

template <typename ResultType, ResultType A, ResultType B, ResultType C,
    ResultType D, std::size_t R, std::size_t S, std::size_t K>
inline ResultType xorshift (ResultType *state, Position<K>, std::size_t &iter)
{
    ResultType xr = state[xorshift_index<ResultType, K, R, S>::r(iter)];
    ResultType xs = state[xorshift_index<ResultType, K, R, S>::s(iter)];
    xr = xorshift_left <A != 0, ResultType, A>::get(xr);
    xr = xorshift_right<B != 0, ResultType, B>::get(xr);
    xs = xorshift_left <C != 0, ResultType, C>::get(xs);
    xs = xorshift_right<D != 0, ResultType, D>::get(xs);
    xorshift_index<ResultType, K, R, S>::shift(state, Position<K>(), iter);

    return state[xorshift_index<ResultType, K, R, S>::k(iter)] = xs^xr;
}

} // namespace vsmc::internal

/// \brief Xorshift RNG engine
/// \ingroup RNG
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
         ResultType A, ResultType B, ResultType C,
         ResultType D, std::size_t R, std::size_t S>
class XorshiftEngine
{
    public :

    typedef ResultType result_type;

    explicit XorshiftEngine (result_type s = 123456) : iter_(0)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(s);
    }

    template <typename SeedSeq>
    explicit XorshiftEngine (SeedSeq &seq) : iter_(0)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(seq);
    }

    XorshiftEngine (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other) :
        iter_(other.iter_), state_(other.state_)
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other) :
        iter_(other.iter_), state_(other.state_)
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine<ResultType, K, A, B, C, D, R, S> &operator= (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other)
    {
        if (this != &other) {
            iter_ = other.iter_;
            state_ = other.state_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    XorshiftEngine (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &&other) :
        iter_(other.iter_), state_(cxx11::move(other.state_))
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine<ResultType, K, A, B, C, D, R, S> &operator= (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &&other)
    {
        if (this != &other) {
            iter_ = other.iter_;
            state_ = cxx11::move(other.state_);
        }

        return *this;
    }
#endif

    void seed (result_type s)
    {
        static VSMC_CONSTEXPR const result_type u32max =
            static_cast<result_type>(std::numeric_limits<uint32_t>::
                    max VSMC_MNE ());
        uint32_t seed = static_cast<uint32_t>(s % u32max);
        std::size_t iter = 0;
        for (std::size_t i = 0; i != K; ++i) {
            internal::xorshift<uint32_t, 13, 17, 5, 0, 0, 0>(
                    &seed, Position<1>(), iter);
            state_[0] = s;
        }
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq) {seq.generate(state_.data(), state_.data() + K);}

    result_type operator() ()
    {
        return internal::xorshift<ResultType, A, B, C, D, R, S>(
                state_, Position<K>(), iter_);
    }

    void discard (std::size_t nskip)
    {
        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max =
        ~(static_cast<result_type>(0));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {
        for (std::size_t i = 0; i != K; ++i) {
            if (eng1.state_[i] != eng2.state_[i])
                return false;
        }

        return true;
    }

    friend inline bool operator!= (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {
        for (std::size_t i = 0; i != K - 1; ++i)
            os << eng.state_[i] << ' ';
        os << eng.state_[K - 1];

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {
        result_type state[K];
        for (std::size_t i = 0; i != K; ++i) {
            if(!(is >> std::ws >> state[i]))
                break;
        }
        if (is) {
            for (std::size_t i = 0; i != K; ++i)
                eng.state_[i] = state[i];
        }

        return is;
    }

    private :

    std::size_t iter_;
    internal::xorshift_storage<result_type, K> state_;
}; // class XorshiftEngine

/// \brief Xorshift RNG engine generating \f$2^32 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 1, 13, 17, 5, 0, 0, 0> Xorshift1x32;

/// \brief Xorshift RNG engine generating \f$2^64 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 1, 13, 7, 17, 0, 0, 0> Xorshift1x64;

/// \brief Xorshift RNG engine generating \f$2^64 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 2, 17, 14, 12, 19, 2, 1> Xorshift2x32;

/// \brief Xorshift RNG engine generating \f$2^128 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 4, 15, 14, 12, 17, 4, 3> Xorshift4x32;

/// \brief Xorshift RNG engine generating \f$2^256 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 8, 18, 13, 14, 15, 8, 3> Xorshift8x32;

/// \brief Xorshift RNG engine generating \f$2^512 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 16, 17, 15, 13, 14, 16, 1> Xorshift16x32;

/// \brief Xorshift RNG engine generating \f$2^1024 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 32, 19, 11, 13, 16, 32, 15> Xorshift32x32;

/// \brief Xorshift RNG engine generating \f$2^2048 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 64, 19, 12, 14, 15, 64, 59> Xorshift64x32;

/// \brief Xorshift RNG engine generating \f$2^4096 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 128, 17, 12, 13, 15, 128, 95> Xorshift128x32;

/// \brief Xorshift RNG engine generating \f$2^128 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 2, 33, 31, 28, 29, 2, 1> Xorshift2x64;

/// \brief Xorshift RNG engine generating \f$2^256 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 4, 37, 27, 29, 33, 4, 3> Xorshift4x64;

/// \brief Xorshift RNG engine generating \f$2^512 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 8, 37, 26, 29, 34, 8, 1> Xorshift8x64;

/// \brief Xorshift RNG engine generating \f$2^1024 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 16, 34, 29, 25, 31, 16, 7> Xorshift16x64;

/// \brief Xorshift RNG engine generating \f$2^2048 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 32, 35, 27, 26, 37, 32, 1> Xorshift32x64;

/// \brief Xorshift RNG engine generating \f$2^4096 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 64, 33, 26, 27, 29, 64, 53> Xorshift64x64;

/// \brief THe default Xorshift engine
typedef Xorshift16x64 Xorshift;

} // namespace vsmc

#endif // VSMC_RNG_XORSHIFT_HPP
