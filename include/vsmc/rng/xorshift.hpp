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

namespace traits {

/// \brief Traits of XorshiftEngine
/// \ingroup Traits
template <typename ResultType>
struct XorshiftEngineTrait
{
    /// \brief Maximum size in bytes that can be allocated on stack
    ///
    /// \details
    /// The size of the states is `K * sizeof(ResultType)` where `K` is the
    /// order of the engine. For states smaller than or equal to this value, it
    /// will be allocated on the stack using an array. Otherwise it will be
    /// allocated on the heap.
    static VSMC_CONSTEXPR const std::size_t max_stack_alloc = 1024;

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

template <bool, typename ResultType, ResultType A>
struct XorshiftLeft
{static ResultType get (ResultType x) {return x;}};

template <typename ResultType, ResultType A>
struct XorshiftLeft<true, ResultType, A>
{static ResultType get (ResultType x) {return x^(x<<A);}};

template <bool, typename ResultType, ResultType A>
struct XorshiftRight
{static ResultType get (ResultType x) {return x;}};

template <typename ResultType, ResultType A>
struct XorshiftRight<true, ResultType, A>
{static ResultType get (ResultType x) {return x^(x>>A);}};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S,
         bool  =
         (K <= traits::XorshiftEngineTrait<ResultType>::max_loop_unroll)>
struct XorshiftIndex
{
    void reset () {}

    static VSMC_CONSTEXPR std::size_t r () {return K - R;}
    static VSMC_CONSTEXPR std::size_t s () {return K - S;}
    static VSMC_CONSTEXPR std::size_t k () {return K - 1;}

    static void shift (ResultType *state, Position<K>)
    {rng_array_shift(state, Position<K>());}
};

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S>
struct XorshiftIndex<ResultType, K, R, S, false>
{
    XorshiftIndex () : iter_(0) {}

    void reset () {iter_ = 0;}

    std::size_t r () {return (K - R + iter_) % K;}
    std::size_t s () {return (K - S + iter_) % K;}
    std::size_t k () {return (K - 1 + iter_) % K;}

    void shift (ResultType *, Position<K>) {iter_ = (iter_ + 1) % K;}

    private :

    std::size_t iter_;
};

template <typename ResultType, ResultType A, ResultType B, ResultType C,
         ResultType, std::size_t, std::size_t, typename IndexType>
inline ResultType xorshift (ResultType *state, IndexType &, Position<1>)
{
    *state ^= (*state)<<A;
    *state ^= (*state)>>B;
    *state ^= (*state)<<C;

    return *state;
}

template <typename ResultType, ResultType A, ResultType B, ResultType C,
    ResultType D, std::size_t R, std::size_t S, typename IndexType,
    std::size_t K>
inline ResultType xorshift (ResultType *state, IndexType &index, Position<K>)
{
    ResultType xr = state[index.r()];
    ResultType xs = state[index.s()];
    xr = XorshiftLeft <A != 0, ResultType, A>::get(xr);
    xr = XorshiftRight<B != 0, ResultType, B>::get(xr);
    xs = XorshiftLeft <C != 0, ResultType, C>::get(xs);
    xs = XorshiftRight<D != 0, ResultType, D>::get(xs);
    index.shift(state, Position<K>());

    return state[index.k()] = xs^xr;
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

    explicit XorshiftEngine (result_type s = 123456)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(s);
    }

    template <typename SeedSeq>
    explicit XorshiftEngine (SeedSeq &seq)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(seq);
    }

    XorshiftEngine (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other) :
        index_(other.index_), state_(other.state_)
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other) :
        index_(other.index_), state_(other.state_)
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine<ResultType, K, A, B, C, D, R, S> &operator= (
            const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &other)
    {
        if (this != &other) {
            index_ = other.index_;
            state_ = other.state_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    XorshiftEngine (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &&other) :
        index_(cxx11::move(other.index_)), state_(cxx11::move(other.state_))
    {VSMC_STATIC_ASSERT_RNG_XORSHIFT;}

    XorshiftEngine<ResultType, K, A, B, C, D, R, S> &operator= (
            XorshiftEngine<ResultType, K, A, B, C, D, R, S> &&other)
    {
        if (this != &other) {
            index_ = cxx11::move(other.index_);
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
        index_.reset();
        for (std::size_t i = 0; i != K; ++i) {
            internal::xorshift<uint32_t, 13, 17, 5, 0, 0, 0>(
                    &seed, index_, Position<1>());
            state_[0] = seed;
        }
        discard(4 * K);
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq)
    {
        index_.reset();
        seq.generate(state_.data(), state_.data() + K);
        discard(4 * K);
    }

    result_type operator() ()
    {
        return internal::xorshift<ResultType, A, B, C, D, R, S>(
                state_, index_, Position<K>());
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

    internal::XorshiftIndex<ResultType, K, R, S> index_;
    internal::RngStorage<result_type, K, (sizeof(ResultType) * K <=
            traits::XorshiftEngineTrait<ResultType>::max_stack_alloc)> state_;
}; // class XorshiftEngine

template <typename Eng,
         typename Eng::result_type D = 362437,
         typename Eng::result_type DInit = 6615241>
class XorwowEngine
{
}; // class XorwowEngine

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

/// \brief THe default 32-bits Xorshift engine
typedef Xorshift128x32 Xorshift;

/// \brief THe default 64-bits Xorshift engine
typedef Xorshift64x64  Xorshift_64;

} // namespace vsmc

#endif // VSMC_RNG_XORSHIFT_HPP
