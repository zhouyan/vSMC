#ifndef VSMC_RNG_XORSHIFT_HPP
#define VSMC_RNG_XORSHIFT_HPP

#include <vsmc/rng/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R) \
    VSMC_STATIC_ASSERT((R != 0), USE_XorshiftEngine_WITH_ZERO_INTERNAL_STATE)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType) \
    VSMC_STATIC_ASSERT((::vsmc::cxx11::is_unsigned<ResultType>::value),      \
            USE_XorshiftEngine_WITH_A_ResultType_NOT_AN_UNSIGNED_INTEGER_TYPE)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType) \
    VSMC_STATIC_ASSERT((sizeof(ResultType) >= sizeof(uint32_t)),             \
            USE_XorshiftEngine_WITH_A_ResultType_SMALLER_THAN_32_BITS)

namespace vsmc {

namespace internal {

template <typename ResultType>
inline void xorshift_assign (ResultType *state, Position<2>)
{
    state[0] = state[1];
}

template <typename ResultType, std::size_t R>
inline void xorshift_assign (ResultType *state, Position<R>)
{
    state[0] = state[1];
    xorshift_assign(state + 1, Position<R - 1>());
}

template <typename ResultType, ResultType A, ResultType B, ResultType C>
inline void xorshift (ResultType *state, Position<1>)
{
    *state ^= (*state)<<A;
    *state ^= (*state)>>B;
    *state ^= (*state)>>C;
}

template <typename ResultType, ResultType A, ResultType B, ResultType C,
    std::size_t R>
inline void xorshift (ResultType *state, Position<R>)
{
    ResultType t = state[0];
    t ^= t<<A;
    t ^= t>>B;
    xorshift_assign(state, Position<R>());
    state[R - 1] = (state[R - 1]^(state[R - 1]>>C))^t;
}

}; // namespace vsmc::internal

/// \brief Xorshift RNG engine
/// \ingroup RNG
template <typename ResultType, std::size_t R,
         ResultType A, ResultType B, ResultType C>
class XorshiftEngine
{
    public :

    typedef ResultType result_type;

    explicit XorshiftEngine (result_type s = 123456)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);
        seed(s);
    }

    template <typename SeedSeq>
    explicit XorshiftEngine (SeedSeq &seq)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);
        seed(seq);
    }

    XorshiftEngine (const XorshiftEngine<ResultType, R, A, B, C> &other)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);
        for (std::size_t i = 0; i != R; ++i)
            state_[i] = other.state_[i];
    }

    XorshiftEngine (XorshiftEngine<ResultType, R, A, B, C> &other)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(R);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);
        VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);
        for (std::size_t i = 0; i != R; ++i)
            state_[i] = other.state_[i];
    }

    XorshiftEngine<ResultType, R, A, B, C> &operator= (
            const XorshiftEngine<ResultType, R, A, B, C> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != R; ++i)
                state_[i] = other.state_[i];
        }

        return *this;
    }

    void seed (result_type s)
    {
        result_type seed = s;
        for (std::size_t i = 0; i != R; ++i) {
            internal::xorshift<ResultType, A, B, C>(&seed, Position<1>());
            state_[0] = s;
        }
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq) {seq.generate(state_, state_ + R);}

    result_type operator() ()
    {
        internal::xorshift<ResultType, A, B, C>(state_, Position<R>());

        return state_[R - 1];
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
            const XorshiftEngine<ResultType, R, A, B, C> &eng1,
            const XorshiftEngine<ResultType, R, A, B, C> &eng2)
    {
        for (std::size_t i = 0; i != R; ++i) {
            if (eng1.state_[i] != eng2.state_[i])
                return false;
        }

        return true;
    }

    friend inline bool operator!= (
            const XorshiftEngine<ResultType, R, A, B, C> &eng1,
            const XorshiftEngine<ResultType, R, A, B, C> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const XorshiftEngine<ResultType, R, A, B, C> &eng)
    {
        for (std::size_t i = 0; i != R - 1; ++i)
            os << eng.state_[i] << ' ';
        os << eng.state_[R - 1];

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            XorshiftEngine<ResultType, R, A, B, C> &eng)
    {
        result_type state[R];
        for (std::size_t i = 0; i != R; ++i) {
            if(!(is >> std::ws >> state[i]))
                break;
        }
        if (is) {
            for (std::size_t i = 0; i != R; ++i)
                eng.state_[i] = state[i];
        }

        return is;
    }

    private :

    result_type state_[R];
}; // class XorshiftEngine

/// \brief Xorshift RNG engine generating \f$2^32 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 1, 13, 17,  5> Xorshift1x32;

/// \brief Xorshift RNG engine generating \f$2^64 - 1\f$ 64-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint64_t, 1, 13,  7, 17> Xorshift1x64;

/// \brief Xorshift RNG engine generating \f$2^64 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 2,  2,  7,  3> Xorshift2x32;

/// \brief Xorshift RNG engine generating \f$2^96 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 3, 13, 19,  3> Xorshift3x32;

/// \brief Xorshift RNG engine generating \f$2^128 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 4,  5, 14,  1> Xorshift4x32;

/// \brief Xorshift RNG engine generating \f$2^160 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef XorshiftEngine<uint32_t, 5,  7, 13,  6> Xorshift5x32;

/// \brief Xorshift RNG engine generating \f$2^160 - 1\f$ 32-bits integers
/// \ingroup RNG
typedef Xorshift5x32 Xorshift;

} // namespace vsmc

#endif // VSMC_RNG_XORSHIFT_HPP
