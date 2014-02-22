#ifndef VSMC_RNG_XORSHIFT_HPP
#define VSMC_RNG_XORSHIFT_HPP

#include <vsmc/rng/seed.hpp>
#include <stdint.h>

namespace vsmc {

namespace internal {

template <typename ResultType>
void xorshift_assign (ResultType *state, Position<2>)
{
    state[0] = state[1];
}

template <typename ResultType, std::size_t Pos>
void xorshift_assign (ResultType *state, Position<Pos>)
{
    state[0] = state[1];
    xorshift_assign(state + 1, Position<Pos - 1>());
}

template <typename ResultType, ResultType A, ResultType B, ResultType C>
inline void xorshift (ResultType *state, Position<1>)
{
    *state ^= (*state)<<A;
    *state ^= (*state)>>B;
    *state ^= (*state)>>C;
}

template <typename ResultType, ResultType A, ResultType B, ResultType C,
    std::size_t Pos>
inline void xorshift (ResultType *state, Position<Pos>)
{
    ResultType t = state[0];
    t ^= t<<A;
    t ^= t>>B;
    xorshift_assign(state, Position<Pos>());
    state[Pos - 1] = (state[Pos - 1]^(state[Pos - 1]>>C))^t;
}

}; // namespace vsmc::internal

/// \brief Xorshift RNG engine
/// \ingroup RNG
template <typename ResultType, std::size_t Round,
         ResultType A, ResultType B, ResultType C>
class XorshiftEngine
{
    public :

    typedef ResultType result_type;

    explicit XorshiftEngine (result_type s = 123456)
    {
        result_type seed = s;
        for (std::size_t i = 0; i != Round; ++i) {
            internal::xorshift<ResultType, A, B, C>(&seed, Position<1>());
            state_[0] = s;
        }
    }

    template <typename SeedSeq>
    explicit XorshiftEngine (SeedSeq &seq)
    {seq.generate(state_, state_ + Round);}

    XorshiftEngine (const XorshiftEngine<ResultType, Round, A, B, C> &other)
    {
        for (std::size_t i = 0; i != Round; ++i)
            state_[i] = other.state_[i];
    }

    XorshiftEngine (XorshiftEngine<ResultType, Round, A, B, C> &other)
    {
        for (std::size_t i = 0; i != Round; ++i)
            state_[i] = other.state_[i];
    }

    XorshiftEngine<ResultType, Round, A, B, C> &operator= (
            const XorshiftEngine<ResultType, Round, A, B, C> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != Round; ++i)
                state_[i] = other.state_[i];
        }

        return *this;
    }

    void seed (result_type s)
    {
        result_type seed = s;
        for (std::size_t i = 0; i != Round; ++i) {
            internal::xorshift<ResultType, A, B, C>(&seed, Position<1>());
            state_[0] = s;
        }
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq) {seq.generate(state_, state_ + Round);}

    result_type operator() ()
    {
        internal::xorshift<ResultType, A, B, C>(state_, Position<Round>());

        return state_[Round - 1];
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
            const XorshiftEngine<ResultType, Round, A, B, C> &eng1,
            const XorshiftEngine<ResultType, Round, A, B, C> &eng2)
    {
        for (std::size_t i = 0; i != Round; ++i) {
            if (eng1.state_[i] != eng2.state_[i])
                return false;
        }

        return true;
    }

    friend inline bool operator!= (
            const XorshiftEngine<ResultType, Round, A, B, C> &eng1,
            const XorshiftEngine<ResultType, Round, A, B, C> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const XorshiftEngine<ResultType, Round, A, B, C> &eng)
    {
        for (std::size_t i = 0; i != Round - 1; ++i)
            os << eng.state_ << ' ';
        os << eng.state_[Round - 1];

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            XorshiftEngine<ResultType, Round, A, B, C> &eng)
    {
        result_type s;
        for (std::size_t i = 0; i != Round; ++i) {
            if (is >> std::ws >> s)
                eng.state_[i] = s;
            else
                break;
        }

        return is;
    }

    private :

    result_type state_[Round];
}; // class Xorshift

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
