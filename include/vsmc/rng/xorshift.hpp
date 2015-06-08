//============================================================================
// vSMC/include/vsmc/rng/xorshift.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RNG_XORSHIFT_HPP
#define VSMC_RNG_XORSHIFT_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(K)                              \
    VSMC_STATIC_ASSERT(                                                       \
        (K != 0), "**XorshiftEngine USEd WITH ORDER EUQAL TO ZERO")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType)                  \
    VSMC_STATIC_ASSERT((std::is_unsigned<ResultType>::value),                 \
        "**XorshiftEngine** USED WITH ResultType NOT AN UNSIGNED INTEGER "    \
        "TYPE")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType)                 \
    VSMC_STATIC_ASSERT((sizeof(ResultType) >= sizeof(std::uint32_t)),         \
        "**XorshiftEngine** USED WITH ResultType SMALLER THAN "               \
        "std::uint32_t")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(I, K)                           \
    VSMC_STATIC_ASSERT((I != 0 || K == 1),                                    \
        "**XorshiftEngine** USED WITH INDEX " #I " EQUAL TO ZERO")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX_ORDER(R, S, K)                  \
    VSMC_STATIC_ASSERT((R > S || K == 1),                                     \
        "**XorshiftEngine** USED WITH INDEX " #R " NOT LARGER THAN " #S)

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(A)                         \
    VSMC_STATIC_ASSERT((A != 0),                                              \
        "**XorshiftEngine** USED WITH SHIFT BITS " #A " EQUAL TO ZERO")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_C(C, K)                    \
    VSMC_STATIC_ASSERT((C != 0 || K != 1),                                    \
        "**XorshiftEngine** USED WTIH SHIFT BITS " #C " EQUAL TO ZERO")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_D(D, K)                    \
    VSMC_STATIC_ASSERT((D != 0 || K == 1),                                    \
        "**XorshiftEngine** USED WTIH SHIFT BITS " #D " EQUAL TO ZERO")

#define VSMC_STATIC_ASSERT_RNG_XORSHIFT                                       \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_ORDER(K);                                 \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_UNSIGNED(ResultType);                     \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_UINT_SIZE(ResultType);                    \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(R, K);                              \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX(S, K);                              \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_INDEX_ORDER(R, S, K);                     \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(A);                            \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS(B);                            \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_C(C, K);                       \
    VSMC_STATIC_ASSERT_RNG_XORSHIFT_SHIFT_BITS_D(D, K);

namespace vsmc
{

namespace internal
{

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void xorshift_left_assign(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void xorshift_left_assign(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = std::get<I + A>(state);
    xorshift_left_assign<K, A, I + 1>(
        state, std::integral_constant<bool, (I + A + 1 < K)>());
}

template <std::size_t K, std::size_t, typename T>
inline void xorshift_left_zero(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t I, typename T>
inline void xorshift_left_zero(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = 0;
    xorshift_left_zero<K, I + 1>(
        state, std::integral_constant<bool, (I + 1 < K)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void xorshift_left_shift(std::array<T, K> &state)
{
    xorshift_left_assign<K, A, 0>(
        state, std::integral_constant<bool, (A > 0 && A < K)>());
    xorshift_left_zero<K, K - A>(
        state, std::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void xorshift_right_assign(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void xorshift_right_assign(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = std::get<I - A>(state);
    xorshift_right_assign<K, A, I - 1>(
        state, std::integral_constant<bool, (A < I)>());
}

template <std::size_t K, std::size_t, typename T>
inline void xorshift_right_zero(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t I, typename T>
inline void xorshift_right_zero(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = 0;
    xorshift_right_zero<K, I - 1>(
        state, std::integral_constant<bool, (I > 0)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void xorshift_right_shift(std::array<T, K> &state)
{
    xorshift_right_assign<K, A, K - 1>(
        state, std::integral_constant<bool, (A > 0 && A < K)>());
    xorshift_right_zero<K, A - 1>(
        state, std::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <bool, typename ResultType, unsigned>
class XorshiftLeft
{
    public:
    static ResultType eval(ResultType x) { return x; }
}; // class XorshiftLeft

template <typename ResultType, unsigned A>
class XorshiftLeft<true, ResultType, A>
{
    public:
    static ResultType eval(ResultType x) { return x ^ (x << A); }
}; // class XorshiftLeft

template <bool, typename ResultType, unsigned>
class XorshiftRight
{
    public:
    static ResultType eval(ResultType x) { return x; }
}; // class XorshiftRight

template <typename ResultType, unsigned A>
class XorshiftRight<true, ResultType, A>
{
    public:
    static ResultType eval(ResultType x) { return x ^ (x >> A); }
}; // class XorshiftRight

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S,
    bool = (K <= 8)>
class XorshiftIndex
{
    public:
    void reset() {}

    static constexpr std::size_t r() { return K - R; }
    static constexpr std::size_t s() { return K - S; }
    static constexpr std::size_t k() { return K - 1; }

    static void eval(std::array<ResultType, K> &state)
    {
        xorshift_left_shift<K, 1, false>(state);
    }
}; // class XorshiftIndex

template <typename ResultType, std::size_t K, std::size_t R, std::size_t S>
class XorshiftIndex<ResultType, K, R, S, false>
{
    public:
    XorshiftIndex() : iter_(0) {}

    void reset() { iter_ = 0; }

    std::size_t r() { return (K - R + iter_) % K; }
    std::size_t s() { return (K - S + iter_) % K; }
    std::size_t k() { return (K - 1 + iter_) % K; }

    void eval(std::array<ResultType, K> &) { iter_ = (iter_ + 1) % K; }

    private:
    std::size_t iter_;
}; // class XorshiftIndex

template <unsigned A, unsigned B, unsigned C, unsigned, typename ResultType,
    std::size_t R, std::size_t S>
inline ResultType xorshift(
    std::array<ResultType, 1> &state, XorshiftIndex<ResultType, 1, R, S> &)
{
    state.front() ^= (state.front()) << A;
    state.front() ^= (state.front()) >> B;
    state.front() ^= (state.front()) << C;

    return state.front();
}

template <unsigned A, unsigned B, unsigned C, unsigned D, typename ResultType,
    std::size_t K, std::size_t R, std::size_t S>
inline ResultType xorshift(std::array<ResultType, K> &state,
    XorshiftIndex<ResultType, K, R, S> &index)
{
    ResultType xr = state[index.r()];
    xr = XorshiftLeft<A != 0, ResultType, A>::eval(xr);
    xr = XorshiftRight<B != 0, ResultType, B>::eval(xr);

    ResultType xs = state[index.s()];
    xs = XorshiftLeft<C != 0, ResultType, C>::eval(xs);
    xs = XorshiftRight<D != 0, ResultType, D>::eval(xs);

    index.eval(state);

    return state[index.k()] = xs ^ xr;
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
template <typename ResultType, std::size_t K, unsigned A, unsigned B,
    unsigned C, unsigned D, std::size_t R, std::size_t S>
class XorshiftEngine
{
    public:
    using result_type = ResultType;

    explicit XorshiftEngine(result_type s = 1)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        if (s == 0)
            s = 1;
        seed(s);
    }

    template <typename SeedSeq>
    explicit XorshiftEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorshiftEngine<ResultType, K, A, B, C, D, R, S>>::value>::type * =
            nullptr)
    {
        VSMC_STATIC_ASSERT_RNG_XORSHIFT;
        seed(seq);
    }

    void seed(result_type s)
    {
        index_.reset();
        std::array<std::uint32_t, 1> seed;
        seed.front() = static_cast<std::uint32_t>(s % uint32_t_max_);
        internal::XorshiftIndex<std::uint32_t, 1, 0, 0> index;
        for (std::size_t i = 0; i != K; ++i)
            state_[i] = internal::xorshift<13, 17, 5, 0>(seed, index);
        discard(4 * K);
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorshiftEngine<ResultType, K, A, B, C, D, R, S>>::value>::type * =
            nullptr)
    {
        index_.reset();
        seq.generate(state_.begin(), state_.end());
        discard(4 * K);
    }

    result_type operator()()
    {
        return internal::xorshift<A, B, C, D>(state_, index_);
    }

    void discard(std::size_t nskip)
    {
        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(
        const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
        const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {
        return eng1.state_ == eng2.state_;
    }

    friend bool operator!=(
        const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng1,
        const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {
        if (!os.good())
            return os;

        os << eng.state_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        XorshiftEngine<ResultType, K, A, B, C, D, R, S> &eng)
    {
        if (!is.good())
            return is;

        std::array<ResultType, K> tmp;
        is >> std::ws >> tmp;

        if (is.good())
            eng.state_ = std::move(tmp);

        return is;
    }

    private:
    internal::XorshiftIndex<ResultType, K, R, S> index_;
    std::array<ResultType, K> state_;

    static constexpr result_type uint32_t_max_ =
        static_cast<result_type>(VSMC_MAX_UINT(std::uint32_t));
}; // class XorshiftEngine

/// \brief Xorwow RNG engine
/// \ingroup Xorshift
///
/// \details
/// Use Marsaglia's Xorwow algorithm with an Xorshift engine.
template <typename Eng, typename Eng::result_type D = 362437,
    typename Eng::result_type DInit = 6615241>
class XorwowEngine
{
    public:
    using result_type = typename Eng::result_type;
    using engine_type = Eng;

    explicit XorwowEngine(result_type s = 1) : eng_(s), weyl_(DInit) {}

    template <typename SeedSeq>
    explicit XorwowEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorwowEngine<Eng, D, DInit>>::value>::type * = nullptr)
        : eng_(seq), weyl_(DInit)
    {
    }

    void seed(result_type s)
    {
        eng_.seed(s);
        weyl_ = DInit;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            XorwowEngine<Eng, D, DInit>>::value>::type * = nullptr)
    {
        eng_.seed(seq);
        weyl_ = DInit;
    }

    result_type operator()() { return eng_() + (weyl_ += D); }

    void discard(std::size_t nskip)
    {
        eng_.discard(nskip);
        weyl_ += D * nskip;
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(const XorwowEngine<Eng, D, DInit> &eng1,
        const XorwowEngine<Eng, D, DInit> &eng2)
    {
        return eng1.eng_ == eng2.eng_ && eng1.weyl_ == eng2.weyl_;
    }

    friend bool operator!=(const XorwowEngine<Eng, D, DInit> &eng1,
        const XorwowEngine<Eng, D, DInit> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const XorwowEngine<Eng, D, DInit> &eng)
    {
        if (!os.good())
            return os;

        os << eng.eng_ << ' ' << eng.weyl_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        XorwowEngine<Eng, D, DInit> &eng)
    {
        if (!is.good())
            return is;

        engine_type eng_tmp;
        result_type weyl_tmp = 0;
        is >> std::ws >> eng_tmp;
        is >> std::ws >> weyl_tmp;

        if (is.good()) {
            eng.eng_ = std::move(eng_tmp);
            eng.weyl_ = weyl_tmp;
        }

        return is;
    }

    private:
    Eng eng_;
    result_type weyl_;
}; // class XorwowEngine

/// \brief Xorshift RNG engine generating \f$2^{32}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift1x32 = XorshiftEngine<std::uint32_t, 1, 13, 17, 5, 0, 0, 0>;

/// \brief Xorshift RNG engine generating \f$2^{64}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift1x64 = XorshiftEngine<std::uint64_t, 1, 13, 7, 17, 0, 0, 0>;

/// \brief Xorshift RNG engine generating \f$2^{64}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift2x32 = XorshiftEngine<std::uint32_t, 2, 17, 14, 12, 19, 2, 1>;

/// \brief Xorshift RNG engine generating \f$2^{128}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift4x32 = XorshiftEngine<std::uint32_t, 4, 15, 14, 12, 17, 4, 3>;

/// \brief Xorshift RNG engine generating \f$2^{256}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift8x32 = XorshiftEngine<std::uint32_t, 8, 18, 13, 14, 15, 8, 3>;

/// \brief Xorshift RNG engine generating \f$2^{512}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift16x32 = XorshiftEngine<std::uint32_t, 16, 17, 15, 13, 14, 16, 1>;

/// \brief Xorshift RNG engine generating \f$2^{1024}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift32x32 =
    XorshiftEngine<std::uint32_t, 32, 19, 11, 13, 16, 32, 15>;

/// \brief Xorshift RNG engine generating \f$2^{2048}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift64x32 =
    XorshiftEngine<std::uint32_t, 64, 19, 12, 14, 15, 64, 59>;

/// \brief Xorshift RNG engine generating \f$2^{4096}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorshift128x32 =
    XorshiftEngine<std::uint32_t, 128, 17, 12, 13, 15, 128, 95>;

/// \brief Xorshift RNG engine generating \f$2^{128}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift2x64 = XorshiftEngine<std::uint64_t, 2, 33, 31, 28, 29, 2, 1>;

/// \brief Xorshift RNG engine generating \f$2^{256}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift4x64 = XorshiftEngine<std::uint64_t, 4, 37, 27, 29, 33, 4, 3>;

/// \brief Xorshift RNG engine generating \f$2^{512}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift8x64 = XorshiftEngine<std::uint64_t, 8, 37, 26, 29, 34, 8, 1>;

/// \brief Xorshift RNG engine generating \f$2^{1024}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift16x64 = XorshiftEngine<std::uint64_t, 16, 34, 29, 25, 31, 16, 7>;

/// \brief Xorshift RNG engine generating \f$2^{2048}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift32x64 = XorshiftEngine<std::uint64_t, 32, 35, 27, 26, 37, 32, 1>;

/// \brief Xorshift RNG engine generating \f$2^{4096}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorshift64x64 =
    XorshiftEngine<std::uint64_t, 64, 33, 26, 27, 29, 64, 53>;

/// \brief The default 32-bits Xorshift RNG engine
/// \ingroup Xorshift
using Xorshift = Xorshift128x32;

/// \brief The default 64-bits Xorshift RNG engine
/// \ingroup Xorshift
using Xorshift_64 = Xorshift64x64;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{32}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow1x32 = XorwowEngine<Xorshift1x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{64}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow1x64 = XorwowEngine<Xorshift1x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{64}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow2x32 = XorwowEngine<Xorshift2x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{128}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow4x32 = XorwowEngine<Xorshift4x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{256}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow8x32 = XorwowEngine<Xorshift8x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{512}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow16x32 = XorwowEngine<Xorshift16x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{1024}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow32x32 = XorwowEngine<Xorshift32x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{2048}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow64x32 = XorwowEngine<Xorshift64x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{4096}-1\f$ 32-bits integers
/// \ingroup Xorshift
using Xorwow128x32 = XorwowEngine<Xorshift128x32>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{128}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow2x64 = XorwowEngine<Xorshift2x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{256}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow4x64 = XorwowEngine<Xorshift4x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{512}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow8x64 = XorwowEngine<Xorshift8x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{1024}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow16x64 = XorwowEngine<Xorshift16x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{2048}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow32x64 = XorwowEngine<Xorshift32x64>;

/// \brief Xorwow RNG engine using Xorshfit \f$2^{4096}-1\f$ 64-bits integers
/// \ingroup Xorshift
using Xorwow64x64 = XorwowEngine<Xorshift64x64>;

/// \brief The default 32-bits Xorwow RNG engine
/// \ingroup Xorshift
using Xorwow = Xorwow128x32;

/// \brief The default 64-bits Xorwow RNG engine
/// \ingroup Xorshift
using Xorwow_64 = Xorwow64x64;

} // namespace vsmc

#endif // VSMC_RNG_XORSHIFT_HPP
