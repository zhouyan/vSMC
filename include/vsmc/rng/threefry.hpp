//============================================================================
// vSMC/include/vsmc/rng/threefry.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_RNG_THREEFRY_HPP
#define VSMC_RNG_THREEFRY_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/counter.hpp>

#define VSMC_DEFINE_RNG_THREEFRY_KS_CONSTANT(W, val)                          \
    template <typename T>                                                     \
    class ThreefryKSConstant<T, W>                                            \
        : public std::integral_constant<T, UINT##W##_C(val)>                  \
    {                                                                         \
    }; // class ThreefryKSConstant

#define VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(W, K, N, I, val)             \
    template <typename T>                                                     \
    class ThreefryRotateConstant<T, K, N, I, W>                               \
        : public std::integral_constant<int, val>                             \
    {                                                                         \
    }; // class ThreefryRotateConstant

/// \brief ThreefryGenerator default rounds
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_ROUNDS
#define VSMC_RNG_THREEFRY_ROUNDS 20
#endif

/// \brief ThreefryGenerator default vector length
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_VECTOR_LENGTH
#define VSMC_RNG_THREEFRY_VECTOR_LENGTH 4
#endif

namespace vsmc
{

namespace internal
{

template <typename T, int = std::numeric_limits<T>::digits>
class ThreefryKSConstant;

VSMC_DEFINE_RNG_THREEFRY_KS_CONSTANT(32, 0x1BD11BDA)
VSMC_DEFINE_RNG_THREEFRY_KS_CONSTANT(64, 0x1BD11BDAA9FC1A22)

template <typename T, std::size_t, std::size_t, std::size_t,
    int = std::numeric_limits<T>::digits>
class ThreefryRotateConstant;

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 0, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 1, 0, 15)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 2, 0, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 3, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 4, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 5, 0, 29)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 6, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 2, 7, 0, 24)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 0, 0, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 1, 0, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 2, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 3, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 4, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 5, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 6, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 7, 0, 18)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 0, 1, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 1, 1, 21)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 2, 1, 27)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 3, 1, 5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 4, 1, 20)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 5, 1, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 6, 1, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(32, 4, 7, 1, 20)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 0, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 1, 0, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 2, 0, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 3, 0, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 4, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 5, 0, 32)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 6, 0, 24)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 2, 7, 0, 21)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 0, 0, 14)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 1, 0, 52)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 2, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 3, 0, 5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 4, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 5, 0, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 6, 0, 58)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 7, 0, 32)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 0, 1, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 1, 1, 57)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 2, 1, 40)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 3, 1, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 4, 1, 33)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 5, 1, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 6, 1, 22)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 4, 7, 1, 32)

template <typename T, int R>
class ThreefryRotateImpl
{
    public:
    static T eval(const T &x)
    {
        return (x << R) | (x >> (std::numeric_limits<T>::digits - R));
    }
}; // class ThreefryRotateImpl

template <typename T, std::size_t K, std::size_t N, bool = (N > 0)>
class ThreefryRotate
{
    public:
    static void eval(std::array<T, K> &) {}
}; // class ThreefryRotate

template <typename T, std::size_t N>
class ThreefryRotate<T, 2, N, true>
{
    public:
    static void eval(std::array<T, 2> &state)
    {
        std::get<0>(state) += std::get<1>(state);
        std::get<1>(state) =
            ThreefryRotateImpl<T, ThreefryRotateConstant<T, 2, r_,
                                      0>::value>::eval(std::get<1>(state));
        std::get<1>(state) ^= std::get<0>(state);
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // class ThreefryRotate

template <typename T, std::size_t N>
class ThreefryRotate<T, 4, N, true>
{
    public:
    static void eval(std::array<T, 4> &state)
    {
        std::get<0>(state) += std::get<i0_>(state);
        std::get<i0_>(state) =
            ThreefryRotateImpl<T, ThreefryRotateConstant<T, 4, r_,
                                      0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) ^= std::get<0>(state);

        std::get<2>(state) += std::get<i2_>(state);
        std::get<i2_>(state) =
            ThreefryRotateImpl<T, ThreefryRotateConstant<T, 4, r_,
                                      1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) ^= std::get<2>(state);
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // class ThreefryRotate

template <typename T, std::size_t Inc>
class ThreefryInsertKeyInc
    : public std::integral_constant<T, static_cast<T>(Inc)>
{
}; // class ThreefryInsertKeyInc

template <typename T, std::size_t K, std::size_t N, bool = (N % 4 == 0)>
class ThreefryInsertKey
{
    public:
    static void eval(std::array<T, K> &, const std::array<T, K + 1> &) {}
}; // class ThreefryInsertKey

template <typename T, std::size_t N>
class ThreefryInsertKey<T, 2, N, true>
{
    public:
    static void eval(std::array<T, 2> &state, const std::array<T, 3> &par)
    {
        std::get<0>(state) += std::get<i0_>(par);
        std::get<1>(state) += std::get<i1_>(par);
        std::get<1>(state) += ThreefryInsertKeyInc<T, inc_>::value;
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // class ThreefryInsertKey

template <typename T, std::size_t N>
class ThreefryInsertKey<T, 4, N, true>
{
    public:
    static void eval(std::array<T, 4> &state, const std::array<T, 5> &par)
    {
        std::get<0>(state) += std::get<i0_>(par);
        std::get<1>(state) += std::get<i1_>(par);
        std::get<2>(state) += std::get<i2_>(par);
        std::get<3>(state) += std::get<i3_>(par);
        std::get<3>(state) += ThreefryInsertKeyInc<T, inc_>::value;
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // class ThreefryInsertKey

} // namespace vsmc::internal

/// \brief Threefry RNG generator
/// \ingroup Threefry
template <typename ResultType, typename T = ResultType,
    std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**ThreefryGenerator** USED WITH ResultType OTHER THAN UNSIGNED "
        "INTEGER TYPES");

    static_assert(sizeof(T) * K % sizeof(ResultType) == 0,
        "**ThreefryGenerator** USED WITH sizeof(T) * K NOT DIVISIBLE BY "
        "sizeof(ResultType)");

    static_assert(std::is_unsigned<T>::value,
        "**ThreefryGenerator** USED WITH T OTHER THAN UNSIGNED INTEGER TYPES");

    static_assert(std::numeric_limits<T>::digits == 32 ||
            std::numeric_limits<T>::digits == 64,
        "**ThreefryGenerator** USED WITH T OF SIZE OTHER THAN 32 OR 64 BITS");

    static_assert(K == 2 || K == 4,
        "**ThreefryGenerator** USED WITH K OTHER THAN 2 OR 4");

    static_assert(
        Rounds != 0, "**ThreefryGenerator** USED WITH ROUNDS EQUAL TO ZERO");

    public:
    using result_type = ResultType;
    using ctr_type =
        std::array<std::uint64_t, sizeof(T) * K / sizeof(std::uint64_t)>;
    using key_type = std::array<T, K>;

    ThreefryGenerator() : par_(0) {}

    static constexpr std::size_t size()
    {
        return sizeof(T) * K / sizeof(ResultType);
    }

    void reset(const key_type &key)
    {
        par_ = internal::ThreefryKSConstant<T>::value;
        for (std::size_t i = 0; i != key.size(); ++i)
            par_ ^= key[i];
    }

    void operator()(ctr_type &ctr, const key_type &key,
        std::array<ResultType, size()> &buffer) const
    {
        union {
            std::array<T, K> state;
            ctr_type ctr;
            std::array<ResultType, size()> result;
        } buf;

        std::array<T, K + 1> par;
        std::copy(key.begin(), key.end(), par.begin());
        par.back() = par_;

        increment(ctr);
        buf.ctr = ctr;
        generate<0>(buf.state, par, std::true_type());
        buffer = buf.result;
    }

    void operator()(ctr_type &ctr, const key_type &key, std::size_t n,
        std::array<ResultType, size()> *buffer) const
    {
        union {
            std::array<T, K> state;
            ctr_type ctr;
            std::array<ResultType, size()> result;
        } buf;

        std::array<T, K + 1> par;
        std::copy(key.begin(), key.end(), par.begin());
        par.back() = par_;

        for (std::size_t i = 0; i != n; ++i) {
            increment(ctr);
            buf.ctr = ctr;
            generate<0>(buf.state, par, std::true_type());
            buffer[i] = buf.result;
        }
    }

    private:
    template <std::size_t>
    void generate(std::array<T, K> &, const std::array<T, K + 1> &,
        std::false_type) const
    {
    }

    template <std::size_t N>
    void generate(std::array<T, K> &state, const std::array<T, K + 1> &par,
        std::true_type) const
    {
        internal::ThreefryRotate<T, K, N>::eval(state);
        internal::ThreefryInsertKey<T, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant<bool, (N < Rounds)>());
    }

    private:
    T par_;
}; // class ThreefryGenerator

/// \brief Threefry RNG engine
/// \ingroup Threefry
template <typename ResultType, typename T,
    std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
using ThreefryEngine =
    CounterEngine<ThreefryGenerator<ResultType, T, K, Rounds>>;

/// \brief Threefry2x32 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry2x32Engine = ThreefryEngine<ResultType, std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry4x32Engine = ThreefryEngine<ResultType, std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry2x64Engine = ThreefryEngine<ResultType, std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry4x64Engine = ThreefryEngine<ResultType, std::uint64_t, 4>;

/// \brief Threefry2x32 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry2x32 = Threefry2x32Engine<std::uint32_t>;

/// \brief Threefry4x32 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry4x32 = Threefry4x32Engine<std::uint32_t>;

/// \brief Threefry2x64 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry2x64 = Threefry2x64Engine<std::uint32_t>;

/// \brief Threefry4x64 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry4x64 = Threefry4x64Engine<std::uint32_t>;

/// \brief Threefry2x32 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry2x32_64 = Threefry2x32Engine<std::uint64_t>;

/// \brief Threefry4x32 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry4x32_64 = Threefry4x32Engine<std::uint64_t>;

/// \brief Threefry2x64 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry2x64_64 = Threefry2x64Engine<std::uint64_t>;

/// \brief Threefry4x64 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry4x64_64 = Threefry4x64Engine<std::uint64_t>;

/// \brief The default 32-bit Threefry engine
/// \ingroup Threefry
using Threefry = Threefry4x64Engine<std::uint32_t>;

/// \brief The default 64-bit Threefry engine
/// \ingroup Threefry
using Threefry_64 = Threefry4x64Engine<std::uint64_t>;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
