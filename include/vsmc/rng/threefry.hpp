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

#define VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(T, K, N, I, val)             \
    template <>                                                               \
    class ThreefryRotateConstant<T, K, N, I>                                  \
        : public std::integral_constant<int, val>                             \
    {                                                                         \
    }; // class ThreefryRotateConstant

/// \brief ThreefryGenerator default vector length
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_VECTOR_LENGTH
#define VSMC_RNG_THREEFRY_VECTOR_LENGTH 4
#endif

/// \brief ThreefryGenerator default rounds
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_ROUNDS
#define VSMC_RNG_THREEFRY_ROUNDS 20
#endif

namespace vsmc
{

namespace internal
{

template <typename>
class ThreefryKSConstant;

template <typename T, template <typename> class SIMD>
class ThreefryKSConstant<SIMD<T>> : public ThreefryKSConstant<T>
{
}; // class ThreefryKSConstant

template <>
class ThreefryKSConstant<std::uint32_t>
    : public std::integral_constant<std::uint32_t, UINT32_C(0x1BD11BDA)>
{
}; // class ThreefryKSConstant

template <>
class ThreefryKSConstant<std::uint64_t>
    : public std::integral_constant<std::uint64_t,
          UINT64_C(0x1BD11BDAA9FC1A22)>
{
}; // class ThreefryKSConstant

template <typename, std::size_t, std::size_t, std::size_t>
class ThreefryRotateConstant;

template <typename T, template <typename> class SIMD, std::size_t K,
    std::size_t N, std::size_t I>
class ThreefryRotateConstant<SIMD<T>, K, N, I>
    : public ThreefryRotateConstant<T, K, N, I>
{
}; // class ThreefryRotateConstant

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 0, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 1, 0, 15)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 2, 0, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 3, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 4, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 5, 0, 29)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 6, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 2, 7, 0, 24)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 0, 0, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 1, 0, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 2, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 3, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 4, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 5, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 6, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 7, 0, 18)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 0, 1, 26)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 1, 1, 21)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 2, 1, 27)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 3, 1, 5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 4, 1, 20)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 5, 1, 11)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 6, 1, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint32_t, 4, 7, 1, 20)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 0, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 1, 0, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 2, 0, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 3, 0, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 4, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 5, 0, 32)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 6, 0, 24)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 2, 7, 0, 21)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 0, 0, 14)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 1, 0, 52)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 2, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 3, 0, 5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 4, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 5, 0, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 6, 0, 58)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 7, 0, 32)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 0, 1, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 1, 1, 57)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 2, 1, 40)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 3, 1, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 4, 1, 33)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 5, 1, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 6, 1, 22)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(std::uint64_t, 4, 7, 1, 32)

template <typename T, std::size_t K>
class ThreefryInitPar
{
    public:
    static void eval(const std::array<T, K> &key, std::array<T, K + 1> &par)
    {
        par.back() = ThreefryKSConstant<T>::value;
        par_xor<0>(key, par, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t>
    static void par_xor(
        const std::array<T, K> &, std::array<T, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void par_xor(
        const std::array<T, K> &key, std::array<T, K + 1> &par, std::true_type)
    {
        std::get<N>(par) = std::get<N>(key);
        par.back() ^= std::get<N>(key);
        par_xor<N + 1>(key, par, std::integral_constant<bool, N + 1 < K>());
    }
}; // class ThreefryInitPar

template <typename T>
class ThreefryRotateBits : public std::integral_constant<int, sizeof(T) * 8>
{
}; // class ThreefryRotateBits

template <typename T, template <typename> class SIMD>
class ThreefryRotateBits<SIMD<T>> : public ThreefryRotateBits<T>
{
}; // class ThreefryRotateBits

template <typename T, int R>
class ThreefryRotateImpl
{
    public:
    static T eval(const T &x)
    {
        return (x << R) | (x >> (ThreefryRotateBits<T>::value - R));
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

template <typename T, template <typename> class SIMD, std::size_t Inc>
class ThreefryInsertKeyInc<SIMD<T>, Inc> : public ThreefryInsertKeyInc<T, Inc>
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
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**ThreefryGenerator** USED WITH ResultType OTHER THAN UNSIGNED "
        "INTEGER TYPES");

    static_assert(sizeof(ResultType) == sizeof(std::uint32_t) ||
            sizeof(ResultType) == sizeof(std::uint64_t),
        "**ThreefryGenerator** USED WITH ResultType OF SIZE OTHER THAN 32 OR "
        "64 BITS");

    static_assert(K == 2 || K == 4,
        "**ThreefryGenerator** USED WITH K OTHER THAN 2 OR 4");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, K>;
    using key_type = std::array<ResultType, K>;

    static constexpr std::size_t size() { return K; }

    void reset(const key_type &) {}

    void operator()(ctr_type &ctr, const key_type &key,
        std::array<result_type, K> &buffer) const
    {
        std::array<result_type, K + 1> par;
        internal::ThreefryInitPar<ResultType, K>::eval(key, par);
        increment(ctr);
        buffer = ctr;
        generate<0>(buffer, par, std::true_type());
    }

    std::size_t operator()(ctr_type &ctr, const key_type &key, std::size_t n,
        result_type *r) const
    {
        const std::size_t m = n / size();
        std::array<result_type, K + 1> par;
        internal::ThreefryInitPar<ResultType, K>::eval(key, par);
        ctr_type *s = reinterpret_cast<ctr_type *>(r);
        increment(ctr, m, s);
        for (std::size_t i = 0; i != m; ++i)
            generate<0>(s[i], par, std::true_type());

        return m * size();
    }

    private:
    template <std::size_t>
    void generate(std::array<result_type, K> &,
        const std::array<result_type, K + 1> &, std::false_type) const
    {
    }

    template <std::size_t N>
    void generate(std::array<result_type, K> &state,
        const std::array<result_type, K + 1> &par, std::true_type) const

    {
        internal::ThreefryRotate<ResultType, K, N>::eval(state);
        internal::ThreefryInsertKey<ResultType, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryGenerator

/// \brief Threefry RNG engine
/// \ingroup Threefry
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
using ThreefryEngine = CounterEngine<ThreefryGenerator<ResultType, K, Rounds>>;

/// \brief Threefry2x32 RNG engine
/// \ingroup Threefry
using Threefry2x32 = ThreefryEngine<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine
/// \ingroup Threefry
using Threefry4x32 = ThreefryEngine<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine
/// \ingroup Threefry
using Threefry2x64 = ThreefryEngine<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine
/// \ingroup Threefry
using Threefry4x64 = ThreefryEngine<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine
/// \ingroup Threefry
using Threefry = ThreefryEngine<std::uint32_t>;

/// \brief The default 64-bits Threefry engine
/// \ingroup Threefry
using Threefry_64 = ThreefryEngine<std::uint64_t>;

#if VSMC_HAS_SSE2

namespace internal
{

template <typename ResultType, std::size_t K>
class ThreefryParPackSSE2
{
    public:
    static void eval(const std::array<ResultType, K + 1> &p,
        std::array<M128I<ResultType>, K + 1> &par)
    {
        pack<0>(p, par, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const std::array<ResultType, K + 1> &,
        std::array<M128I<ResultType>, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<ResultType, K + 1> &p,
        std::array<M128I<ResultType>, K + 1> &par, std::true_type)
    {
        std::get<N>(par).set1(std::get<N>(p));
        pack<N + 1>(p, par, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // class ThreefryParPackSSE2

template <typename ResultType, std::size_t K>
class ThreefryCtrPackSSE2
{
    public:
    static void eval(std::array<ResultType, K> &ctr,
        std::array<M128I<ResultType>, K> &state)
    {
        std::array<std::array<ResultType, K>, M128I<ResultType>::size()>
            ctr_block;
        increment(ctr, ctr_block);
        pack<0>(ctr_block, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(const std::array<std::array<ResultType, K>,
                         M128I<ResultType>::size()> &,
        std::array<M128I<ResultType>, K> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<std::array<ResultType, K>,
                         M128I<ResultType>::size()> &ctr_block,
        std::array<M128I<ResultType>, K> &state, std::true_type)
    {
        set<N>(ctr_block, state,
            std::integral_constant<std::size_t, sizeof(ResultType)>());
        pack<N + 1>(
            ctr_block, state, std::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t N>
    static void set(const std::array<std::array<ResultType, K>,
                        M128I<ResultType>::size()> &ctr_block,
        std::array<M128I<ResultType>, K> &state,
        std::integral_constant<std::size_t, 4>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)),
            std::get<N>(std::get<2>(ctr_block)),
            std::get<N>(std::get<3>(ctr_block)));
    }

    template <std::size_t N>
    static void set(const std::array<std::array<ResultType, K>,
                        M128I<ResultType>::size()> &ctr_block,
        std::array<M128I<ResultType>, K> &state,
        std::integral_constant<std::size_t, 8>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)));
    }
}; // class ThreefryCtrPackSSE2

} // namespace vsmc::internal

/// \brief Threefry RNG generator using SSE2
/// \ingroup Threefry
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryGeneratorSSE2
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**ThreefryGeneratorSSE2** USED WITH ResultType OTHER THAN UNSIGNED "
        "INTEGER TYPES");

    static_assert(sizeof(ResultType) == sizeof(std::uint32_t) ||
            sizeof(ResultType) == sizeof(std::uint64_t),
        "**ThreefryGeneratorSSE2** USED WITH ResultType OF SIZE OTHER THAN "
        "32 OR 64 BITS");

    static_assert(K == 2 || K == 4,
        "**ThreefryGeneratorSSE2** USED WITH K OTHER THAN 2 OR 4");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, K>;
    using key_type = std::array<ResultType, K>;

    static constexpr std::size_t size()
    {
        return K * M128I<ResultType>::size();
    }

    void reset(const key_type &) {}

    void operator()(ctr_type &ctr, const key_type &key,
        std::array<result_type, K * M128I<ResultType>::size()> &buffer)
    {
        union {
            std::array<M128I<ResultType>, K> state;
            std::array<ResultType, size()> result;
        } buf;

        std::array<result_type, K + 1> p;
        std::array<M128I<ResultType>, K + 1> par;
        internal::ThreefryInitPar<ResultType, K>::eval(key, p);
        internal::ThreefryParPackSSE2<ResultType, K>::eval(p, par);
        internal::ThreefryCtrPackSSE2<ResultType, K>::eval(ctr, buf.state);
        generate<0>(buf.state, par, std::true_type());
        buffer = buf.result;
    }

    std::size_t operator()(
        ctr_type &, const key_type &, std::size_t, result_type *) const
    {
        return 0;
    }

    private:
    template <std::size_t>
    void generate(std::array<M128I<ResultType>, K> &,
        const std::array<M128I<ResultType>, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate(std::array<M128I<ResultType>, K> &state,
        const std::array<M128I<ResultType>, K + 1> &par, std::true_type)
    {
        internal::ThreefryRotate<M128I<ResultType>, K, N>::eval(state);
        internal::ThreefryInsertKey<M128I<ResultType>, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryGeneratorSSE2

/// \brief Threefry RNG engine using SSE2
/// \ingroup Threefry
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
using ThreefryEngineSSE2 =
    CounterEngine<ThreefryGeneratorSSE2<ResultType, K, Rounds>>;

/// \brief Threefry2x32 RNG engine using SSE2
/// \ingroup Threefry
using Threefry2x32SSE2 = ThreefryEngineSSE2<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine using SSE2
/// \ingroup Threefry
using Threefry4x32SSE2 = ThreefryEngineSSE2<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine using SSE2
/// \ingroup Threefry
using Threefry2x64SSE2 = ThreefryEngineSSE2<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine using SSE2
/// \ingroup Threefry
using Threefry4x64SSE2 = ThreefryEngineSSE2<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine using SSE2
/// \ingroup Threefry
using ThreefrySSE2 = ThreefryEngineSSE2<std::uint32_t>;

/// \brief The default 64-bits ThreefrySSE2 engine using SSE2
/// \ingroup Threefry
using ThreefrySSE2_64 = ThreefryEngineSSE2<std::uint64_t>;

#endif // VSMC_HAS_SSE2

#if VSMC_HAS_AVX2

namespace internal
{

template <typename ResultType, std::size_t K>
class ThreefryParPackAVX2
{
    public:
    static void eval(const std::array<ResultType, K + 1> &p,
        std::array<M256I<ResultType>, K + 1> &par)
    {
        pack<0>(p, par, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const std::array<ResultType, K + 1> &,
        std::array<M256I<ResultType>, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<ResultType, K + 1> &p,
        std::array<M256I<ResultType>, K + 1> &par, std::true_type)
    {
        std::get<N>(par).set1(std::get<N>(p));
        pack<N + 1>(p, par, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // class ThreefryParPackAVX2

template <typename ResultType, std::size_t K>
class ThreefryCtrPackAVX2
{
    public:
    static void eval(std::array<ResultType, K> &ctr,
        std::array<M256I<ResultType>, K> &state)
    {
        std::array<std::array<ResultType, K>, M256I<ResultType>::size()>
            ctr_block;
        increment(ctr, ctr_block);
        pack<0>(ctr_block, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(const std::array<std::array<ResultType, K>,
                         M256I<ResultType>::size()> &,
        std::array<M256I<ResultType>, K> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<std::array<ResultType, K>,
                         M256I<ResultType>::size()> &ctr_block,
        std::array<M256I<ResultType>, K> &state, std::true_type)
    {
        set<N>(ctr_block, state,
            std::integral_constant<std::size_t, sizeof(ResultType)>());
        pack<N + 1>(
            ctr_block, state, std::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t N>
    static void set(const std::array<std::array<ResultType, K>,
                        M256I<ResultType>::size()> &ctr_block,
        std::array<M256I<ResultType>, K> &state,
        std::integral_constant<std::size_t, 4>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)),
            std::get<N>(std::get<2>(ctr_block)),
            std::get<N>(std::get<3>(ctr_block)),
            std::get<N>(std::get<4>(ctr_block)),
            std::get<N>(std::get<5>(ctr_block)),
            std::get<N>(std::get<6>(ctr_block)),
            std::get<N>(std::get<7>(ctr_block)));
    }

    template <std::size_t N>
    static void set(const std::array<std::array<ResultType, K>,
                        M256I<ResultType>::size()> &ctr_block,
        std::array<M256I<ResultType>, K> &state,
        std::integral_constant<std::size_t, 8>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)),
            std::get<N>(std::get<2>(ctr_block)),
            std::get<N>(std::get<3>(ctr_block)));
    }
}; // class ThreefryCtrPackAVX2

} // namespace vsmc::internal

/// \brief Threefry RNG generator using AVX2
/// \ingroup Threefry
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryGeneratorAVX2
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**ThreefryGeneratorAVX2** USED WITH ResultType OTHER THAN UNSIGNED "
        "INTEGER TYPES");

    static_assert(sizeof(ResultType) == sizeof(std::uint32_t) ||
            sizeof(ResultType) == sizeof(std::uint64_t),
        "**ThreefryGeneratorAVX2** USED WITH ResultType OF SIZE OTHER THAN "
        "32 OR 64 BITS");

    static_assert(K == 2 || K == 4,
        "**ThreefryGeneratorAVX2** USED WITH K OTHER THAN 2 OR 4");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, K>;
    using key_type = std::array<ResultType, K>;

    static constexpr std::size_t size()
    {
        return K * M256I<ResultType>::size();
    }

    void reset(const key_type &) {}

    void operator()(ctr_type &ctr, const key_type &key,
        std::array<result_type, K * M256I<ResultType>::size()> &buffer)
    {
        union {
            std::array<M256I<ResultType>, K> state;
            std::array<ResultType, size()> result;
        } buf;

        std::array<result_type, K + 1> p;
        std::array<M256I<ResultType>, K + 1> par;
        internal::ThreefryInitPar<ResultType, K>::eval(key, p);
        internal::ThreefryParPackAVX2<ResultType, K>::eval(p, par);
        internal::ThreefryCtrPackAVX2<ResultType, K>::eval(ctr, buf.state);
        generate<0>(buf.state, par, std::true_type());
        buffer = buf.result;
    }

    std::size_t operator()(
        ctr_type &, const key_type &, std::size_t, result_type *) const
    {
        return 0;
    }

    private:
    template <std::size_t>
    void generate(std::array<M256I<ResultType>, K> &,
        const std::array<M256I<ResultType>, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate(std::array<M256I<ResultType>, K> &state,
        const std::array<M256I<ResultType>, K + 1> &par, std::true_type)
    {
        internal::ThreefryRotate<M256I<ResultType>, K, N>::eval(state);
        internal::ThreefryInsertKey<M256I<ResultType>, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryGeneratorAVX2

/// \brief Threefry RNG engine using AVX2
/// \ingroup Threefry
template <typename ResultType, std::size_t K = VSMC_RNG_THREEFRY_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
using ThreefryEngineAVX2 =
    CounterEngine<ThreefryGeneratorAVX2<ResultType, K, Rounds>>;

/// \brief Threefry2x32 RNG engine using AVX2
/// \ingroup Threefry
using Threefry2x32AVX2 = ThreefryEngineAVX2<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine using AVX2
/// \ingroup Threefry
using Threefry4x32AVX2 = ThreefryEngineAVX2<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine using AVX2
/// \ingroup Threefry
using Threefry2x64AVX2 = ThreefryEngineAVX2<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine using AVX2
/// \ingroup Threefry
using Threefry4x64AVX2 = ThreefryEngineAVX2<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine using AVX2
/// \ingroup Threefry
using ThreefryAVX2 = ThreefryEngineAVX2<std::uint32_t>;

/// \brief The default 64-bits Threefry engine using AVX2
/// \ingroup Threefry
using ThreefryAVX2_64 = ThreefryEngineAVX2<std::uint64_t>;

#endif // VSMC_HAS_AVX2

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
