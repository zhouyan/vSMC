//============================================================================
// vSMC/include/vsmc/rng/philox.hpp
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

#ifndef VSMC_RNG_PHILOX_HPP
#define VSMC_RNG_PHILOX_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/counter.hpp>
#include <vsmc/rngc/philox.h>

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

#define VSMC_DEFINE_RNG_PHILOX_WEYL_CONSTANT(W, K, val)                       \
    template <typename T>                                                     \
    class PhiloxWeylConstant<T, K, W>                                         \
        : public std::integral_constant<T, UINT##W##_C(val)>                  \
    {                                                                         \
    }; // PhiloxRoundConstant

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(W, K, I, val)                   \
    template <typename T>                                                     \
    class PhiloxRoundConstant<T, K, I, W>                                     \
        : public std::integral_constant<T, UINT##W##_C(val)>                  \
    {                                                                         \
    }; // PhiloxRoundConstant

/// \brief PhiloxGenerator default rounds
/// \ingroup Config
#ifndef VSMC_RNG_PHILOX_ROUNDS
#define VSMC_RNG_PHILOX_ROUNDS 10
#endif

/// \brief PhiloxGenerator default vector length
/// \ingroup Config
#ifndef VSMC_RNG_PHILOX_VECTOR_LENGTH
#define VSMC_RNG_PHILOX_VECTOR_LENGTH 4
#endif

namespace vsmc
{

namespace internal
{

template <typename T, std::size_t, int = std::numeric_limits<T>::digits>
class PhiloxWeylConstant;

VSMC_DEFINE_RNG_PHILOX_WEYL_CONSTANT(32, 0, 0x9E3779B9)
VSMC_DEFINE_RNG_PHILOX_WEYL_CONSTANT(32, 1, 0xBB67AE85)
VSMC_DEFINE_RNG_PHILOX_WEYL_CONSTANT(64, 0, 0x9E3779B97F4A7C15)
VSMC_DEFINE_RNG_PHILOX_WEYL_CONSTANT(64, 1, 0xBB67AE8584CAA73B)

template <typename T, std::size_t, std::size_t,
    int = std::numeric_limits<T>::digits>
class PhiloxRoundConstant;

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(32, 2, 0, 0xD256D193)
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(32, 4, 0, 0xD2511F53)
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(32, 4, 1, 0xCD9E8D57)
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(64, 2, 0, 0xD2B74407B1CE6E93)
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(64, 4, 0, 0xD2E7470EE14C6C93)
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(64, 4, 1, 0xCA5A826395121157)

template <typename T, std::size_t, std::size_t,
    int = std::numeric_limits<T>::digits>
class PhiloxHiLo;

template <typename T, std::size_t K, std::size_t I>
class PhiloxHiLo<T, K, I, 32>
{
    public:
    static std::array<T, 2> eval(T b)
    {
        union {
            std::uint64_t prod;
            std::array<T, 2> hilo;
        } buf;

        buf.prod = static_cast<std::uint64_t>(b) *
            static_cast<std::uint64_t>(PhiloxRoundConstant<T, K, I>::value);

        return buf.hilo;
    }
}; // class PhiloxHiLo

template <typename T, std::size_t K, std::size_t I>
class PhiloxHiLo<T, K, I, 64>
{
    public:
    static std::array<T, 2> eval(T b)
    {
#if VSMC_HAS_INT128
        union {
            unsigned VSMC_INT128 prod;
            std::array<T, 2> hilo;
        } buf;

        buf.prod = static_cast<unsigned VSMC_INT128>(b) *
            static_cast<unsigned VSMC_INT128>(
                       PhiloxRoundConstant<T, K, I>::value);

        return buf.hilo;
#elif defined(VSMC_MSVC)
        unsigned __int64 Multiplier =
            static_cast<unsigned __int64>(PhiloxRoundConstant<T, K, I>::value);
        unsigned __int64 Multiplicand = static_cast<unsigned __int64>(b);
        unsigned __int64 hi = 0;
        unsigned __int64 lo = 0;
        lo = _umul128(Multiplier, Multiplicand, &hi);
        std::array<T, 2> hilo = {{ static_cast<T>(lo), static_cast<T>(hi) }};

        return hilo;
#else  // VSMC_HAS_INT128
        const T a = PhiloxRoundConstant<T, K, I>::value;
        const T lomask = (static_cast<T>(1) << 32) - 1;
        const T ahi = a >> 32;
        const T alo = a & lomask;
        const T bhi = b >> 32;
        const T blo = b & lomask;
        const T ahbl = ahi * blo;
        const T albh = alo * bhi;
        const T ahbl_albh = ((ahbl & lomask) + (albh & lomask));

        T lo = a * b;
        T hi = ahi * bhi + (ahbl >> 32) + (albh >> 32);
        hi += ahbl_albh >> 32;
        hi += ((lo >> 32) < (ahbl_albh & lomask));
        std::array<T, 2> hilo = {{lo, hi}};

        return hilo;
#endif // VSMC_HAS_INT128
    }
}; // class PhiloxHiLo

template <typename T, std::size_t K, std::size_t N, bool = (N > 1)>
class PhiloxBumpKey
{
    public:
    static void eval(std::array<T, K / 2> &) {}
}; // class PhiloxBumpKey

template <typename T, std::size_t N>
class PhiloxBumpKey<T, 2, N, true>
{
    public:
    static void eval(std::array<T, 1> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<T, 0>::value;
    }
}; // class PhiloxBumpKey

template <typename T, std::size_t N>
class PhiloxBumpKey<T, 4, N, true>
{
    public:
    static void eval(std::array<T, 2> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<T, 0>::value;
        std::get<1>(par) += PhiloxWeylConstant<T, 1>::value;
    }
}; // class PhiloxBumpKey

template <typename T, std::size_t K>
class PhiloxInitPar
{
    public:
    template <std::size_t R>
    static void eval(const std::array<T, K / 2> &key,
        std::array<std::array<T, K / 2>, R> &par)
    {
        std::get<0>(par) = key;
        eval<1>(par, std::integral_constant<bool, 1 < R>());
    }

    private:
    template <std::size_t, std::size_t R>
    static void eval(std::array<std::array<T, K / 2>, R> &, std::false_type)
    {
    }

    template <std::size_t N, std::size_t R>
    static void eval(std::array<std::array<T, K / 2>, R> &par, std::true_type)
    {
        std::get<N>(par) = std::get<N - 1>(par);
        PhiloxBumpKey<T, K, N>::eval(std::get<N>(par));
        eval<N + 1>(par, std::integral_constant<bool, N + 1 < R>());
    }
}; // class PhiloxInitPar

template <typename T, std::size_t K, std::size_t N, bool = (N > 0)>
class PhiloxRound
{
    public:
    template <std::size_t R>
    static void eval(
        std::array<T, K> &, const std::array<std::array<T, K / 2>, R> &)
    {
    }
}; // class PhiloxRound

template <typename T, std::size_t N>
class PhiloxRound<T, 2, N, true>
{
    public:
    template <std::size_t R>
    static void eval(
        std::array<T, 2> &state, const std::array<std::array<T, 1>, R> &par)
    {
        std::array<T, 2> hilo = PhiloxHiLo<T, 2, 0>::eval(std::get<0>(state));
        std::get<1>(hilo) ^= std::get<0>(std::get<N>(par));
        std::get<0>(state) = std::get<1>(hilo) ^ std::get<1>(state);
        std::get<1>(state) = std::get<0>(hilo);
    }
}; // class PhiloxRound

template <typename T, std::size_t N>
class PhiloxRound<T, 4, N, true>
{
    public:
    template <std::size_t R>
    static void eval(
        std::array<T, 4> &state, const std::array<std::array<T, 2>, R> &par)
    {
        std::array<T, 2> hilo0 = PhiloxHiLo<T, 4, 1>::eval(std::get<2>(state));
        std::array<T, 2> hilo2 = PhiloxHiLo<T, 4, 0>::eval(std::get<0>(state));
        std::get<1>(hilo0) ^= std::get<0>(std::get<N>(par));
        std::get<1>(hilo2) ^= std::get<1>(std::get<N>(par));
        std::get<0>(state) = std::get<1>(hilo0) ^ std::get<1>(state);
        std::get<1>(state) = std::get<0>(hilo0);
        std::get<2>(state) = std::get<1>(hilo2) ^ std::get<3>(state);
        std::get<3>(state) = std::get<0>(hilo2);
    }
}; // class PhiloxRound

} // namespace vsmc::internal

/// \brief Philox RNG generator
/// \ingroup Philox
template <typename ResultType, std::size_t K = VSMC_RNG_PHILOX_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
class PhiloxGenerator
{
    static_assert(std::is_unsigned<ResultType>::value,
        "**PhiloxGenerator** USED WITH ResultType OTHER THAN UNSIGNED INTEGER "
        "TYPES");

    static_assert(std::numeric_limits<ResultType>::digits == 32 ||
            std::numeric_limits<ResultType>::digits == 64,
        "**PhiloxGenerator** USED WITH ResultType OF SIZE OTHER THAN 32 OR 64 "
        "BITS");

    static_assert(
        K == 2 || K == 4, "**PhiloxGenerator** USED WITH K OTHER THAN 2 OR 4");

    static_assert(
        Rounds != 0, "**PhiloxGenerator** USED WITH ROUNDS EQUAL TO ZERO");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, K>;
    using key_type = std::array<ResultType, K / 2>;

    static constexpr std::size_t size() { return K; }

    void reset(const key_type &) {}

    void operator()(ctr_type &ctr, const key_type &key, ctr_type &buffer) const
    {
        std::array<key_type, Rounds + 1> par;
        internal::PhiloxInitPar<ResultType, K>::eval(key, par);

        increment(ctr);
        buffer = ctr;
        generate<0>(buffer, par, std::true_type());
    }

    void operator()(ctr_type &ctr, const key_type &key, std::size_t n,
        ctr_type *buffer) const
    {
        std::array<key_type, Rounds + 1> par;
        internal::PhiloxInitPar<ResultType, K>::eval(key, par);

        for (std::size_t i = 0; i != n; ++i) {
            increment(ctr);
            buffer[i] = ctr;
            generate<0>(buffer[i], par, std::true_type());
        }
    }

    private:
    template <std::size_t>
    void generate(
        ctr_type &, std::array<key_type, Rounds + 1> &, std::false_type) const
    {
    }

    template <std::size_t N>
    void generate(ctr_type &state, std::array<key_type, Rounds + 1> &par,
        std::true_type) const
    {
        internal::PhiloxRound<ResultType, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant<bool, (N < Rounds)>());
    }
}; // class PhiloxGenerator

/// \brief Philox RNG engine
/// \ingroup Philox
template <typename ResultType, std::size_t K = VSMC_RNG_PHILOX_VECTOR_LENGTH,
    std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
using PhiloxEngine = CounterEngine<PhiloxGenerator<ResultType, K, Rounds>>;

/// \brief Philox2x32 RNG engine
/// \ingroup Philox
using Philox2x32 = PhiloxEngine<std::uint32_t, 2>;

/// \brief Philox4x32 RNG engine
/// \ingroup Philox
using Philox4x32 = PhiloxEngine<std::uint32_t, 4>;

/// \brief Philox2x64 RNG engine
/// \ingroup Philox
using Philox2x64 = PhiloxEngine<std::uint64_t, 2>;

/// \brief Philox4x64 RNG engine
/// \ingroup Philox
using Philox4x64 = PhiloxEngine<std::uint64_t, 4>;

/// \brief The default 32-bit Philox engine
/// \ingroup Philox
using Philox = PhiloxEngine<std::uint32_t>;

/// \brief The default 64-bit Philox engine
/// \ingroup Philox
using Philox_64 = PhiloxEngine<std::uint64_t>;

} // namespace vsmc

#endif // VSMC_RNG_PHILOX_HPP
