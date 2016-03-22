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

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

#define VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(T, I, val)                       \
    template <>                                                               \
    class PhiloxWeylConstant<T, I> : public std::integral_constant<T, val>    \
    {                                                                         \
    }; // class PhiloxWeylConstant

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val)                   \
    template <>                                                               \
    class PhiloxRoundConstant<T, K, I>                                        \
        : public std::integral_constant<T, val>                               \
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

template <typename, std::size_t>
class PhiloxWeylConstant;

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 0, UINT32_C(0x9E3779B9))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 1, UINT32_C(0xBB67AE85))

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 0, UINT64_C(0x9E3779B97F4A7C15))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 1, UINT64_C(0xBB67AE8584CAA73B))

template <typename, std::size_t, std::size_t>
class PhiloxRoundConstant;

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint32_t, 2, 0, UINT32_C(0xD256D193))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint32_t, 4, 0, UINT32_C(0xD2511F53))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint32_t, 4, 1, UINT32_C(0xCD9E8D57))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint64_t, 2, 0, UINT64_C(0xD2B74407B1CE6E93))

VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint64_t, 4, 0, UINT64_C(0xD2E7470EE14C6C93))
VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(
    std::uint64_t, 4, 1, UINT64_C(0xCA5A826395121157))

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint32_t b, std::uint32_t &hi, std::uint32_t &lo)
{
    std::uint64_t prod = static_cast<std::uint64_t>(b) *
        static_cast<std::uint64_t>(
                             PhiloxRoundConstant<std::uint32_t, K, I>::value);
    hi = static_cast<std::uint32_t>(prod >> 32);
    lo = static_cast<std::uint32_t>(prod);
}

#if VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint64_t b, std::uint64_t &hi, std::uint64_t &lo)
{
    unsigned VSMC_INT128 prod =
        static_cast<unsigned VSMC_INT128>(b) *
        static_cast<unsigned VSMC_INT128>(
            PhiloxRoundConstant<std::uint64_t, K, I>::value);
    hi = static_cast<std::uint64_t>(prod >> 64);
    lo = static_cast<std::uint64_t>(prod);
}

#elif defined(VSMC_MSVC) // VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint64_t b, std::uint64_t &hi, std::uint64_t &lo)
{
    lo = _umul128(PhiloxRoundConstant<std::uint64_t, K, I>::value, b, &hi);
}

#else // VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint64_t b, std::uint64_t &hi, std::uint64_t &lo)
{
    const std::uint64_t a = PhiloxRoundConstant<std::uint64_t, K, I>::value;
    const unsigned whalf = 32;
    const std::uint64_t lomask = (static_cast<std::uint64_t>(1) << whalf) - 1;

    lo = static_cast<std::uint64_t>(a * b);

    const std::uint64_t ahi = a >> whalf;
    const std::uint64_t alo = a & lomask;
    const std::uint64_t bhi = b >> whalf;
    const std::uint64_t blo = b & lomask;

    const std::uint64_t ahbl = ahi * blo;
    const std::uint64_t albh = alo * bhi;

    const std::uint64_t ahbl_albh = ((ahbl & lomask) + (albh & lomask));

    hi = ahi * bhi + (ahbl >> whalf) + (albh >> whalf);
    hi += ahbl_albh >> whalf;
    hi += ((lo >> whalf) < (ahbl_albh & lomask));
}

#endif // VSMC_HAS_INT128

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

template <typename T, std::size_t K, std::size_t N, bool = (N > 0)>
class PhiloxRound
{
    public:
    static void eval(std::array<T, K> &, const std::array<T, K / 2> &) {}
}; // class PhiloxRound

template <typename T, std::size_t N>
class PhiloxRound<T, 2, N, true>
{
    public:
    static void eval(std::array<T, 2> &state, const std::array<T, 1> &par)
    {
        T hi = 0;
        T lo = 0;
        philox_hilo<2, 0>(std::get<0>(state), hi, lo);
        hi ^= std::get<0>(par);
        std::get<0>(state) = hi ^ std::get<1>(state);
        std::get<1>(state) = lo;
    }
}; // class PhiloxRound

template <typename T, std::size_t N>
class PhiloxRound<T, 4, N, true>
{
    public:
    static void eval(std::array<T, 4> &state, const std::array<T, 2> &par)
    {
        T hi0 = 0;
        T lo1 = 0;
        T hi2 = 0;
        T lo3 = 0;
        philox_hilo<4, 1>(std::get<2>(state), hi0, lo1);
        philox_hilo<4, 0>(std::get<0>(state), hi2, lo3);

        hi0 ^= std::get<0>(par);
        hi2 ^= std::get<1>(par);
        std::get<0>(state) = hi0 ^ std::get<1>(state);
        std::get<1>(state) = lo1;
        std::get<2>(state) = hi2 ^ std::get<3>(state);
        std::get<3>(state) = lo3;
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

    static_assert(sizeof(ResultType) == sizeof(std::uint32_t) ||
            sizeof(ResultType) == sizeof(std::uint64_t),
        "**PhiloxGenerator** USED WITH ResultType OF SIZE OTHER THAN 32 OR 64 "
        "BITS");

    static_assert(
        K == 2 || K == 4, "**PhiloxGenerator** USED WITH K OTHER THAN 2 OR 4");

    public:
    using result_type = ResultType;
    using ctr_type = std::array<ResultType, K>;
    using key_type = std::array<ResultType, K / 2>;

    static constexpr std::size_t size() { return K; }

    void reset(const key_type &) {}

    void operator()(ctr_type &ctr, const key_type &key, ctr_type &buffer) const
    {
        increment(ctr);
        buffer = ctr;
        key_type par = key;
        generate<0>(buffer, par, std::true_type());
    }

    void operator()(ctr_type &ctr, const key_type &key, std::size_t n,
        ctr_type *buffer) const
    {
        increment(ctr, n, buffer);
        for (std::size_t i = 0; i != n; ++i) {
            key_type par = key;
            generate<0>(buffer[i], par, std::true_type());
        }
    }

    private:
    template <std::size_t>
    void generate(ctr_type &, key_type &, std::false_type) const
    {
    }

    template <std::size_t N>
    void generate(ctr_type &state, key_type &par, std::true_type) const
    {
        internal::PhiloxBumpKey<ResultType, K, N>::eval(par);
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
