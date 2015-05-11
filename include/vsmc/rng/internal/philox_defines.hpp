//============================================================================
// vSMC/include/vsmc/rng/internal/philox_defines.hpp
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

#ifndef VSMC_RNG_INTERNAL_PHILOX_DEFINES_HPP
#define VSMC_RNG_INTERNAL_PHILOX_DEFINES_HPP

#include <vsmc/rng/internal/common.hpp>

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

#define VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType)                 \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, std::uint32_t>::value ||     \
                           std::is_same<ResultType, std::uint64_t>::value),   \
        "**PhiloxEngine** USED WITH ResultType OTHER THAN std::uint32_t OR "  \
        "std::uint64_t")

#define VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K)                                 \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                    \
        "**PhiloxEngine** USED WITH SIZE OTHER THAN 2 OR 4")

#define VSMC_STATIC_ASSERT_RNG_PHILOX                                         \
    VSMC_STATIC_ASSERT_RNG_PHILOX_RESULT_TYPE(ResultType);                    \
    VSMC_STATIC_ASSERT_RNG_PHILOX_SIZE(K);

#define VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(T, I, val)                       \
    template <>                                                               \
    struct PhiloxWeylConstant<T, I> : public std::integral_constant<T, val> { \
    };

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val)                   \
    template <>                                                               \
    struct PhiloxRoundConstant<T, K, I>                                       \
        : public std::integral_constant<T, val> {                             \
    };

/// \brief PhiloxEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_PHILOX_ROUNDS
#define VSMC_RNG_PHILOX_ROUNDS 10
#endif

namespace vsmc
{

namespace internal
{

template <typename, std::size_t>
struct PhiloxWeylConstant;

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 0, UINT32_C(0x9E3779B9))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 1, UINT32_C(0xBB67AE85))

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 0, UINT64_C(0x9E3779B97F4A7C15))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 1, UINT64_C(0xBB67AE8584CAA73B))

template <typename, std::size_t, std::size_t>
struct PhiloxRoundConstant;

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

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 1)>
struct PhiloxBumpKey {
    static void eval(std::array<ResultType, K / 2> &) {}
};

template <typename ResultType, std::size_t N>
struct PhiloxBumpKey<ResultType, 2, N, true> {
    static void eval(std::array<ResultType, 1> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<ResultType, 0>::value;
    }
}; // struct PhiloxBumpKey

template <typename ResultType, std::size_t N>
struct PhiloxBumpKey<ResultType, 4, N, true> {
    static void eval(std::array<ResultType, 2> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<ResultType, 0>::value;
        std::get<1>(par) += PhiloxWeylConstant<ResultType, 1>::value;
    }
}; // struct PhiloxBumpKey

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

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct PhiloxRound {
    static void eval(
        std::array<ResultType, K> &, const std::array<ResultType, K / 2> &)
    {
    }
}; // struct PhiloxRound

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 2, N, true> {
    static void eval(
        std::array<ResultType, 2> &state, const std::array<ResultType, 1> &par)
    {
        ResultType hi = 0;
        ResultType lo = 0;
        philox_hilo<2, 0>(std::get<0>(state), hi, lo);
        hi ^= std::get<0>(par);
        std::get<0>(state) = hi ^ std::get<1>(state);
        std::get<1>(state) = lo;
    }
}; // struct PhiloxRound

template <typename ResultType, std::size_t N>
struct PhiloxRound<ResultType, 4, N, true> {
    static void eval(
        std::array<ResultType, 4> &state, const std::array<ResultType, 2> &par)
    {
        ResultType hi0 = 0;
        ResultType lo1 = 0;
        ResultType hi2 = 0;
        ResultType lo3 = 0;
        philox_hilo<4, 1>(std::get<2>(state), hi0, lo1);
        philox_hilo<4, 0>(std::get<0>(state), hi2, lo3);

        hi0 ^= std::get<0>(par);
        hi2 ^= std::get<1>(par);
        std::get<0>(state) = hi0 ^ std::get<1>(state);
        std::get<1>(state) = lo1;
        std::get<2>(state) = hi2 ^ std::get<3>(state);
        std::get<3>(state) = lo3;
    }
}; // struct PhiloxRound

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_PHILOX_DEFINES_HPP
