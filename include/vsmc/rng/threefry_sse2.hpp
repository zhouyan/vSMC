//============================================================================
// vSMC/include/vsmc/rng/threefry_sse2.hpp
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

#ifndef VSMC_RNG_THREEFRY_SSE2_HPP
#define VSMC_RNG_THREEFRY_SSE2_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/internal/m128i.hpp>

#define VSMC_STATIC_ASSERT_RNG_THREFRY_SSE2_RESULT_TYPE(ResultType)           \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, std::uint32_t>::value ||     \
                           std::is_same<ResultType, std::uint64_t>::value),   \
        "**ThreefrySSE2Engine** USED WITH ResultType OTHER THAN "             \
        "std::uint32_t OR std::uint64_t")

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_SSE2_SIZE(K)                          \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                    \
        "**ThreefrySSE2Engine** USED WITH SIZE OTHER THAN 2 OR 4")

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_SSE2                                  \
    VSMC_STATIC_ASSERT_RNG_THREEFRY_SSE2_RESULT_TYPE(ResultType);             \
    VSMC_STATIC_ASSERT_RNG_THREEFRY_SSE2_SIZE(K);

#define VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(T, K, N, I, val)        \
    template <>                                                               \
    struct ThreefrySSE2RotateConstantValue<T, K, N, I>                        \
        : public std::integral_constant<int, val> {                           \
    };

/// \brief ThreefrySSE2SSE2Engine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_SSE2_ROUNDS
#define VSMC_RNG_THREEFRY_SSE2_ROUNDS 20
#endif

namespace vsmc
{

namespace internal
{

template <typename>
struct ThreefrySSE2KSConstantValue;

template <>
struct ThreefrySSE2KSConstantValue<std::uint32_t>
    : public std::integral_constant<std::uint32_t, UINT32_C(0x1BD11BDA)> {
};

template <>
struct ThreefrySSE2KSConstantValue<std::uint64_t>
    : public std::integral_constant<std::uint64_t,
          UINT64_C(0x1BD11BDAA9FC1A22)> {
};

template <typename, std::size_t, std::size_t, std::size_t>
struct ThreefrySSE2RotateConstantValue;

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 0, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 1, 0, 15)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 2, 0, 26)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 3, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 4, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 5, 0, 29)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 6, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 2, 7, 0, 24)

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 0, 0, 10)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 1, 0, 11)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 2, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 3, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 4, 0, 6)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 5, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 6, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 7, 0, 18)

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 0, 1, 26)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 1, 1, 21)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 2, 1, 27)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 3, 1, 5)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 4, 1, 20)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 5, 1, 11)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 6, 1, 10)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint32_t, 4, 7, 1, 20)

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 0, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 1, 0, 42)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 2, 0, 12)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 3, 0, 31)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 4, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 5, 0, 32)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 6, 0, 24)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 2, 7, 0, 21)

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 0, 0, 14)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 1, 0, 52)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 2, 0, 23)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 3, 0, 5)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 4, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 5, 0, 46)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 6, 0, 58)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 7, 0, 32)

VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 0, 1, 16)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 1, 1, 57)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 2, 1, 40)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 3, 1, 37)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 4, 1, 33)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 5, 1, 12)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 6, 1, 22)
VSMC_DEFINE_RNG_THREEFRY_SSE2_ROTATE_CONSTANT(std::uint64_t, 4, 7, 1, 32)

template <typename ResultType, int R>
struct ThreefrySSE2RotateImpl;

template <int R>
struct ThreefrySSE2RotateImpl<std::uint32_t, R> {
    static __m128i eval(__m128i x)
    {
        __m128i a = _mm_slli_epi32(x, R);
        __m128i b = _mm_srli_epi32(x, 32 - R);

        return _mm_or_si128(a, b);
    }
};

template <int R>
struct ThreefrySSE2RotateImpl<std::uint64_t, R> {
    static __m128i eval(__m128i x)
    {
        __m128i a = _mm_slli_epi64(x, R);
        __m128i b = _mm_srli_epi64(x, 32 - R);

        return _mm_or_si128(a, b);
    }
};

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct ThreefrySSE2Rotate {
    static void eval(std::array<__m128i, K> &) {}
};

template <std::size_t N>
struct ThreefrySSE2Rotate<std::uint32_t, 2, N, true> {
    static void eval(std::array<__m128i, 2> &state)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefrySSE2RotateImpl<std::uint32_t,
                ThreefrySSE2RotateConstantValue<std::uint32_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefrySSE2Rotate

template <std::size_t N>
struct ThreefrySSE2Rotate<std::uint32_t, 4, N, true> {
    static void eval(std::array<__m128i, 4> &state)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefrySSE2RotateImpl<std::uint32_t,
                ThreefrySSE2RotateConstantValue<std::uint32_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm_add_epi32(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefrySSE2RotateImpl<std::uint32_t,
                ThreefrySSE2RotateConstantValue<std::uint32_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm_xor_si128(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefrySSE2Rotate

template <std::size_t N>
struct ThreefrySSE2Rotate<std::uint64_t, 2, N, true> {
    static void eval(std::array<__m128i, 2> &state)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefrySSE2RotateImpl<std::uint64_t,
                ThreefrySSE2RotateConstantValue<std::uint64_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefrySSE2Rotate

template <std::size_t N>
struct ThreefrySSE2Rotate<std::uint64_t, 4, N, true> {
    static void eval(std::array<__m128i, 4> &state)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefrySSE2RotateImpl<std::uint64_t,
                ThreefrySSE2RotateConstantValue<std::uint64_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm_add_epi64(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefrySSE2RotateImpl<std::uint64_t,
                ThreefrySSE2RotateConstantValue<std::uint64_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm_xor_si128(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefrySSE2Rotate

template <typename ResultType, std::size_t K, std::size_t N,
    bool = (N % 4 == 0)>
struct ThreefrySSE2InsertKey {
    static void eval(
        std::array<__m128i, K> &, const std::array<__m128i, K + 1> &)
    {
    }
}; // struct ThreefrySSE2InsertKey

template <std::size_t N>
struct ThreefrySSE2InsertKey<std::uint32_t, 2, N, true> {
    static void eval(
        std::array<__m128i, 2> &state, const std::array<__m128i, 3> &par)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm_add_epi32(std::get<1>(state), std::get<i1_>(par));
        std::get<1>(state) = _mm_add_epi32(
            std::get<1>(state), _mm_set1_epi32(static_cast<int>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefrySSE2InsertKey

template <std::size_t N>
struct ThreefrySSE2InsertKey<std::uint32_t, 4, N, true> {
    static void eval(
        std::array<__m128i, 4> &state, const std::array<__m128i, 5> &par)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm_add_epi32(std::get<1>(state), std::get<i1_>(par));
        std::get<2>(state) =
            _mm_add_epi32(std::get<2>(state), std::get<i2_>(par));
        std::get<3>(state) =
            _mm_add_epi32(std::get<3>(state), std::get<i3_>(par));
        std::get<3>(state) = _mm_add_epi32(
            std::get<3>(state), _mm_set1_epi32(static_cast<int>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefrySSE2InsertKey

template <std::size_t N>
struct ThreefrySSE2InsertKey<std::uint64_t, 2, N, true> {
    static void eval(
        std::array<__m128i, 2> &state, const std::array<__m128i, 3> &par)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm_add_epi64(std::get<1>(state), std::get<i1_>(par));
        std::get<1>(state) = _mm_add_epi64(std::get<1>(state),
            _mm_set1_epi64x(static_cast<VSMC_INT64>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefrySSE2InsertKey

template <std::size_t N>
struct ThreefrySSE2InsertKey<std::uint64_t, 4, N, true> {
    static void eval(
        std::array<__m128i, 4> &state, const std::array<__m128i, 5> &par)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm_add_epi64(std::get<1>(state), std::get<i1_>(par));
        std::get<2>(state) =
            _mm_add_epi64(std::get<2>(state), std::get<i2_>(par));
        std::get<3>(state) =
            _mm_add_epi64(std::get<3>(state), std::get<i3_>(par));
        std::get<3>(state) = _mm_add_epi64(std::get<3>(state),
            _mm_set1_epi64x(static_cast<VSMC_INT64>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefrySSE2InsertKey

template <typename, std::size_t>
struct ThreefrySSE2IncrementAndPack;

template <>
struct ThreefrySSE2IncrementAndPack<std::uint32_t, 2> {
    typedef std::array<__m128i, 2> buffer_type;
    typedef std::array<std::uint32_t, 2> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, buffer_type &buffer)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;

        std::get<0>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<0>(ctr0)),
                static_cast<int>(std::get<0>(ctr1)),
                static_cast<int>(std::get<0>(ctr2)),
                static_cast<int>(std::get<0>(ctr3)));
        std::get<1>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<1>(ctr0)),
                static_cast<int>(std::get<1>(ctr1)),
                static_cast<int>(std::get<1>(ctr2)),
                static_cast<int>(std::get<1>(ctr3)));
    }
}; // struct ThreefrySSE2IncrementAndPack

template <>
struct ThreefrySSE2IncrementAndPack<std::uint32_t, 4> {
    typedef std::array<__m128i, 4> buffer_type;
    typedef std::array<std::uint32_t, 4> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, buffer_type &buffer)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;

        std::get<0>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<0>(ctr0)),
                static_cast<int>(std::get<0>(ctr1)),
                static_cast<int>(std::get<0>(ctr2)),
                static_cast<int>(std::get<0>(ctr3)));
        std::get<1>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<1>(ctr0)),
                static_cast<int>(std::get<1>(ctr1)),
                static_cast<int>(std::get<1>(ctr2)),
                static_cast<int>(std::get<1>(ctr3)));
        std::get<2>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<2>(ctr0)),
                static_cast<int>(std::get<2>(ctr1)),
                static_cast<int>(std::get<2>(ctr2)),
                static_cast<int>(std::get<2>(ctr3)));
        std::get<3>(buffer) =
            _mm_set_epi32(static_cast<int>(std::get<3>(ctr0)),
                static_cast<int>(std::get<3>(ctr1)),
                static_cast<int>(std::get<3>(ctr2)),
                static_cast<int>(std::get<3>(ctr3)));
    }
}; // struct ThreefrySSE2IncrementAndPack

template <>
struct ThreefrySSE2IncrementAndPack<std::uint64_t, 2> {
    typedef std::array<__m128i, 2> buffer_type;
    typedef std::array<std::uint64_t, 2> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, buffer_type &buffer)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;

        std::get<0>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<0>(ctr0)),
                static_cast<VSMC_INT64>(std::get<0>(ctr1)));
        std::get<1>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<1>(ctr0)),
                static_cast<VSMC_INT64>(std::get<1>(ctr1)));
    }
}; // struct ThreefrySSE2IncrementAndPack

template <>
struct ThreefrySSE2IncrementAndPack<std::uint64_t, 4> {
    typedef std::array<__m128i, 4> buffer_type;
    typedef std::array<std::uint64_t, 4> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, buffer_type &buffer)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;

        std::get<0>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<0>(ctr0)),
                static_cast<VSMC_INT64>(std::get<0>(ctr1)));
        std::get<1>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<1>(ctr0)),
                static_cast<VSMC_INT64>(std::get<1>(ctr1)));
        std::get<2>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<2>(ctr0)),
                static_cast<VSMC_INT64>(std::get<2>(ctr1)));
        std::get<3>(buffer) =
            _mm_set_epi64x(static_cast<VSMC_INT64>(std::get<3>(ctr0)),
                static_cast<VSMC_INT64>(std::get<3>(ctr1)));
    }
}; // struct ThreefrySSE2IncrementAndPack

template <typename, std::size_t>
struct ThreefrySSE2ParPack;

template <>
struct ThreefrySSE2ParPack<std::uint32_t, 2> {
    static void eval(
        const std::array<std::uint32_t, 3> &par, std::array<__m128i, 3> &pack)
    {
        std::get<0>(pack) = _mm_set1_epi32(static_cast<int>(std::get<0>(par)));
        std::get<1>(pack) = _mm_set1_epi32(static_cast<int>(std::get<1>(par)));
        std::get<2>(pack) = _mm_set1_epi32(static_cast<int>(std::get<2>(par)));
    }
}; // struct ThreefrySSE2ParPack

template <>
struct ThreefrySSE2ParPack<std::uint32_t, 4> {
    static void eval(
        const std::array<std::uint32_t, 5> &par, std::array<__m128i, 5> &pack)
    {
        std::get<0>(pack) = _mm_set1_epi32(static_cast<int>(std::get<0>(par)));
        std::get<1>(pack) = _mm_set1_epi32(static_cast<int>(std::get<1>(par)));
        std::get<2>(pack) = _mm_set1_epi32(static_cast<int>(std::get<2>(par)));
        std::get<3>(pack) = _mm_set1_epi32(static_cast<int>(std::get<3>(par)));
        std::get<4>(pack) = _mm_set1_epi32(static_cast<int>(std::get<4>(par)));
    }
}; // struct ThreefrySSE2ParPack

template <>
struct ThreefrySSE2ParPack<std::uint64_t, 2> {
    static void eval(
        const std::array<std::uint64_t, 3> &par, std::array<__m128i, 3> &pack)
    {
        std::get<0>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<2>(par)));
    }
}; // struct ThreefrySSE2ParPack

template <>
struct ThreefrySSE2ParPack<std::uint64_t, 4> {
    static void eval(
        const std::array<std::uint64_t, 5> &par, std::array<__m128i, 5> &pack)
    {
        std::get<0>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<2>(par)));
        std::get<3>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<3>(par)));
        std::get<4>(pack) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<4>(par)));
    }
}; // struct ThreefrySSE2ParPack

} // namespace vsmc::internal

/// \brief ThreefrySSE2 RNG engine reimplemented
/// \ingroup ThreefrySSE2
///
/// \details
/// This is a reimplementation of the algorithm ThreefrySSE2 as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented
/// in [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// Depending on the compilers, processors and RNG configurations, it might be
/// slightly faster or slower than the original implementation. At most
/// two-folds performace difference (both faster and slower) were observed.
///
/// This implementation is slightly more flexible in the sense that it does
/// not limit the number of rounds. However, larger number of rounds can have
/// undesired effects. To say the least, currently all loops are unrolled,
/// which can slow down significantly when the number of rounds is large.
///
/// Compared to `r123:Engine<r123::ThreefrySSE24x32>` etc., when using the
/// default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^n\f$ iterations, where \f$n\f$ is the number of
/// bits (32 or 64).  Further iterations may produce different results, as
/// vSMC increment the counter slightly differently, but it still cover the
/// same range and has the same period as the original.
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefrySSE2Engine
{
    public:
    typedef ResultType result_type;
    typedef std::array<__m128i, K> buffer_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K> key_type;

    private:
    typedef Counter<ctr_type> counter;
    static constexpr std::size_t M = sizeof(__m128i) / sizeof(result_type);

    public:
    explicit ThreefrySSE2Engine(result_type s = 0) : index_(K * M)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefrySSE2Engine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefrySSE2Engine<ResultType, K, Rounds>>::value>::type
            * = nullptr)
        : index_(K * M)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(seq);
    }

    ThreefrySSE2Engine(const key_type &k) : index_(K * M)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY;
        seed(k);
    }

    void seed(result_type s)
    {
        counter::reset(ctr_);
        key_type k;
        k.fill(0);
        k.front() = s;
        init_par(k);
        index_ = K * M;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefrySSE2Engine<ResultType, K, Rounds>>::value>::type
            * = nullptr)
    {
        counter::reset(ctr_);
        key_type k;
        seq.generate(k.begin(), k.end());
        init_par(k);
        index_ = K * M;
    }

    void seed(const key_type &k)
    {
        counter::reset(ctr_);
        init_par(k);
        index_ = K * M;
    }

    ctr_type ctr() const { return ctr_; }

    key_type key() const
    {
        key_type k;
        for (std::size_t i = 0; i != K; ++i)
            k[i] = par_[i];

        return k;
    }

    void ctr(const ctr_type &c)
    {
        counter::set(ctr_, c);
        index_ = K * M;
    }

    void key(const key_type &k)
    {
        init_par(k);
        index_ = K * M;
    }

    result_type operator()()
    {
        if (index_ == K * M) {
            internal::ThreefrySSE2IncrementAndPack<ResultType, K>::eval(
                ctr_, buffer_);
            generate_buffer();
            index_ = 0;
        }

        return reinterpret_cast<const result_type *>(buffer_.data())[index_++];
    }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= K * M) {
            index_ += n;
            return;
        }

        n -= K * M - index_;
        if (n <= K * M) {
            index_ = K * M;
            operator()();
            index_ = n;
            return;
        }

        counter::increment(ctr_, static_cast<result_type>(n / (K * M)));
        index_ = K * M;
        operator()();
        index_ = n % (K * M);
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(
        const ThreefrySSE2Engine<ResultType, K, Rounds> &eng1,
        const ThreefrySSE2Engine<ResultType, K, Rounds> &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.ctr_ == eng2.ctr_ &&
            eng1.par_ == eng2.par_;
    }

    friend bool operator!=(
        const ThreefrySSE2Engine<ResultType, K, Rounds> &eng1,
        const ThreefrySSE2Engine<ResultType, K, Rounds> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const ThreefrySSE2Engine<ResultType, K, Rounds> &eng)
    {
        if (!os.good())
            return os;

        os << eng.ctr_ << ' ';
        os << eng.par_ << ' ';
        os << eng.buffer_ << ' ';
        for (std::size_t k = 0; k != K; ++k) {
            internal::m128i_output(os, eng.buffer_[k]);
            os << ' ';
        }
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        ThreefrySSE2Engine<ResultType, K, Rounds> &eng)
    {
        if (!is.good())
            return is;

        ThreefrySSE2Engine<ResultType, K, Rounds> eng_tmp;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.par_;
        for (std::size_t k = 0; k != K; ++k)
            internal::m128i_input(is, eng_tmp.buffer_[k]);
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    ctr_type ctr_;
    std::array<ResultType, K + 1> par_;
    buffer_type buffer_;
    std::size_t index_;

    typedef std::array<__m128i, K + 1> par_type;

    void generate_buffer()
    {
        par_type par;
        internal::ThreefrySSE2ParPack<ResultType, K>::eval(par_, par);
        generate_buffer<0>(par, std::true_type());
    }

    template <std::size_t>
    void generate_buffer(const par_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(const par_type &par, std::true_type)
    {
        internal::ThreefrySSE2Rotate<ResultType, K, N>::eval(buffer_);
        internal::ThreefrySSE2InsertKey<ResultType, K, N>::eval(buffer_, par);
        generate_buffer<N + 1>(
            par, std::integral_constant < bool, N<Rounds>());
    }

    void init_par(const key_type &key)
    {
        par_.back() = internal::ThreefrySSE2KSConstantValue<ResultType>::value;
        par_xor<0>(key, std::integral_constant<bool, 0 < K>());
    }

    template <std::size_t>
    void par_xor(const key_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void par_xor(const key_type &key, std::true_type)
    {
        std::get<N>(par_) = std::get<N>(key);
        par_.back() ^= std::get<N>(key);
        par_xor<N + 1>(key, std::integral_constant<bool, N + 1 < K>());
    }
}; // class ThreefrySSE2Engine

/// \brief ThreefrySSE22x32 RNG engine reimplemented
/// \ingroup ThreefrySSE2
typedef ThreefrySSE2Engine<std::uint32_t, 2> Threefry2x32SSE2;

/// \brief ThreefrySSE24x32 RNG engine reimplemented
/// \ingroup ThreefrySSE2
typedef ThreefrySSE2Engine<std::uint32_t, 4> Threefry4x32SSE2;

/// \brief ThreefrySSE22x64 RNG engine reimplemented
/// \ingroup ThreefrySSE2
typedef ThreefrySSE2Engine<std::uint64_t, 2> Threefry2x64SSE2;

/// \brief ThreefrySSE24x64 RNG engine reimplemented
/// \ingroup ThreefrySSE2
typedef ThreefrySSE2Engine<std::uint64_t, 4> Threefry4x64SSE2;

/// \brief The default 32-bits ThreefrySSE2 engine
/// \ingroup ThreefrySSE2
typedef Threefry4x32SSE2 ThreefrySSE2;

/// \brief The default 64-bits ThreefrySSE2 engine
/// \ingroup ThreefrySSE2
typedef Threefry4x64SSE2 ThreefrySSE2_64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_SSE2_HPP
