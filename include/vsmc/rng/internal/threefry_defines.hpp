//============================================================================
// vSMC/include/vsmc/rng/internal/threefry_defines.hpp
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

#ifndef VSMC_RNG_INTERNAL_THREEFRY_DEFINES_HPP
#define VSMC_RNG_INTERNAL_THREEFRY_DEFINES_HPP

#include <vsmc/rng/internal/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_RESULT_TYPE(ResultType, SIMD)         \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, std::uint32_t>::value ||     \
                           std::is_same<ResultType, std::uint64_t>::value),   \
        "**Threefry" #SIMD                                                    \
        "Engine** USED WITH ResultType OTHER THAN std::uint32_tOR "           \
        "std::uint64_t")

#define VSMC_STATIC_ASSERT_RNG_THREEFRY_SIZE(K, SIMD)                         \
    VSMC_STATIC_ASSERT((K == 2 || K == 4),                                    \
        "**Threefry" #SIMD "** USED WITH SIZE OTHER THAN 2 OR 4")

#define VSMC_STATIC_ASSERT_RNG_THREEFRY(SIMD)                                 \
    VSMC_STATIC_ASSERT_RNG_THREEFRY_RESULT_TYPE(ResultType, SIMD);            \
    VSMC_STATIC_ASSERT_RNG_THREEFRY_SIZE(K, SIMD);

#define VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(T, K, N, I, val)             \
    template <>                                                               \
    struct ThreefryRotateConstant<T, K, N, I>                                 \
        : public std::integral_constant<int, val> {                           \
    };

/// \brief ThreefryEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_THREEFRY_ROUNDS
#define VSMC_RNG_THREEFRY_ROUNDS 20
#endif

namespace vsmc
{

namespace internal
{

template <typename>
struct ThreefryKSConstant;

template <typename T, template <typename> class Wrapper>
struct ThreefryKSConstant<Wrapper<T>> : public ThreefryKSConstant<T> {
};

template <>
struct ThreefryKSConstant<std::uint32_t>
    : public std::integral_constant<std::uint32_t, UINT32_C(0x1BD11BDA)> {
};

template <>
struct ThreefryKSConstant<std::uint64_t>
    : public std::integral_constant<std::uint64_t,
          UINT64_C(0x1BD11BDAA9FC1A22)> {
};

template <typename, std::size_t, std::size_t, std::size_t>
struct ThreefryRotateConstant;

template <typename T, template <typename> class Wrapper, std::size_t K,
    std::size_t N, std::size_t I>
struct ThreefryRotateConstant<Wrapper<T>, K, N, I>
    : public ThreefryRotateConstant<T, K, N, I> {
};

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

template <typename ResultType, std::size_t K>
struct ThreefryInitPar {
    static void eval(const std::array<ResultType, K> &key,
        std::array<ResultType, K + 1> &par)
    {
        par.back() = ThreefryKSConstant<ResultType>::value;
        par_xor<0>(key, par, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t>
    static void par_xor(const std::array<ResultType, K> &,
        std::array<ResultType, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void par_xor(const std::array<ResultType, K> &key,
        std::array<ResultType, K + 1> &par, std::true_type)
    {
        std::get<N>(par) = std::get<N>(key);
        par.back() ^= std::get<N>(key);
        par_xor<N + 1>(key, par, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryInitPar

template <typename ResultType>
struct ThreefryRotateBits
    : public std::integral_constant<int, sizeof(ResultType) * 8> {
};

template <typename T, template <typename> class Wrapper>
struct ThreefryRotateBits<Wrapper<T>> : public ThreefryRotateBits<T> {
};

template <typename ResultType, int R>
struct ThreefryRotateImpl {
    static ResultType eval(const ResultType &x)
    {
        return (x << R) | (x >> (ThreefryRotateBits<ResultType>::value - R));
    }
}; // struct ThreefryRotateImpl

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct ThreefryRotate {
    static void eval(std::array<ResultType, K> &) {}
};

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 2, N, true> {
    static void eval(std::array<ResultType, 2> &state)
    {
        std::get<0>(state) += std::get<1>(state);
        std::get<1>(state) =
            ThreefryRotateImpl<ResultType,
                ThreefryRotateConstant<ResultType, 2, r_,
                                   0>::value>::eval(std::get<1>(state));
        std::get<1>(state) ^= std::get<0>(state);
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotate

template <typename ResultType, std::size_t N>
struct ThreefryRotate<ResultType, 4, N, true> {
    static void eval(std::array<ResultType, 4> &state)
    {
        std::get<0>(state) += std::get<i0_>(state);
        std::get<i0_>(state) =
            ThreefryRotateImpl<ResultType,
                ThreefryRotateConstant<ResultType, 4, r_,
                                   0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) ^= std::get<0>(state);

        std::get<2>(state) += std::get<i2_>(state);
        std::get<i2_>(state) =
            ThreefryRotateImpl<ResultType,
                ThreefryRotateConstant<ResultType, 4, r_,
                                   1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) ^= std::get<2>(state);
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotate

template <typename ResultType, std::size_t Inc>
struct ThreefryInsertKeyInc
    : public std::integral_constant<ResultType, static_cast<ResultType>(Inc)> {
};

template <typename T, template <typename> class Wrapper, std::size_t Inc>
struct ThreefryInsertKeyInc<Wrapper<T>, Inc>
    : public ThreefryInsertKeyInc<T, Inc> {
};

template <typename ResultType, std::size_t K, std::size_t N,
    bool = (N % 4 == 0)>
struct ThreefryInsertKey {
    static void eval(
        std::array<ResultType, K> &, const std::array<ResultType, K + 1> &)
    {
    }
}; // struct ThreefryInsertKey

template <typename ResultType, std::size_t N>
struct ThreefryInsertKey<ResultType, 2, N, true> {
    static void eval(
        std::array<ResultType, 2> &state, const std::array<ResultType, 3> &par)
    {
        std::get<0>(state) += std::get<i0_>(par);
        std::get<1>(state) += std::get<i1_>(par);
        std::get<1>(state) += static_cast<ResultType>(
            ThreefryInsertKeyInc<ResultType, inc_>::value);
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefryInsertKey

template <typename ResultType, std::size_t N>
struct ThreefryInsertKey<ResultType, 4, N, true> {
    static void eval(
        std::array<ResultType, 4> &state, const std::array<ResultType, 5> &par)
    {
        std::get<0>(state) += std::get<i0_>(par);
        std::get<1>(state) += std::get<i1_>(par);
        std::get<2>(state) += std::get<i2_>(par);
        std::get<3>(state) += std::get<i3_>(par);
        std::get<3>(state) += static_cast<ResultType>(
            ThreefryInsertKeyInc<ResultType, inc_>::value);
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefryInsertKey

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_THREEFRY_DEFINES_HPP
