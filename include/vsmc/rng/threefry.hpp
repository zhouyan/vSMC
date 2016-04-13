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

#define VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(K, I, val)                \
    template <>                                                               \
    class ThreefryPermutateConstant<K, I>                                     \
        : public std::integral_constant<std::size_t, val>                     \
    {                                                                         \
    }; // class ThreefryPermutateConstant

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

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 0, 0, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 1, 0, 33)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 2, 0, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 3, 0, 44)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 4, 0, 39)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 5, 0, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 6, 0, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 7, 0, 8)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 0, 1, 36)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 1, 1, 27)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 2, 1, 49)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 3, 1, 9)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 4, 1, 30)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 5, 1, 50)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 6, 1, 29)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 7, 1, 35)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 0, 2, 19)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 1, 2, 14)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 2, 2, 36)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 3, 2, 54)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 4, 2, 34)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 5, 2, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 6, 2, 39)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 7, 2, 56)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 0, 3, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 1, 3, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 2, 3, 39)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 3, 3, 56)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 4, 3, 24)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 5, 3, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 6, 3, 43)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 8, 7, 3, 22)

VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 0, 24)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 0, 38)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 0, 33)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 0, 5)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 0, 41)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 0, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 0, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 0, 9)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 1, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 1, 19)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 1, 4)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 1, 20)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 1, 9)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 1, 34)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 1, 44)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 1, 48)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 2, 8)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 2, 10)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 2, 51)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 2, 48)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 2, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 2, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 2, 47)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 2, 35)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 3, 47)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 3, 55)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 3, 13)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 3, 41)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 3, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 3, 51)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 3, 46)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 3, 52)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 4, 8)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 4, 49)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 4, 34)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 4, 47)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 4, 12)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 4, 4)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 4, 19)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 4, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 5, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 5, 18)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 5, 41)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 5, 28)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 5, 47)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 5, 53)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 5, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 5, 31)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 6, 22)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 6, 23)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 6, 59)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 6, 16)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 6, 44)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 6, 42)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 6, 44)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 6, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 0, 7, 37)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 1, 7, 52)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 2, 7, 17)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 3, 7, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 4, 7, 30)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 5, 7, 41)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 6, 7, 25)
VSMC_DEFINE_RNG_THREEFRY_ROTATE_CONSTANT(64, 16, 7, 7, 20)

template <std::size_t, std::size_t>
class ThreefryPermutateConstant;

VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(2, 0, 0)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(2, 1, 1)

VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(4, 0, 0)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(4, 1, 3)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(4, 2, 2)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(4, 3, 1)

VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 0, 2)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 1, 1)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 2, 4)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 3, 7)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 4, 6)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 5, 5)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 6, 0)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(8, 7, 3)

VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 0, 0)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 1, 9)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 2, 2)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 3, 13)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 4, 6)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 5, 11)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 6, 4)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 7, 15)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 8, 10)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 9, 7)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 10, 12)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 11, 3)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 12, 14)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 13, 5)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 14, 8)
VSMC_DEFINE_RNG_THREEFRY_PERMUTATE_CONSTANT(16, 15, 1)

template <typename T, std::size_t K, std::size_t N, bool = (N > 0)>
class ThreefryMix
{
    public:
    static void eval(std::array<T, K> &) {}
}; // class ThreefryMix

template <typename T, std::size_t K, std::size_t N>
class ThreefryMix<T, K, N, true>
{
    public:
    static void eval(std::array<T, K> &state)
    {
        eval<0>(state, std::integral_constant<bool, 1 < K>());
    }

    private:
    template <std::size_t>
    static void eval(std::array<T, K> &, std::false_type)
    {
    }

    template <std::size_t I>
    static void eval(std::array<T, K> &state, std::true_type)
    {
        static constexpr int L =
            ThreefryRotateConstant<T, K, (N - 1) % 8, I / 2>::value;
        static constexpr int R = std::numeric_limits<T>::digits - L;

        T x = std::get<I + 1>(state);
        std::get<I>(state) += x;
        std::get<I + 1>(state) = std::get<I>(state) ^ ((x << L) | (x >> R));
        eval<I + 2>(state, std::integral_constant<bool, I + 3 < K>());
    }
}; // class ThreefryMix

template <typename T, std::size_t K, std::size_t N, bool = (N > 0)>
class ThreefryPermutate
{
    public:
    static void eval(std::array<T, K> &) {}
}; // class ThreefryPermutate

template <typename T, std::size_t K, std::size_t N>
class ThreefryPermutate<T, K, N, true>
{
    public:
    static void eval(std::array<T, K> &state)
    {
        std::array<T, K> tmp;
        eval<0>(state, tmp, std::integral_constant<bool, 0 < K>());
        std::memcpy(state.data(), tmp.data(), sizeof(T) * K);
    }

    private:
    template <std::size_t>
    static void eval(
        const std::array<T, K> &, std::array<T, K> &, std::false_type)
    {
    }

    template <std::size_t I>
    static void eval(
        const std::array<T, K> &state, std::array<T, K> &tmp, std::true_type)
    {
        std::get<I>(tmp) =
            std::get<ThreefryPermutateConstant<K, I>::value>(state);
        eval<I + 1>(state, tmp, std::integral_constant<bool, I + 1 < K>());
    }
}; // class ThreefryPermutate

template <typename T, std::size_t N>
class ThreefryPermutate<T, 2, N, true>
{
    public:
    static void eval(std::array<T, 2> &) {}
}; // class ThreefryPermutate

template <typename T, std::size_t N>
class ThreefryPermutate<T, 4, N, true>
{
    public:
    static void eval(std::array<T, 4> &state)
    {
        std::swap(std::get<1>(state), std::get<3>(state));
    }
}; // class ThreefryPermutate

template <typename T, std::size_t N>
class ThreefryPermutate<T, 8, N, true>
{
    public:
    static void eval(std::array<T, 8> &state)
    {
        std::swap(std::get<3>(state), std::get<7>(state));
        T x = std::get<0>(state);
        std::get<0>(state) = std::get<2>(state);
        std::get<2>(state) = std::get<4>(state);
        std::get<4>(state) = std::get<6>(state);
        std::get<6>(state) = x;
    }
}; // class ThreefryPermutate

template <typename T, std::size_t K, std::size_t N, bool = (N % 4 == 0)>
class ThreefryInsertKey
{
    public:
    static void eval(std::array<T, K> &, const std::array<T, K + 1> &) {}
}; // class ThreefryInsertKey

template <typename T, std::size_t K, std::size_t N>
class ThreefryInsertKey<T, K, N, true>
{
    public:
    static void eval(std::array<T, K> &state, const std::array<T, K + 1> &par)
    {
        eval<0>(state, par, std::integral_constant<bool, 0 < K>());
        state.back() += static_cast<T>(s_);
    }

    private:
    static constexpr std::size_t s_ = N / 4;

    template <std::size_t>
    static void eval(
        std::array<T, K> &, const std::array<T, K + 1> &, std::false_type)
    {
    }

    template <std::size_t I>
    static void eval(std::array<T, K> &state, const std::array<T, K + 1> &par,
        std::true_type)
    {
        std::get<I>(state) += std::get<(s_ + I) % (K + 1)>(par);
        eval<I + 1>(state, par, std::integral_constant<bool, I + 1 < K>());
    }
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

    static_assert(K == 2 || K == 4 ||
            ((K == 8 || K == 16) && (std::numeric_limits<T>::digits == 64)),
        "**ThreefryGenerator** USED WITH K OTHER THAN 2 OR 4 (32 OR 64 BITS) "
        "OR 8 OR 16 (64 BITS)");

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
        internal::ThreefryMix<T, K, N>::eval(state);
        internal::ThreefryPermutate<T, K, N>::eval(state);
        internal::ThreefryInsertKey<T, K, N>::eval(state, par);
        generate<N + 1>(
            state, par, std::integral_constant<bool, (N < Rounds)>());
    }

    private:
    T par_;
}; // class ThreefryGenerator

/// \brief Threefry RNG engine
/// \ingroup Threefry
template <typename ResultType, typename T = ResultType,
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

/// \brief Threefry8x64 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry8x64Engine = ThreefryEngine<ResultType, std::uint64_t, 8>;

/// \brief Threefry16x64 RNG engine
/// \ingroup Threefry
template <typename ResultType>
using Threefry16x64Engine = ThreefryEngine<ResultType, std::uint64_t, 16>;

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

/// \brief Threefry8x64 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry8x64 = Threefry8x64Engine<std::uint32_t>;

/// \brief Threefry16x64 RNG engine with 32-bit integer output
/// \ingroup Threefry
using Threefry16x64 = Threefry16x64Engine<std::uint32_t>;

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

/// \brief Threefry8x64 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry8x64_64 = Threefry8x64Engine<std::uint64_t>;

/// \brief Threefry16x64 RNG engine with 64-bit integer output
/// \ingroup Threefry
using Threefry16x64_64 = Threefry16x64Engine<std::uint64_t>;

/// \brief The default 32-bit Threefry engine
/// \ingroup Threefry
using Threefry = Threefry4x64Engine<std::uint32_t>;

/// \brief The default 64-bit Threefry engine
/// \ingroup Threefry
using Threefry_64 = Threefry4x64Engine<std::uint64_t>;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
