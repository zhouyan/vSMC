//============================================================================
// vSMC/include/vsmc/rng/threefry_avx2.hpp
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

#ifndef VSMC_RNG_THREEFRY_AVX2_HPP
#define VSMC_RNG_THREEFRY_AVX2_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/internal/m256i.hpp>
#include <vsmc/rng/internal/threefry_defines.hpp>

namespace vsmc
{

namespace internal
{

template <typename ResultType, int R>
struct ThreefryRotateImplAVX2;

template <int R>
struct ThreefryRotateImplAVX2<std::uint32_t, R> {
    static __m256i eval(__m256i x)
    {
        __m256i a = _mm256_slli_epi32(x, R);
        __m256i b = _mm256_srli_epi32(x, 32 - R);

        return _mm256_or_si256(a, b);
    }
};

template <int R>
struct ThreefryRotateImplAVX2<std::uint64_t, R> {
    static __m256i eval(__m256i x)
    {
        __m256i a = _mm256_slli_epi64(x, R);
        __m256i b = _mm256_srli_epi64(x, 32 - R);

        return _mm256_or_si256(a, b);
    }
};

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct ThreefryRotateAVX2 {
    static void eval(std::array<__m256i, K> &) {}
};

template <std::size_t N>
struct ThreefryRotateAVX2<std::uint32_t, 2, N, true> {
    static void eval(std::array<__m256i, 2> &state)
    {
        std::get<0>(state) =
            _mm256_add_epi32(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefryRotateImplAVX2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm256_xor_si256(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateAVX2

template <std::size_t N>
struct ThreefryRotateAVX2<std::uint32_t, 4, N, true> {
    static void eval(std::array<__m256i, 4> &state)
    {
        std::get<0>(state) =
            _mm256_add_epi32(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefryRotateImplAVX2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm256_xor_si256(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm256_add_epi32(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefryRotateImplAVX2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm256_xor_si256(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateAVX2

template <std::size_t N>
struct ThreefryRotateAVX2<std::uint64_t, 2, N, true> {
    static void eval(std::array<__m256i, 2> &state)
    {
        std::get<0>(state) =
            _mm256_add_epi64(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefryRotateImplAVX2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm256_xor_si256(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateAVX2

template <std::size_t N>
struct ThreefryRotateAVX2<std::uint64_t, 4, N, true> {
    static void eval(std::array<__m256i, 4> &state)
    {
        std::get<0>(state) =
            _mm256_add_epi64(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefryRotateImplAVX2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm256_xor_si256(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm256_add_epi64(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefryRotateImplAVX2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm256_xor_si256(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateAVX2

template <typename ResultType, std::size_t K, std::size_t N,
    bool = (N % 4 == 0)>
struct ThreefryInsertKeyAVX2 {
    static void eval(
        std::array<__m256i, K> &, const std::array<__m256i, K + 1> &)
    {
    }
}; // struct ThreefryInsertKeyAVX2

template <std::size_t N>
struct ThreefryInsertKeyAVX2<std::uint32_t, 2, N, true> {
    static void eval(
        std::array<__m256i, 2> &state, const std::array<__m256i, 3> &par)
    {
        std::get<0>(state) =
            _mm256_add_epi32(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm256_add_epi32(std::get<1>(state), std::get<i1_>(par));
        std::get<1>(state) = _mm256_add_epi32(
            std::get<1>(state), _mm256_set1_epi32(static_cast<int>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefryInsertKeyAVX2

template <std::size_t N>
struct ThreefryInsertKeyAVX2<std::uint32_t, 4, N, true> {
    static void eval(
        std::array<__m256i, 4> &state, const std::array<__m256i, 5> &par)
    {
        std::get<0>(state) =
            _mm256_add_epi32(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm256_add_epi32(std::get<1>(state), std::get<i1_>(par));
        std::get<2>(state) =
            _mm256_add_epi32(std::get<2>(state), std::get<i2_>(par));
        std::get<3>(state) =
            _mm256_add_epi32(std::get<3>(state), std::get<i3_>(par));
        std::get<3>(state) = _mm256_add_epi32(
            std::get<3>(state), _mm256_set1_epi32(static_cast<int>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefryInsertKeyAVX2

template <std::size_t N>
struct ThreefryInsertKeyAVX2<std::uint64_t, 2, N, true> {
    static void eval(
        std::array<__m256i, 2> &state, const std::array<__m256i, 3> &par)
    {
        std::get<0>(state) =
            _mm256_add_epi64(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm256_add_epi64(std::get<1>(state), std::get<i1_>(par));
        std::get<1>(state) = _mm256_add_epi64(std::get<1>(state),
            _mm256_set1_epi64x(static_cast<long long>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 3;
    static constexpr std::size_t i1_ = (inc_ + 1) % 3;
}; // struct ThreefryInsertKeyAVX2

template <std::size_t N>
struct ThreefryInsertKeyAVX2<std::uint64_t, 4, N, true> {
    static void eval(
        std::array<__m256i, 4> &state, const std::array<__m256i, 5> &par)
    {
        std::get<0>(state) =
            _mm256_add_epi64(std::get<0>(state), std::get<i0_>(par));
        std::get<1>(state) =
            _mm256_add_epi64(std::get<1>(state), std::get<i1_>(par));
        std::get<2>(state) =
            _mm256_add_epi64(std::get<2>(state), std::get<i2_>(par));
        std::get<3>(state) =
            _mm256_add_epi64(std::get<3>(state), std::get<i3_>(par));
        std::get<3>(state) = _mm256_add_epi64(std::get<3>(state),
            _mm256_set1_epi64x(static_cast<long long>(inc_)));
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefryInsertKeyAVX2

template <typename, std::size_t>
struct ThreefryParPackAVX2;

template <>
struct ThreefryParPackAVX2<std::uint32_t, 2> {
    static void eval(
        const std::array<std::uint32_t, 3> &par, std::array<__m256i, 3> &pack)
    {
        std::get<0>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<2>(par)));
    }
}; // struct ThreefryParPackAVX2

template <>
struct ThreefryParPackAVX2<std::uint32_t, 4> {
    static void eval(
        const std::array<std::uint32_t, 5> &par, std::array<__m256i, 5> &pack)
    {
        std::get<0>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<2>(par)));
        std::get<3>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<3>(par)));
        std::get<4>(pack) =
            _mm256_set1_epi32(static_cast<int>(std::get<4>(par)));
    }
}; // struct ThreefryParPackAVX2

template <>
struct ThreefryParPackAVX2<std::uint64_t, 2> {
    static void eval(
        const std::array<std::uint64_t, 3> &par, std::array<__m256i, 3> &pack)
    {
        std::get<0>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<2>(par)));
    }
}; // struct ThreefryParPackAVX2

template <>
struct ThreefryParPackAVX2<std::uint64_t, 4> {
    static void eval(
        const std::array<std::uint64_t, 5> &par, std::array<__m256i, 5> &pack)
    {
        std::get<0>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<0>(par)));
        std::get<1>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<1>(par)));
        std::get<2>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<2>(par)));
        std::get<3>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<3>(par)));
        std::get<4>(pack) =
            _mm256_set1_epi64x(static_cast<long long>(std::get<4>(par)));
    }
}; // struct ThreefryParPackAVX2

template <typename, std::size_t>
struct ThreefryCtrPackAVX2;

template <>
struct ThreefryCtrPackAVX2<std::uint32_t, 2> {
    typedef std::array<__m256i, 2> state_type;
    typedef std::array<std::uint32_t, 2> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, state_type &state)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;
        counter::increment(ctr);
        ctr_type ctr4 = ctr;
        counter::increment(ctr);
        ctr_type ctr5 = ctr;
        counter::increment(ctr);
        ctr_type ctr6 = ctr;
        counter::increment(ctr);
        ctr_type ctr7 = ctr;

        std::get<0>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<0>(ctr7)),
                static_cast<int>(std::get<0>(ctr6)),
                static_cast<int>(std::get<0>(ctr5)),
                static_cast<int>(std::get<0>(ctr4)),
                static_cast<int>(std::get<0>(ctr3)),
                static_cast<int>(std::get<0>(ctr2)),
                static_cast<int>(std::get<0>(ctr1)),
                static_cast<int>(std::get<0>(ctr0)));
        std::get<1>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<1>(ctr7)),
                static_cast<int>(std::get<1>(ctr6)),
                static_cast<int>(std::get<1>(ctr5)),
                static_cast<int>(std::get<1>(ctr4)),
                static_cast<int>(std::get<1>(ctr3)),
                static_cast<int>(std::get<1>(ctr2)),
                static_cast<int>(std::get<1>(ctr1)),
                static_cast<int>(std::get<1>(ctr0)));
    }
}; // struct ThreefryCtrPackAVX2

template <>
struct ThreefryCtrPackAVX2<std::uint32_t, 4> {
    typedef std::array<__m256i, 4> state_type;
    typedef std::array<std::uint32_t, 4> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, state_type &state)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;
        counter::increment(ctr);
        ctr_type ctr4 = ctr;
        counter::increment(ctr);
        ctr_type ctr5 = ctr;
        counter::increment(ctr);
        ctr_type ctr6 = ctr;
        counter::increment(ctr);
        ctr_type ctr7 = ctr;

        std::get<0>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<0>(ctr7)),
                static_cast<int>(std::get<0>(ctr6)),
                static_cast<int>(std::get<0>(ctr5)),
                static_cast<int>(std::get<0>(ctr4)),
                static_cast<int>(std::get<0>(ctr3)),
                static_cast<int>(std::get<0>(ctr2)),
                static_cast<int>(std::get<0>(ctr1)),
                static_cast<int>(std::get<0>(ctr0)));
        std::get<1>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<1>(ctr7)),
                static_cast<int>(std::get<1>(ctr6)),
                static_cast<int>(std::get<1>(ctr5)),
                static_cast<int>(std::get<1>(ctr4)),
                static_cast<int>(std::get<1>(ctr3)),
                static_cast<int>(std::get<1>(ctr2)),
                static_cast<int>(std::get<1>(ctr1)),
                static_cast<int>(std::get<1>(ctr0)));
        std::get<2>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<2>(ctr7)),
                static_cast<int>(std::get<2>(ctr6)),
                static_cast<int>(std::get<2>(ctr5)),
                static_cast<int>(std::get<2>(ctr4)),
                static_cast<int>(std::get<2>(ctr3)),
                static_cast<int>(std::get<2>(ctr2)),
                static_cast<int>(std::get<2>(ctr1)),
                static_cast<int>(std::get<2>(ctr0)));
        std::get<3>(state) =
            _mm256_set_epi32(static_cast<int>(std::get<3>(ctr7)),
                static_cast<int>(std::get<3>(ctr6)),
                static_cast<int>(std::get<3>(ctr5)),
                static_cast<int>(std::get<3>(ctr4)),
                static_cast<int>(std::get<3>(ctr3)),
                static_cast<int>(std::get<3>(ctr2)),
                static_cast<int>(std::get<3>(ctr1)),
                static_cast<int>(std::get<3>(ctr0)));
    }
}; // struct ThreefryCtrPackAVX2

template <>
struct ThreefryCtrPackAVX2<std::uint64_t, 2> {
    typedef std::array<__m256i, 2> state_type;
    typedef std::array<std::uint64_t, 2> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, state_type &state)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;

        std::get<0>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<0>(ctr3)),
                static_cast<VSMC_INT64>(std::get<0>(ctr2)),
                static_cast<VSMC_INT64>(std::get<0>(ctr1)),
                static_cast<VSMC_INT64>(std::get<0>(ctr0)));
        std::get<1>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<1>(ctr3)),
                static_cast<VSMC_INT64>(std::get<1>(ctr2)),
                static_cast<VSMC_INT64>(std::get<1>(ctr1)),
                static_cast<VSMC_INT64>(std::get<1>(ctr0)));
    }
}; // struct ThreefryCtrPackAVX2

template <>
struct ThreefryCtrPackAVX2<std::uint64_t, 4> {
    typedef std::array<__m256i, 4> state_type;
    typedef std::array<std::uint64_t, 4> ctr_type;
    typedef Counter<ctr_type> counter;

    static void eval(ctr_type &ctr, state_type &state)
    {
        counter::increment(ctr);
        ctr_type ctr0 = ctr;
        counter::increment(ctr);
        ctr_type ctr1 = ctr;
        counter::increment(ctr);
        ctr_type ctr2 = ctr;
        counter::increment(ctr);
        ctr_type ctr3 = ctr;

        std::get<0>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<0>(ctr3)),
                static_cast<VSMC_INT64>(std::get<0>(ctr2)),
                static_cast<VSMC_INT64>(std::get<0>(ctr1)),
                static_cast<VSMC_INT64>(std::get<0>(ctr0)));
        std::get<1>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<1>(ctr3)),
                static_cast<VSMC_INT64>(std::get<1>(ctr2)),
                static_cast<VSMC_INT64>(std::get<1>(ctr1)),
                static_cast<VSMC_INT64>(std::get<1>(ctr0)));
        std::get<2>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<2>(ctr3)),
                static_cast<VSMC_INT64>(std::get<2>(ctr2)),
                static_cast<VSMC_INT64>(std::get<2>(ctr1)),
                static_cast<VSMC_INT64>(std::get<2>(ctr0)));
        std::get<3>(state) =
            _mm256_set_epi64x(static_cast<VSMC_INT64>(std::get<3>(ctr3)),
                static_cast<VSMC_INT64>(std::get<3>(ctr2)),
                static_cast<VSMC_INT64>(std::get<3>(ctr1)),
                static_cast<VSMC_INT64>(std::get<3>(ctr0)));
    }
}; // struct ThreefryCtrPackAVX2

template <typename, std::size_t>
struct ThreefryUnpackAVX2;

template <typename ResultType, std::size_t K>
struct ThreefryUnpackAVX2 {
    static constexpr std::size_t M = sizeof(__m256i) / sizeof(ResultType);

    static void eval(const std::array<__m256i, K> &state,
        std::array<ResultType, K * M> &buffer)
    {
        unpack<0>(state, buffer, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t>
    static void unpack(const std::array<__m256i, K> &,
        std::array<ResultType, K * M> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void unpack(const std::array<__m256i, K> &state,
        std::array<ResultType, K * M> &buffer, std::true_type)
    {
        internal::m256i_unpack<N * M>(std::get<N>(state), buffer);
        unpack<N + 1>(
            state, buffer, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryUnpackAVX2

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented using AVX2
/// \ingroup Threefry
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngineAVX2
{
    public:
    typedef ResultType result_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K> key_type;

    private:
    typedef Counter<ctr_type> counter;

    public:
    explicit ThreefryEngineAVX2(result_type s = 0) : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngineAVX2(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineAVX2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
        : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(seq);
    }

    ThreefryEngineAVX2(const key_type &k) : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(k);
    }

    void seed(result_type s)
    {
        counter::reset(ctr_);
        key_type k;
        k.fill(0);
        k.front() = s;
        init_par(k);
        index_ = K * M_;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineAVX2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
    {
        counter::reset(ctr_);
        key_type k;
        seq.generate(k.begin(), k.end());
        init_par(k);
        index_ = K * M_;
    }

    void seed(const key_type &k)
    {
        counter::reset(ctr_);
        init_par(k);
        index_ = K * M_;
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
        index_ = K * M_;
    }

    void key(const key_type &k)
    {
        init_par(k);
        index_ = K * M_;
    }

    result_type operator()()
    {
        if (index_ == K * M_) {
            generate_buffer();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= K * M_) {
            index_ += n;
            return;
        }

        n -= K * M_ - index_;
        if (n <= K * M_) {
            index_ = K * M_;
            operator()();
            index_ = n;
            return;
        }

        counter::increment(ctr_, static_cast<result_type>(n / (K * M_)));
        index_ = K * M_;
        operator()();
        index_ = n % (K * M_);
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(
        const ThreefryEngineAVX2<ResultType, K, Rounds> &eng1,
        const ThreefryEngineAVX2<ResultType, K, Rounds> &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.ctr_ == eng2.ctr_ &&
            eng1.par_ == eng2.par_;
    }

    friend bool operator!=(
        const ThreefryEngineAVX2<ResultType, K, Rounds> &eng1,
        const ThreefryEngineAVX2<ResultType, K, Rounds> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const ThreefryEngineAVX2<ResultType, K, Rounds> &eng)
    {
        if (!os.good())
            return os;

        os << eng.ctr_ << ' ';
        os << eng.par_ << ' ';
        os << eng.buffer_ << ' ';
        for (std::size_t k = 0; k != K; ++k) {
            internal::m256i_output(os, eng.buffer_[k]);
            os << ' ';
        }
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        ThreefryEngineAVX2<ResultType, K, Rounds> &eng)
    {
        if (!is.good())
            return is;

        ThreefryEngineAVX2<ResultType, K, Rounds> eng_tmp;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.par_;
        for (std::size_t k = 0; k != K; ++k)
            internal::m256i_input(is, eng_tmp.buffer_[k]);
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    typedef std::array<__m256i, K> state_type;
    typedef std::array<__m256i, K + 1> par_type;

    static constexpr std::size_t M_ = sizeof(__m256i) / sizeof(result_type);

    ctr_type ctr_;
    std::array<ResultType, K + 1> par_;
    std::array<ResultType, K * M_> buffer_;
    std::size_t index_;

    void generate_buffer()
    {
        par_type par;
        state_type state;
        internal::ThreefryParPackAVX2<ResultType, K>::eval(par_, par);
        internal::ThreefryCtrPackAVX2<ResultType, K>::eval(ctr_, state);
        generate_buffer<0>(par, state, std::true_type());
        internal::ThreefryUnpackAVX2<ResultType, K>::eval(state, buffer_);
    }

    template <std::size_t>
    void generate_buffer(const par_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(
        const par_type &par, state_type &state, std::true_type)
    {
        internal::ThreefryRotateAVX2<ResultType, K, N>::eval(state);
        internal::ThreefryInsertKeyAVX2<ResultType, K, N>::eval(state, par);
        generate_buffer<N + 1>(
            par, state, std::integral_constant < bool, N<Rounds>());
    }

    void init_par(const key_type &key)
    {
        par_.back() = internal::ThreefryKSConstantValue<ResultType>::value;
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
}; // class ThreefryEngineAVX2

/// \brief Threefry2x32 RNG engine reimplemented using AVX2
/// \ingroup Threefry
typedef ThreefryEngineAVX2<std::uint32_t, 2> Threefry2x32AVX2;

/// \brief Threefry4x32 RNG engine reimplemented using AVX2
/// \ingroup Threefry
typedef ThreefryEngineAVX2<std::uint32_t, 4> Threefry4x32AVX2;

/// \brief Threefry2x64 RNG engine reimplemented using AVX2
/// \ingroup Threefry
typedef ThreefryEngineAVX2<std::uint64_t, 2> Threefry2x64AVX2;

/// \brief Threefry4x64 RNG engine reimplemented using AVX2
/// \ingroup Threefry
typedef ThreefryEngineAVX2<std::uint64_t, 4> Threefry4x64AVX2;

/// \brief The default 32-bits Threefry engine using AVX2
/// \ingroup Threefry
typedef Threefry4x32AVX2 ThreefryAVX2;

/// \brief The default 64-bits Threefry engine using AVX2
/// \ingroup Threefry
typedef Threefry4x64AVX2 ThreefryAVX2_64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_AVX2_HPP
