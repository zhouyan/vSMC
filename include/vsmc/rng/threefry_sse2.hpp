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
#include <vsmc/rng/internal/threefry_defines.hpp>

namespace vsmc
{

namespace internal
{

template <typename ResultType, int R>
struct ThreefryRotateImplSSE2;

template <int R>
struct ThreefryRotateImplSSE2<std::uint32_t, R> {
    static __m128i eval(__m128i x)
    {
        __m128i a = _mm_slli_epi32(x, R);
        __m128i b = _mm_srli_epi32(x, 32 - R);

        return _mm_or_si128(a, b);
    }
};

template <int R>
struct ThreefryRotateImplSSE2<std::uint64_t, R> {
    static __m128i eval(__m128i x)
    {
        __m128i a = _mm_slli_epi64(x, R);
        __m128i b = _mm_srli_epi64(x, 64 - R);

        return _mm_or_si128(a, b);
    }
};

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
struct ThreefryRotateSSE2 {
    static void eval(std::array<__m128i, K> &) {}
};

template <std::size_t N>
struct ThreefryRotateSSE2<std::uint32_t, 2, N, true> {
    static void eval(std::array<__m128i, 2> &state)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefryRotateImplSSE2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateSSE2

template <std::size_t N>
struct ThreefryRotateSSE2<std::uint32_t, 4, N, true> {
    static void eval(std::array<__m128i, 4> &state)
    {
        std::get<0>(state) =
            _mm_add_epi32(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefryRotateImplSSE2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm_add_epi32(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefryRotateImplSSE2<std::uint32_t,
                ThreefryRotateConstantValue<std::uint32_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm_xor_si128(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateSSE2

template <std::size_t N>
struct ThreefryRotateSSE2<std::uint64_t, 2, N, true> {
    static void eval(std::array<__m128i, 2> &state)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<1>(state));
        std::get<1>(state) =
            ThreefryRotateImplSSE2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 2, r_,
                                       0>::value>::eval(std::get<1>(state));
        std::get<1>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<1>(state));
    }

    private:
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateSSE2

template <std::size_t N>
struct ThreefryRotateSSE2<std::uint64_t, 4, N, true> {
    static void eval(std::array<__m128i, 4> &state)
    {
        std::get<0>(state) =
            _mm_add_epi64(std::get<0>(state), std::get<i0_>(state));
        std::get<i0_>(state) =
            ThreefryRotateImplSSE2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 4, r_,
                                       0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) =
            _mm_xor_si128(std::get<0>(state), std::get<i0_>(state));

        std::get<2>(state) =
            _mm_add_epi64(std::get<2>(state), std::get<i2_>(state));
        std::get<i2_>(state) =
            ThreefryRotateImplSSE2<std::uint64_t,
                ThreefryRotateConstantValue<std::uint64_t, 4, r_,
                                       1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) =
            _mm_xor_si128(std::get<2>(state), std::get<i2_>(state));
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotateSSE2

template <typename ResultType, std::size_t K, std::size_t N,
    bool = (N % 4 == 0)>
struct ThreefryInsertKeySSE2 {
    static void eval(
        std::array<__m128i, K> &, const std::array<__m128i, K + 1> &)
    {
    }
}; // struct ThreefryInsertKeySSE2

template <std::size_t N>
struct ThreefryInsertKeySSE2<std::uint32_t, 2, N, true> {
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
}; // struct ThreefryInsertKeySSE2

template <std::size_t N>
struct ThreefryInsertKeySSE2<std::uint32_t, 4, N, true> {
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
}; // struct ThreefryInsertKeySSE2

template <std::size_t N>
struct ThreefryInsertKeySSE2<std::uint64_t, 2, N, true> {
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
}; // struct ThreefryInsertKeySSE2

template <std::size_t N>
struct ThreefryInsertKeySSE2<std::uint64_t, 4, N, true> {
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
}; // struct ThreefryInsertKeySSE2

template <typename, std::size_t>
struct ThreefryParPackSSE2;

template <std::size_t K>
struct ThreefryParPackSSE2<std::uint32_t, K> {
    static void eval(const std::array<std::uint32_t, K + 1> &par,
        std::array<__m128i, K + 1> &par128)
    {
        pack<0>(par, par128, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const std::array<std::uint32_t, K + 1> &,
        std::array<__m128i, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<std::uint32_t, K + 1> &par,
        std::array<__m128i, K + 1> &par128, std::true_type)
    {
        std::get<N>(par128) =
            _mm_set1_epi32(static_cast<int>(std::get<N>(par)));
        pack<N + 1>(
            par, par128, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // ThreefryParPackSSE2

template <std::size_t K>
struct ThreefryParPackSSE2<std::uint64_t, K> {
    static void eval(const std::array<std::uint64_t, K + 1> &par,
        std::array<__m128i, K + 1> &par128)
    {
        pack<0>(par, par128, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const std::array<std::uint64_t, K + 1> &,
        std::array<__m128i, K + 1> &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<std::uint64_t, K + 1> &par,
        std::array<__m128i, K + 1> &par128, std::true_type)
    {
        std::get<N>(par128) =
            _mm_set1_epi64x(static_cast<VSMC_INT64>(std::get<N>(par)));
        pack<N + 1>(
            par, par128, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // ThreefryParPackSSE2

template <typename, std::size_t>
struct ThreefryCtrPackSSE2;

template <std::size_t K>
struct ThreefryCtrPackSSE2<std::uint32_t, K> {
    typedef std::array<__m128i, K> state_type;
    typedef std::array<std::uint32_t, K> ctr_type;

    static void eval(ctr_type &ctr, state_type &state)
    {
        std::array<ctr_type, 4> ctr_blocks;
        Counter<ctr_type>::increment(ctr, ctr_blocks);
        pack<0>(ctr_blocks, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(
        const std::array<ctr_type, 4> &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(
        std::array<ctr_type, 4> &ctr_blocks, state_type &state, std::true_type)
    {
        std::get<N>(state) = _mm_set_epi32(
            static_cast<int>(std::get<N>(std::get<0>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<1>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<2>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<3>(ctr_blocks))));
        pack<N + 1>(
            ctr_blocks, state, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryCtrPackSSE2

template <std::size_t K>
struct ThreefryCtrPackSSE2<std::uint64_t, K> {
    typedef std::array<__m128i, K> state_type;
    typedef std::array<std::uint64_t, K> ctr_type;

    static void eval(ctr_type &ctr, state_type &state)
    {
        std::array<ctr_type, 2> ctr_blocks;
        Counter<ctr_type>::increment(ctr, ctr_blocks);
        pack<0>(ctr_blocks, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(
        const std::array<ctr_type, 2> &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const std::array<ctr_type, 2> &ctr_blocks,
        state_type &state, std::true_type)
    {
        std::get<N>(state) = _mm_set_epi64x(
            static_cast<VSMC_INT64>(std::get<N>(std::get<0>(ctr_blocks))),
            static_cast<VSMC_INT64>(std::get<N>(std::get<1>(ctr_blocks))));
        pack<N + 1>(
            ctr_blocks, state, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryCtrPackSSE2

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented using SSE2
/// \ingroup Threefry
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngineSSE2
{
    public:
    typedef ResultType result_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K> key_type;

    private:
    typedef Counter<ctr_type> counter;

    public:
    explicit ThreefryEngineSSE2(result_type s = 0) : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngineSSE2(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineSSE2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
        : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
        seed(seq);
    }

    ThreefryEngineSSE2(const key_type &k) : index_(K * M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
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
            key_type, ThreefryEngineSSE2<ResultType, K, Rounds>>::value>::type
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
        const ThreefryEngineSSE2<ResultType, K, Rounds> &eng1,
        const ThreefryEngineSSE2<ResultType, K, Rounds> &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.ctr_ == eng2.ctr_ &&
            eng1.par_ == eng2.par_;
    }

    friend bool operator!=(
        const ThreefryEngineSSE2<ResultType, K, Rounds> &eng1,
        const ThreefryEngineSSE2<ResultType, K, Rounds> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const ThreefryEngineSSE2<ResultType, K, Rounds> &eng)
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
        ThreefryEngineSSE2<ResultType, K, Rounds> &eng)
    {
        if (!is.good())
            return is;

        ThreefryEngineSSE2<ResultType, K, Rounds> eng_tmp;
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
    typedef std::array<__m128i, K> state_type;
    typedef std::array<__m128i, K + 1> par_type;

    static constexpr std::size_t M_ = sizeof(__m128i) / sizeof(result_type);

    ctr_type ctr_;
    std::array<ResultType, K + 1> par_;
    std::array<ResultType, K * M_> buffer_;
    std::size_t index_;

    void generate_buffer()
    {
        par_type par;
        state_type state;
        internal::ThreefryParPackSSE2<ResultType, K>::eval(par_, par);
        internal::ThreefryCtrPackSSE2<ResultType, K>::eval(ctr_, state);
        generate_buffer<0>(par, state, std::true_type());
        std::memcpy(buffer_.data(), state.data(), sizeof(__m128i) * K);
    }

    template <std::size_t>
    void generate_buffer(const par_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(
        const par_type &par, state_type &state, std::true_type)
    {
        internal::ThreefryRotateSSE2<ResultType, K, N>::eval(state);
        internal::ThreefryInsertKeySSE2<ResultType, K, N>::eval(state, par);
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
}; // class ThreefryEngineSSE2

/// \brief Threefry2x32 RNG engine reimplemented using SSE2
/// \ingroup Threefry
typedef ThreefryEngineSSE2<std::uint32_t, 2> Threefry2x32SSE2;

/// \brief Threefry4x32 RNG engine reimplemented using SSE2
/// \ingroup Threefry
typedef ThreefryEngineSSE2<std::uint32_t, 4> Threefry4x32SSE2;

/// \brief Threefry2x64 RNG engine reimplemented using SSE2
/// \ingroup Threefry
typedef ThreefryEngineSSE2<std::uint64_t, 2> Threefry2x64SSE2;

/// \brief Threefry4x64 RNG engine reimplemented using SSE2
/// \ingroup Threefry
typedef ThreefryEngineSSE2<std::uint64_t, 4> Threefry4x64SSE2;

/// \brief The default 32-bits Threefry engine using SSE2
/// \ingroup Threefry
typedef Threefry4x32SSE2 ThreefrySSE2;

/// \brief The default 64-bits ThreefrySSE2 engine using SSE2
/// \ingroup Threefry
typedef Threefry4x64SSE2 ThreefrySSE2_64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_SSE2_HPP
