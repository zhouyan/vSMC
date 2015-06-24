//============================================================================
// vSMC/include/vsmc/rng/threefry.hpp
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

#ifndef VSMC_RNG_THREEFRY_HPP
#define VSMC_RNG_THREEFRY_HPP

#include <vsmc/rng/internal/common.hpp>
#if VSMC_HAS_SSE2
#include <vsmc/rng/m128i.hpp>
#endif
#if VSMC_HAS_AVX2
#include <vsmc/rng/m256i.hpp>
#endif

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
    class ThreefryRotateConstant<T, K, N, I>                                  \
        : public std::integral_constant<int, val>                             \
    {                                                                         \
    }; // class ThreefryRotateConstant

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
class ThreefryKSConstant;

template <typename T, template <typename> class Wrapper>
class ThreefryKSConstant<Wrapper<T>> : public ThreefryKSConstant<T>
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

template <typename T, template <typename> class Wrapper, std::size_t K,
    std::size_t N, std::size_t I>
class ThreefryRotateConstant<Wrapper<T>, K, N, I>
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

template <typename ResultType, std::size_t K>
class ThreefryInitPar
{
    public:
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
}; // class ThreefryInitPar

template <typename ResultType>
class ThreefryRotateBits
    : public std::integral_constant<int, sizeof(ResultType) * 8>
{
}; // class ThreefryRotateBits

template <typename T, template <typename> class Wrapper>
class ThreefryRotateBits<Wrapper<T>> : public ThreefryRotateBits<T>
{
}; // class ThreefryRotateBits

template <typename ResultType, int R>
class ThreefryRotateImpl
{
    public:
    static ResultType eval(const ResultType &x)
    {
        return (x << R) | (x >> (ThreefryRotateBits<ResultType>::value - R));
    }
}; // class ThreefryRotateImpl

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 0)>
class ThreefryRotate
{
    public:
    static void eval(std::array<ResultType, K> &) {}
}; // class ThreefryRotate

template <typename ResultType, std::size_t N>
class ThreefryRotate<ResultType, 2, N, true>
{
    public:
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
}; // class ThreefryRotate

template <typename ResultType, std::size_t N>
class ThreefryRotate<ResultType, 4, N, true>
{
    public:
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
}; // class ThreefryRotate

template <typename ResultType, std::size_t Inc>
class ThreefryInsertKeyInc
    : public std::integral_constant<ResultType, static_cast<ResultType>(Inc)>
{
}; // class ThreefryInsertKeyInc

template <typename T, template <typename> class Wrapper, std::size_t Inc>
class ThreefryInsertKeyInc<Wrapper<T>, Inc>
    : public ThreefryInsertKeyInc<T, Inc>
{
}; // class ThreefryInsertKeyInc

template <typename ResultType, std::size_t K, std::size_t N,
    bool = (N % 4 == 0)>
class ThreefryInsertKey
{
    public:
    static void eval(
        std::array<ResultType, K> &, const std::array<ResultType, K + 1> &)
    {
    }
}; // class ThreefryInsertKey

template <typename ResultType, std::size_t N>
class ThreefryInsertKey<ResultType, 2, N, true>
{
    public:
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
}; // class ThreefryInsertKey

template <typename ResultType, std::size_t N>
class ThreefryInsertKey<ResultType, 4, N, true>
{
    public:
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
}; // class ThreefryInsertKey

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented
/// \ingroup Threefry
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngine
{
    public:
    using result_type = ResultType;
    using key_type = std::array<ResultType, K>;
    using ctr_type = std::array<ResultType, K>;

    explicit ThreefryEngine(result_type s = 0) : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY();
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngine<ResultType, K, Rounds>>::value>::type * =
            nullptr)
        : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY();
        seed(seq);
    }

    ThreefryEngine(const key_type &k) : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY();
        seed(k);
    }

    void seed(result_type s)
    {
        ctr_.fill(0);
        key_type k;
        k.fill(0);
        k.front() = s;
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = K;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngine<ResultType, K, Rounds>>::value>::type * =
            nullptr)
    {
        ctr_.fill(0);
        key_type k;
        seq.generate(k.begin(), k.end());
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = K;
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = K;
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
        ctr_ = c;
        index_ = K;
    }

    void key(const key_type &k)
    {
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = K;
    }

    result_type operator()()
    {
        if (index_ == K) {
            generate_buffer();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= K) {
            index_ += n;
            return;
        }

        n -= K - index_;
        if (n <= K) {
            index_ = K;
            operator()();
            index_ = n;
            return;
        }

        internal::increment(ctr_, static_cast<result_type>(n / K));
        index_ = K;
        operator()();
        index_ = n % K;
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    friend bool operator==(const ThreefryEngine<ResultType, K, Rounds> &eng1,
        const ThreefryEngine<ResultType, K, Rounds> &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.ctr_ == eng2.ctr_ &&
            eng1.par_ == eng2.par_;
    }

    friend bool operator!=(const ThreefryEngine<ResultType, K, Rounds> &eng1,
        const ThreefryEngine<ResultType, K, Rounds> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const ThreefryEngine<ResultType, K, Rounds> &eng)
    {
        if (!os.good())
            return os;

        os << eng.buffer_ << ' ';
        os << eng.par_ << ' ';
        os << eng.ctr_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        ThreefryEngine<ResultType, K, Rounds> &eng)
    {
        if (!is.good())
            return is;

        ThreefryEngine<ResultType, K, Rounds> eng_tmp;
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.par_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    std::array<ResultType, K> buffer_;
    std::array<ResultType, K + 1> par_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        internal::increment(ctr_);
        buffer_ = ctr_;
        generate_buffer<0>(std::true_type());
    }

    template <std::size_t>
    void generate_buffer(std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(std::true_type)
    {
        internal::ThreefryRotate<ResultType, K, N>::eval(buffer_);
        internal::ThreefryInsertKey<ResultType, K, N>::eval(buffer_, par_);
        generate_buffer<N + 1>(std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryEngine

/// \brief Threefry2x32 RNG engine reimplemented
/// \ingroup Threefry
using Threefry2x32 = ThreefryEngine<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine reimplemented
/// \ingroup Threefry
using Threefry4x32 = ThreefryEngine<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine reimplemented
/// \ingroup Threefry
using Threefry2x64 = ThreefryEngine<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine reimplemented
/// \ingroup Threefry
using Threefry4x64 = ThreefryEngine<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine
/// \ingroup Threefry
using Threefry = Threefry4x32;

/// \brief The default 64-bits Threefry engine
/// \ingroup Threefry
using Threefry_64 = Threefry4x64;

#if VSMC_HAS_AVX2

namespace internal
{

template <typename ResultType, std::size_t K>
class ThreefryParPackAVX2
{
    public:
    using par_type = std::array<ResultType, K + 1>;
    using par256_type = std::array<M256I<ResultType>, K + 1>;

    static void eval(const par_type &par, par256_type &par256)
    {
        pack<0>(par, par256, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const par_type &, par256_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const par_type &par, par256_type &par256, std::true_type)
    {
        std::get<N>(par256).set1(std::get<N>(par));
        pack<N + 1>(
            par, par256, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // class ThreefryParPackAVX2

template <typename ResultType, std::size_t K>
class ThreefryCtrPackAVX2
{
    public:
    using state_type = std::array<M256I<ResultType>, K>;
    using ctr_type = std::array<ResultType, K>;
    using ctr_block_type = std::array<ctr_type, M256I<ResultType>::size()>;

    static void eval(ctr_type &ctr, state_type &state)
    {
        ctr_block_type ctr_block;
        increment(ctr, ctr_block);
        pack<0>(ctr_block, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(const ctr_block_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(
        const ctr_block_type &ctr_block, state_type &state, std::true_type)
    {
        set<N>(ctr_block, state,
            std::integral_constant<std::size_t, sizeof(ResultType)>());
        pack<N + 1>(
            ctr_block, state, std::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t N>
    static void set(const ctr_block_type &ctr_block, state_type &state,
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
    static void set(const ctr_block_type &ctr_block, state_type &state,
        std::integral_constant<std::size_t, 8>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)),
            std::get<N>(std::get<2>(ctr_block)),
            std::get<N>(std::get<3>(ctr_block)));
    }
}; // class ThreefryCtrPackAVX2

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented using AVX2
/// \ingroup Threefry
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngineAVX2
{
    public:
    using result_type = ResultType;
    using key_type = std::array<ResultType, K>;
    using ctr_type = std::array<ResultType, K>;

    explicit ThreefryEngineAVX2(result_type s = 0) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngineAVX2(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineAVX2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
        : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(seq);
    }

    ThreefryEngineAVX2(const key_type &k) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(AVX2);
        seed(k);
    }

    void seed(result_type s)
    {
        ctr_.fill(0);
        key_type k;
        k.fill(0);
        k.front() = s;
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineAVX2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
    {
        ctr_.fill(0);
        key_type k;
        seq.generate(k.begin(), k.end());
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
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
        ctr_ = c;
        index_ = M_;
    }

    void key(const key_type &k)
    {
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generate_buffer();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= M_) {
            index_ += n;
            return;
        }

        n -= M_ - index_;
        if (n <= M_) {
            index_ = M_;
            operator()();
            index_ = n;
            return;
        }

        internal::increment(ctr_, static_cast<result_type>(n / M_));
        index_ = M_;
        operator()();
        index_ = n % M_;
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

        os << eng.buffer_ << ' ';
        os << eng.par_ << ' ';
        os << eng.ctr_ << ' ';
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
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.par_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    using par_type = std::array<M256I<ResultType>, K + 1>;
    using state_type = std::array<M256I<ResultType>, K>;

    static constexpr std::size_t M_ = K * M256I<ResultType>::size();

    alignas(32) std::array<ResultType, M_> buffer_;
    std::array<ResultType, K + 1> par_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        par_type par;
        union {
            state_type state;
            std::array<ResultType, M_> result;
        } buf;
        internal::ThreefryParPackAVX2<ResultType, K>::eval(par_, par);
        internal::ThreefryCtrPackAVX2<ResultType, K>::eval(ctr_, buf.state);
        generate_buffer<0>(par, buf.state, std::true_type());
        buffer_ = buf.result;
    }

    template <std::size_t>
    void generate_buffer(const par_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(
        const par_type &par, state_type &state, std::true_type)
    {
        internal::ThreefryRotate<M256I<ResultType>, K, N>::eval(state);
        internal::ThreefryInsertKey<M256I<ResultType>, K, N>::eval(state, par);
        generate_buffer<N + 1>(
            par, state, std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryEngineAVX2

/// \brief Threefry2x32 RNG engine reimplemented using AVX2
/// \ingroup Threefry
using Threefry2x32AVX2 = ThreefryEngineAVX2<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine reimplemented using AVX2
/// \ingroup Threefry
using Threefry4x32AVX2 = ThreefryEngineAVX2<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine reimplemented using AVX2
/// \ingroup Threefry
using Threefry2x64AVX2 = ThreefryEngineAVX2<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine reimplemented using AVX2
/// \ingroup Threefry
using Threefry4x64AVX2 = ThreefryEngineAVX2<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine using AVX2
/// \ingroup Threefry
using ThreefryAVX2 = Threefry4x32AVX2;

/// \brief The default 64-bits Threefry engine using AVX2
/// \ingroup Threefry
using ThreefryAVX2_64 = Threefry4x64AVX2;

#endif // VSMC_HAS_AVX2

#if VSMC_HAS_AVX2

#endif // VSMC_HAS_AVX2

namespace internal
{

template <typename ResultType, std::size_t K>
class ThreefryParPackSSE2
{
    public:
    using par_type = std::array<ResultType, K + 1>;
    using par128_type = std::array<M128I<ResultType>, K + 1>;

    static void eval(const par_type &par, par128_type &par128)
    {
        pack<0>(par, par128, std::integral_constant<bool, 0 < K + 1>());
    }

    private:
    template <std::size_t>
    static void pack(const par_type &, par128_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(const par_type &par, par128_type &par128, std::true_type)
    {
        std::get<N>(par128).set1(std::get<N>(par));
        pack<N + 1>(
            par, par128, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // class ThreefryParPackSSE2

template <typename ResultType, std::size_t K>
class ThreefryCtrPackSSE2
{
    public:
    using state_type = std::array<M128I<ResultType>, K>;
    using ctr_type = std::array<ResultType, K>;
    using ctr_block_type = std::array<ctr_type, M128I<ResultType>::size()>;

    static void eval(ctr_type &ctr, state_type &state)
    {
        ctr_block_type ctr_block;
        increment(ctr, ctr_block);
        pack<0>(ctr_block, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(const ctr_block_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(
        const ctr_block_type &ctr_block, state_type &state, std::true_type)
    {
        set<N>(ctr_block, state,
            std::integral_constant<std::size_t, sizeof(ResultType)>());
        pack<N + 1>(
            ctr_block, state, std::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t N>
    static void set(const ctr_block_type &ctr_block, state_type &state,
        std::integral_constant<std::size_t, 4>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)),
            std::get<N>(std::get<2>(ctr_block)),
            std::get<N>(std::get<3>(ctr_block)));
    }

    template <std::size_t N>
    static void set(const ctr_block_type &ctr_block, state_type &state,
        std::integral_constant<std::size_t, 8>)
    {
        std::get<N>(state).set(std::get<N>(std::get<0>(ctr_block)),
            std::get<N>(std::get<1>(ctr_block)));
    }
}; // class ThreefryCtrPackSSE2

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented using SSE2
/// \ingroup Threefry
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngineSSE2
{
    public:
    using result_type = ResultType;
    using key_type = std::array<ResultType, K>;
    using ctr_type = std::array<ResultType, K>;

    explicit ThreefryEngineSSE2(result_type s = 0) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
        seed(s);
    }

    template <typename SeedSeq>
    explicit ThreefryEngineSSE2(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineSSE2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
        : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
        seed(seq);
    }

    ThreefryEngineSSE2(const key_type &k) : index_(M_)
    {
        VSMC_STATIC_ASSERT_RNG_THREEFRY(SSE2);
        seed(k);
    }

    void seed(result_type s)
    {
        ctr_.fill(0);
        key_type k;
        k.fill(0);
        k.front() = s;
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngineSSE2<ResultType, K, Rounds>>::value>::type
            * = nullptr)
    {
        ctr_.fill(0);
        key_type k;
        seq.generate(k.begin(), k.end());
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
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
        ctr_ = c;
        index_ = M_;
    }

    void key(const key_type &k)
    {
        internal::ThreefryInitPar<ResultType, K>::eval(k, par_);
        index_ = M_;
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generate_buffer();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void discard(result_type nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);
        if (index_ + n <= M_) {
            index_ += n;
            return;
        }

        n -= M_ - index_;
        if (n <= M_) {
            index_ = M_;
            operator()();
            index_ = n;
            return;
        }

        internal::increment(ctr_, static_cast<result_type>(n / M_));
        index_ = M_;
        operator()();
        index_ = n % M_;
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

        os << eng.buffer_ << ' ';
        os << eng.par_ << ' ';
        os << eng.ctr_ << ' ';
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
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    using par_type = std::array<M128I<ResultType>, K + 1>;
    using state_type = std::array<M128I<ResultType>, K>;

    static constexpr std::size_t M_ = K * M128I<ResultType>::size();

    alignas(16) std::array<ResultType, M_> buffer_;
    std::array<ResultType, K + 1> par_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        par_type par;
        union {
            state_type state;
            std::array<ResultType, M_> result;
        } buf;
        internal::ThreefryParPackSSE2<ResultType, K>::eval(par_, par);
        internal::ThreefryCtrPackSSE2<ResultType, K>::eval(ctr_, buf.state);
        generate_buffer<0>(par, buf.state, std::true_type());
        buffer_ = buf.result;
    }

    template <std::size_t>
    void generate_buffer(const par_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(
        const par_type &par, state_type &state, std::true_type)
    {
        internal::ThreefryRotate<M128I<ResultType>, K, N>::eval(state);
        internal::ThreefryInsertKey<M128I<ResultType>, K, N>::eval(state, par);
        generate_buffer<N + 1>(
            par, state, std::integral_constant < bool, N<Rounds>());
    }
}; // class ThreefryEngineSSE2

/// \brief Threefry2x32 RNG engine reimplemented using SSE2
/// \ingroup Threefry
using Threefry2x32SSE2 = ThreefryEngineSSE2<std::uint32_t, 2>;

/// \brief Threefry4x32 RNG engine reimplemented using SSE2
/// \ingroup Threefry
using Threefry4x32SSE2 = ThreefryEngineSSE2<std::uint32_t, 4>;

/// \brief Threefry2x64 RNG engine reimplemented using SSE2
/// \ingroup Threefry
using Threefry2x64SSE2 = ThreefryEngineSSE2<std::uint64_t, 2>;

/// \brief Threefry4x64 RNG engine reimplemented using SSE2
/// \ingroup Threefry
using Threefry4x64SSE2 = ThreefryEngineSSE2<std::uint64_t, 4>;

/// \brief The default 32-bits Threefry engine using SSE2
/// \ingroup Threefry
using ThreefrySSE2 = Threefry4x32SSE2;

/// \brief The default 64-bits ThreefrySSE2 engine using SSE2
/// \ingroup Threefry
using ThreefrySSE2_64 = Threefry4x64SSE2;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
