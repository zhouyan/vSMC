//============================================================================
// vSMC/include/vsmc/rng/philox.hpp
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

#ifndef VSMC_RNG_PHILOX_HPP
#define VSMC_RNG_PHILOX_HPP

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
    class PhiloxWeylConstant<T, I> : public std::integral_constant<T, val>    \
    {                                                                         \
    }; // class PhiloxWeylConstant

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val)                   \
    template <>                                                               \
    class PhiloxRoundConstant<T, K, I>                                        \
        : public std::integral_constant<T, val>                               \
    {                                                                         \
    }; // PhiloxRoundConstant

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

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 1)>
class PhiloxBumpKey
{
    public:
    static void eval(std::array<ResultType, K / 2> &) {}
}; // class PhiloxBumpKey

template <typename ResultType, std::size_t N>
class PhiloxBumpKey<ResultType, 2, N, true>
{
    public:
    static void eval(std::array<ResultType, 1> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<ResultType, 0>::value;
    }
}; // class PhiloxBumpKey

template <typename ResultType, std::size_t N>
class PhiloxBumpKey<ResultType, 4, N, true>
{
    public:
    static void eval(std::array<ResultType, 2> &par)
    {
        std::get<0>(par) += PhiloxWeylConstant<ResultType, 0>::value;
        std::get<1>(par) += PhiloxWeylConstant<ResultType, 1>::value;
    }
}; // class PhiloxBumpKey

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
class PhiloxRound
{
    public:
    static void eval(
        std::array<ResultType, K> &, const std::array<ResultType, K / 2> &)
    {
    }
}; // class PhiloxRound

template <typename ResultType, std::size_t N>
class PhiloxRound<ResultType, 2, N, true>
{
    public:
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
}; // class PhiloxRound

template <typename ResultType, std::size_t N>
class PhiloxRound<ResultType, 4, N, true>
{
    public:
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
}; // class PhiloxRound

} // namespace vsmc::internal

/// \brief Philox RNG engine reimplemented
/// \ingroup Philox
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
class PhiloxEngine
{
    public:
    using result_type = ResultType;
    using key_type = std::array<ResultType, K / 2>;
    using ctr_type = std::array<ResultType, K>;

    explicit PhiloxEngine(result_type s = 0) : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(s);
    }

    template <typename SeedSeq>
    explicit PhiloxEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, PhiloxEngine<ResultType, K, Rounds>>::value>::type * =
            nullptr)
        : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(seq);
    }

    PhiloxEngine(const key_type &k) : index_(K)
    {
        VSMC_STATIC_ASSERT_RNG_PHILOX;
        seed(k);
    }

    void seed(result_type s)
    {
        ctr_.fill(0);
        key_.fill(0);
        key_.front() = s;
        index_ = K;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, PhiloxEngine<ResultType, K, Rounds>>::value>::type * =
            nullptr)
    {
        ctr_.fill(0);
        seq.generate(key_.begin(), key_.end());
        index_ = K;
    }

    void seed(const key_type &k)
    {
        ctr_.fill(0);
        key_ = k;
        index_ = K;
    }

    ctr_type ctr() const { return ctr_; }

    key_type key() const { return key_; }

    void ctr(const ctr_type &c)
    {
        ctr_ = c;
        index_ = K;
    }

    void key(const key_type &k)
    {
        key_ = k;
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

    friend bool operator==(const PhiloxEngine<ResultType, K, Rounds> &eng1,
        const PhiloxEngine<ResultType, K, Rounds> &eng2)
    {
        return eng1.index_ == eng2.index_ && eng1.ctr_ == eng2.ctr_ &&
            eng1.key_ == eng2.key_;
    }

    friend bool operator!=(const PhiloxEngine<ResultType, K, Rounds> &eng1,
        const PhiloxEngine<ResultType, K, Rounds> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const PhiloxEngine<ResultType, K, Rounds> &eng)
    {
        if (!os.good())
            return os;

        os << eng.buffer_ << ' ';
        os << eng.key_ << ' ';
        os << eng.ctr_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        PhiloxEngine<ResultType, K, Rounds> &eng)
    {
        if (!is.good())
            return is;

        PhiloxEngine<ResultType, K, Rounds> eng_tmp;
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.key_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    std::array<ResultType, K> buffer_;
    key_type key_;
    ctr_type ctr_;
    std::size_t index_;

    void generate_buffer()
    {
        internal::increment(ctr_);
        buffer_ = ctr_;
        key_type par = key_;
        generate_buffer<0>(par, std::true_type());
    }

    template <std::size_t>
    void generate_buffer(key_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(key_type &par, std::true_type)
    {
        internal::PhiloxBumpKey<ResultType, K, N>::eval(par);
        internal::PhiloxRound<ResultType, K, N>::eval(buffer_, par);
        generate_buffer<N + 1>(
            par, std::integral_constant < bool, N<Rounds>());
    }
}; // class PhiloxEngine

/// \brief Philox2x32 RNG engine reimplemented
/// \ingroup Philox
using Philox2x32 = PhiloxEngine<std::uint32_t, 2>;

/// \brief Philox4x32 RNG engine reimplemented
/// \ingroup Philox
using Philox4x32 = PhiloxEngine<std::uint32_t, 4>;

/// \brief Philox2x64 RNG engine reimplemented
/// \ingroup Philox
using Philox2x64 = PhiloxEngine<std::uint64_t, 2>;

/// \brief Philox4x64 RNG engine reimplemented
/// \ingroup Philox
using Philox4x64 = PhiloxEngine<std::uint64_t, 4>;

/// \brief The default 32-bits Philox engine
/// \ingroup Philox
using Philox = Philox4x32;

/// \brief The default 64-bits Philox engine
/// \ingroup Philox
using Philox_64 = Philox4x64;

} // namespace vsmc

#endif // VSMC_RNG_PHILOX_HPP
