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
    struct PhiloxWeylConstantValue<T, I>                                      \
        : public std::integral_constant<T, val> {                             \
    };

#define VSMC_DEFINE_RNG_PHILOX_ROUND_CONSTANT(T, K, I, val)                   \
    template <>                                                               \
    struct PhiloxRoundConstantValue<T, K, I>                                  \
        : public std::integral_constant<T, val> {                             \
    };

/// \brief PhiloxEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_PHILOX_ROUNDS
#define VSMC_RNG_PHILOX_ROUNDS 10
#endif

namespace vsmc
{

namespace traits
{

namespace internal
{

template <typename, std::size_t>
struct PhiloxWeylConstantValue;

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 0, UINT32_C(0x9E3779B9))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(std::uint32_t, 1, UINT32_C(0xBB67AE85))

VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 0, UINT64_C(0x9E3779B97F4A7C15))
VSMC_DEFINE_RNG_PHILOX_WELY_CONSTANT(
    std::uint64_t, 1, UINT64_C(0xBB67AE8584CAA73B))

template <typename, std::size_t, std::size_t>
struct PhiloxRoundConstantValue;

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

} // namespace vsmc::traits::internal

/// \brief Traits of PhiloxEngine constants for bumping the key (Weyl
/// sequence)
/// \ingroup Traits
///
/// \details
/// The first template argument is either `std::uint32_t` or `std::uint64_t`.
/// The second
/// is either 0 or 1. Specializing the class templates
/// `PhiloxWeylConstantTrait<std::uint64_t, 0>` etc., are equivalent to define
/// macros `PHILOX_W64_0` etc., in the original implementation.
template <typename ResultType, std::size_t I>
struct PhiloxWeylConstantTrait
    : public internal::PhiloxWeylConstantValue<ResultType, I> {
};

/// \brief Traits of PhiloxEngine constants for rounding
/// \ingroup Traits
///
/// \details
/// The first template argument is either `std::uint32_t` or `std::uint64_t`.
/// The second
/// is the size of the RNG, either 2 or 4. The third is either 0 or 1.
/// Specializing the class templates `PhiloxRoundConstantTrait<std::uint64_t,
/// 4,
/// 0>`
/// etc., are equivalent to define macros `PHILOX_M4x64_0` etc., in the
/// original implementation.
template <typename ResultType, std::size_t K, std::size_t I>
struct PhiloxRoundConstantTrait
    : public internal::PhiloxRoundConstantValue<ResultType, K, I> {
};

} // namespace vsmc::traits

namespace internal
{

template <typename ResultType, std::size_t K, std::size_t N, bool = (N > 1)>
struct PhiloxBumpKey {
    static void eval(std::array<ResultType, K / 2> &) {}
};

template <typename ResultType, std::size_t N>
struct PhiloxBumpKey<ResultType, 2, N, true> {
    static void eval(std::array<ResultType, 1> &par)
    {
        std::get<0>(par) +=
            traits::PhiloxWeylConstantTrait<ResultType, 0>::value;
    }
}; // struct PhiloxBumpKey

template <typename ResultType, std::size_t N>
struct PhiloxBumpKey<ResultType, 4, N, true> {
    static void eval(std::array<ResultType, 2> &par)
    {
        std::get<0>(par) +=
            traits::PhiloxWeylConstantTrait<ResultType, 0>::value;
        std::get<1>(par) +=
            traits::PhiloxWeylConstantTrait<ResultType, 1>::value;
    }
}; // struct PhiloxBumpKey

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint32_t b, std::uint32_t &hi, std::uint32_t &lo)
{
    std::uint64_t prod =
        static_cast<std::uint64_t>(b) *
        static_cast<std::uint64_t>(
            traits::PhiloxRoundConstantTrait<std::uint32_t, K, I>::value);
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
            traits::PhiloxRoundConstantTrait<std::uint64_t, K, I>::value);
    hi = static_cast<std::uint64_t>(prod >> 64);
    lo = static_cast<std::uint64_t>(prod);
}

#elif defined(VSMC_MSVC) // VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint64_t b, std::uint64_t &hi, std::uint64_t &lo)
{
    lo = _umul128(
        traits::PhiloxRoundConstantTrait<std::uint64_t, K, I>::value, b, &hi);
}

#else // VSMC_HAS_INT128

template <std::size_t K, std::size_t I>
inline void philox_hilo(std::uint64_t b, std::uint64_t &hi, std::uint64_t &lo)
{
    const std::uint64_t a =
        traits::PhiloxRoundConstantTrait<std::uint64_t, K, I>::value;
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
        std::get<0>(state) = hi ^ (std::get<0>(par) ^ std::get<1>(state));
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

/// \brief Philox RNG engine reimplemented
/// \ingroup Philox
///
/// \details
/// This is a reimplementation of the algorithm Philox as described in
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
/// Currently the 64-bits version is much slower than the original, except
/// when using recent Clang, GCC, Intel C++ or MSVC on x86-64 computers. The
/// original implementation use some platform dependent assembly or intrinsics
/// to optimize the performance. This implementation use standard C99 when
/// used on other platforms.
///
/// This implementation is slightly more flexible in the sense that it does
/// not limit the number of rounds. However, larger number of rounds can have
/// undesired effects. To say the least, currently all loops are unrolled,
/// which can slow down significantly when the number of rounds is large.
///
/// Compared to `r123:Engine<r123::Philox4x32>` etc., when using the default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^n\f$ iterations, where \f$n\f$ is the number of
/// bits (32 or 64).  Further iterations may produce different results, as
/// vSMC increment the counter slightly differently, but it still cover the
/// same range and has the same period as the original.
///
/// The constants of bumping the key (Weyl constants) and those used in each
/// rounds can be set through traits, `vsmc::traits::PhiloxWeylConstantTrait`
/// and `vsmc::traits::PhiloxRoundConstantTrait`.
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
class PhiloxEngine
{
    public:
    typedef ResultType result_type;
    typedef std::array<ResultType, K> buffer_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K / 2> key_type;

    private:
    typedef Counter<ctr_type> counter;

    public:
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
        counter::reset(ctr_);
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
        counter::reset(ctr_);
        seq.generate(key_.begin(), key_.end());
        index_ = K;
    }

    void seed(const key_type &k)
    {
        counter::reset(ctr_);
        key_ = k;
        index_ = K;
    }

    ctr_type ctr() const { return ctr_; }

    key_type key() const { return key_; }

    void ctr(const ctr_type &c)
    {
        counter::set(ctr_, c);
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
            counter::increment(ctr_);
            generate_buffer(ctr_, buffer_);
            index_ = 0;
        }

        return buffer_[index_++];
    }

    /// \brief Generate a buffer of random bits given a counter using the
    /// current key
    buffer_type operator()(const ctr_type &c) const
    {
        buffer_type buf;
        generate_buffer(c, buf);

        return buf;
    }

    /// \brief Generate random bits in a pre-allocated buffer given a counter
    /// using the current key
    void operator()(const ctr_type &c, buffer_type &buf) const
    {
        generate_buffer(c, buf);
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

        counter::increment(ctr_, static_cast<result_type>(n / K));
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

        os << eng.ctr_ << ' ';
        os << eng.key_ << ' ';
        os << eng.buffer_ << ' ';
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
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.key_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good())
            eng = std::move(eng_tmp);

        return is;
    }

    private:
    ctr_type ctr_;
    key_type key_;
    buffer_type buffer_;
    std::size_t index_;

    void generate_buffer(const ctr_type c, buffer_type &buf) const
    {
        buf = c;
        key_type par = key_;
        generate_buffer<0>(buf, par, std::true_type());
    }

    template <std::size_t>
    void generate_buffer(buffer_type &, key_type &, std::false_type) const
    {
    }

    template <std::size_t N>
    void generate_buffer(buffer_type &buf, key_type &par, std::true_type) const
    {
        internal::PhiloxBumpKey<ResultType, K, N>::eval(par);
        internal::PhiloxRound<ResultType, K, N>::eval(buf, par);
        generate_buffer<N + 1>(
            buf, par, std::integral_constant < bool, N<Rounds>());
    }
}; // class PhiloxEngine

/// \brief Philox2x32 RNG engine reimplemented
/// \ingroup Philox
typedef PhiloxEngine<std::uint32_t, 2> Philox2x32;

/// \brief Philox4x32 RNG engine reimplemented
/// \ingroup Philox
typedef PhiloxEngine<std::uint32_t, 4> Philox4x32;

/// \brief Philox2x64 RNG engine reimplemented
/// \ingroup Philox
typedef PhiloxEngine<std::uint64_t, 2> Philox2x64;

/// \brief Philox4x64 RNG engine reimplemented
/// \ingroup Philox
typedef PhiloxEngine<std::uint64_t, 4> Philox4x64;

/// \brief The default 32-bits Philox engine
/// \ingroup Philox
typedef Philox4x32 Philox;

/// \brief The default 64-bits Philox engine
/// \ingroup Philox
typedef Philox4x64 Philox_64;

} // namespace vsmc

#endif // VSMC_RNG_PHILOX_HPP
