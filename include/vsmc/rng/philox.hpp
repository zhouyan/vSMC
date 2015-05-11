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
#include <vsmc/rng/internal/philox_defines.hpp>

namespace vsmc
{

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
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_PHILOX_ROUNDS>
class PhiloxEngine
{
    public:
    typedef ResultType result_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K / 2> key_type;

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
    std::array<ResultType, K> buffer_;
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
