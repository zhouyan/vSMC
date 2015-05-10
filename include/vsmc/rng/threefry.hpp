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
#include <vsmc/rng/internal/threefry_defines.hpp>

namespace vsmc
{

namespace internal
{

template <typename ResultType, ResultType R>
struct ThreefryRotateImpl;

template <std::uint32_t R>
struct ThreefryRotateImpl<std::uint32_t, R> {
    static std::uint32_t eval(std::uint32_t x)
    {
        return x << R | x >> (32 - R);
    }
};

template <std::uint64_t R>
struct ThreefryRotateImpl<std::uint64_t, R> {
    static std::uint64_t eval(std::uint64_t x)
    {
        return x << R | x >> (64 - R);
    }
};

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
                ThreefryRotateConstantValue<ResultType, 2, r_,
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
                ThreefryRotateConstantValue<ResultType, 4, r_,
                                   0>::value>::eval(std::get<i0_>(state));
        std::get<i0_>(state) ^= std::get<0>(state);

        std::get<2>(state) += std::get<i2_>(state);
        std::get<i2_>(state) =
            ThreefryRotateImpl<ResultType,
                ThreefryRotateConstantValue<ResultType, 4, r_,
                                   1>::value>::eval(std::get<i2_>(state));
        std::get<i2_>(state) ^= std::get<2>(state);
    }

    private:
    static constexpr std::size_t i0_ = N % 2 ? 1 : 3;
    static constexpr std::size_t i2_ = N % 2 ? 3 : 1;
    static constexpr std::size_t r_ = (N - 1) % 8;
}; // struct ThreefryRotate

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
        std::get<1>(state) += static_cast<ResultType>(inc_);
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
        std::get<3>(state) += static_cast<ResultType>(inc_);
    }

    private:
    static constexpr std::size_t inc_ = N / 4;
    static constexpr std::size_t i0_ = (inc_ + 0) % 5;
    static constexpr std::size_t i1_ = (inc_ + 1) % 5;
    static constexpr std::size_t i2_ = (inc_ + 2) % 5;
    static constexpr std::size_t i3_ = (inc_ + 3) % 5;
}; // struct ThreefryInsertKey

} // namespace vsmc::internal

/// \brief Threefry RNG engine reimplemented
/// \ingroup Threefry
///
/// \details
/// This is a reimplementation of the algorithm Threefry as described in
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
/// Compared to `r123:Engine<r123::Threefry4x32>` etc., when using the default
/// constructor or the one with a single seed, the output shall be exactly the
/// same for the first \f$2^n\f$ iterations, where \f$n\f$ is the number of
/// bits (32 or 64).  Further iterations may produce different results, as
/// vSMC increment the counter slightly differently, but it still cover the
/// same range and has the same period as the original.
template <typename ResultType, std::size_t K,
    std::size_t Rounds = VSMC_RNG_THREEFRY_ROUNDS>
class ThreefryEngine
{
    public:
    typedef ResultType result_type;
    typedef std::array<ResultType, K> buffer_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ResultType, K> key_type;

    private:
    typedef Counter<ctr_type> counter;

    public:
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
        counter::reset(ctr_);
        key_type k;
        k.fill(0);
        k.front() = s;
        init_par(k);
        index_ = K;
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, ThreefryEngine<ResultType, K, Rounds>>::value>::type * =
            nullptr)
    {
        counter::reset(ctr_);
        key_type k;
        seq.generate(k.begin(), k.end());
        init_par(k);
        index_ = K;
    }

    void seed(const key_type &k)
    {
        counter::reset(ctr_);
        init_par(k);
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
        counter::set(ctr_, c);
        index_ = K;
    }

    void key(const key_type &k)
    {
        init_par(k);
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

        os << eng.ctr_ << ' ';
        os << eng.par_ << ' ';
        os << eng.buffer_ << ' ';
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
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.par_;
        is >> std::ws >> eng_tmp.buffer_;
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

    void generate_buffer(const ctr_type &c, buffer_type &buf) const
    {
        buf = c;
        generate_buffer<0>(buf, std::true_type());
    }

    template <std::size_t>
    void generate_buffer(buffer_type &, std::false_type) const
    {
    }

    template <std::size_t N>
    void generate_buffer(buffer_type &buf, std::true_type) const
    {
        internal::ThreefryRotate<ResultType, K, N>::eval(buf);
        internal::ThreefryInsertKey<ResultType, K, N>::eval(buf, par_);
        generate_buffer<N + 1>(
            buf, std::integral_constant < bool, N<Rounds>());
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
}; // class ThreefryEngine

/// \brief Threefry2x32 RNG engine reimplemented
/// \ingroup Threefry
typedef ThreefryEngine<std::uint32_t, 2> Threefry2x32;

/// \brief Threefry4x32 RNG engine reimplemented
/// \ingroup Threefry
typedef ThreefryEngine<std::uint32_t, 4> Threefry4x32;

/// \brief Threefry2x64 RNG engine reimplemented
/// \ingroup Threefry
typedef ThreefryEngine<std::uint64_t, 2> Threefry2x64;

/// \brief Threefry4x64 RNG engine reimplemented
/// \ingroup Threefry
typedef ThreefryEngine<std::uint64_t, 4> Threefry4x64;

/// \brief The default 32-bits Threefry engine
/// \ingroup Threefry
typedef Threefry4x32 Threefry;

/// \brief The default 64-bits Threefry engine
/// \ingroup Threefry
typedef Threefry4x64 Threefry_64;

} // namespace vsmc

#endif // VSMC_RNG_THREEFRY_HPP
