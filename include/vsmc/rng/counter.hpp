//============================================================================
// vSMC/include/vsmc/rng/counter.hpp
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

#ifndef VSMC_RNG_COUNTER_HPP
#define VSMC_RNG_COUNTER_HPP

#include <vsmc/rng/internal/common.hpp>

namespace vsmc
{

namespace internal
{

template <std::size_t, typename T, std::size_t K>
inline void increment_single(std::array<T, K> &, std::false_type)
{
}

template <std::size_t N, typename T, std::size_t K>
inline void increment_single(std::array<T, K> &ctr, std::true_type)
{
    if (++std::get<N>(ctr) != 0)
        return;

    increment_single<N + 1>(ctr, std::integral_constant<bool, N + 1 < K>());
}

template <typename T, std::size_t K>
inline void increment(std::array<T, K> &ctr)
{
    increment_single<0>(ctr, std::true_type());
}

template <typename T, std::size_t K, T NSkip>
inline void increment(std::array<T, K> &ctr, std::integral_constant<T, NSkip>)
{
    if (ctr.front() < std::numeric_limits<T>::max() - NSkip)
        ctr.front() += NSkip;
    else
        increment_single<0>(ctr, std::true_type());
}

template <typename T, std::size_t K>
inline void increment(std::array<T, K> &ctr, T nskip)
{
    if (ctr.front() < std::numeric_limits<T>::max() - nskip)
        ctr.front() += nskip;
    else
        increment_single<0>(ctr, std::true_type());
}

template <std::size_t, typename T, std::size_t K, std::size_t Blocks>
inline void increment_block(std::array<T, K> &,
    std::array<std::array<T, K>, Blocks> &, std::false_type)
{
}

template <std::size_t B, typename T, std::size_t K, std::size_t Blocks>
inline void increment_block(std::array<T, K> &ctr,
    std::array<std::array<T, K>, Blocks> &ctr_block, std::true_type)
{
    std::get<B>(ctr_block) = ctr;
    increment(std::get<B>(ctr_block), std::integral_constant<T, B + 1>());
    increment_block<B + 1>(
        ctr, ctr_block, std::integral_constant<bool, B + 1 < Blocks>());
}

template <std::size_t, typename T, std::size_t K, std::size_t Blocks>
inline void increment_block_safe(std::array<T, K> &,
    std::array<std::array<T, K>, Blocks> &, std::false_type)
{
}

template <std::size_t B, typename T, std::size_t K, std::size_t Blocks>
inline void increment_block_safe(std::array<T, K> &ctr,
    std::array<std::array<T, K>, Blocks> &ctr_block, std::true_type)
{
    std::get<B>(ctr_block) = ctr;
    ++std::get<B>(ctr_block).front();
    increment_block<B + 1>(
        ctr, ctr_block, std::integral_constant<bool, B + 1 < Blocks>());
}

template <typename T, std::size_t K, std::size_t Blocks>
inline void increment(
    std::array<T, K> &ctr, std::array<std::array<T, K>, Blocks> &ctr_block)
{
    const T limit = std::numeric_limits<T>::max() - static_cast<T>(Blocks);
    if (ctr.front() < limit)
        increment_block_safe<0>(ctr, ctr_block, std::true_type());
    else
        increment_block<0>(ctr, ctr_block, std::true_type());
    ctr = ctr_block.back();
}

template <std::size_t, typename T, std::size_t K, std::size_t B>
inline void increment_safe_set(const std::array<T, K> &,
    std::array<std::array<T, K>, B> &, std::false_type)
{
}

template <std::size_t N, typename T, std::size_t K, std::size_t B>
inline void increment_safe_set(const std::array<T, K> &ctr,
    std::array<std::array<T, K>, B> &cb, std::true_type)
{
    std::get<N>(cb) = ctr;
    increment_safe_set<N + 1>(
        ctr, cb, std::integral_constant<bool, N + 1 < B>());
}

template <typename T, std::size_t K>
inline void increment_safe_set(std::size_t n, const std::array<T, K> &ctr,
    std::array<T, K> *ctr_block, std::false_type)
{
    for (std::size_t i = 0; i != n; ++i)
        ctr_block[i] = ctr;
}

#if VSMC_HAS_AVX2

template <typename T, std::size_t K>
inline void increment_safe_set(std::size_t n, const std::array<T, K> &ctr,
    std::array<T, K> *ctr_block, std::true_type)
{
    const std::size_t B = M256I<T>::size() / K;
    const std::size_t m = n / B;
    const std::size_t l = n % B;
    M256I<> c;
    std::array<std::array<T, K>, B> cb;
    increment_safe_set<0>(ctr, cb, std::integral_constant<bool, 0 < B>());
    c.load(cb.data());
    if (reinterpret_cast<std::uintptr_t>(ctr_block) % 32 == 0) {
        for (std::size_t i = 0; i != m; ++i, ctr_block += B)
            c.store_a(ctr_block);
    } else {
        for (std::size_t i = 0; i != m; ++i, ctr_block += B)
            c.store_u(ctr_block);
    }
    for (std::size_t i = 0; i != l; ++i)
        ctr_block[i] = ctr;
}

template <typename T, std::size_t K>
inline void increment_safe_set(
    std::size_t n, const std::array<T, K> &ctr, std::array<T, K> *ctr_block)
{
    increment_safe_set(n, ctr, ctr_block,
        std::integral_constant<bool, M256I<T>::size() % K == 0>());
}

#elif VSMC_HAS_SSE2

template <typename T, std::size_t K>
inline void increment_safe_set(std::size_t n, const std::array<T, K> &ctr,
    std::array<T, K> *ctr_block, std::true_type)
{
    const std::size_t B = M128I<T>::size() / K;
    const std::size_t m = n / B;
    const std::size_t l = n % B;
    M128I<> c;
    std::array<std::array<T, K>, B> cb;
    increment_safe_set<0>(ctr, cb, std::integral_constant<bool, 0 < B>());
    c.load(cb.data());
    if (reinterpret_cast<std::uintptr_t>(ctr_block) % 32 == 0) {
        for (std::size_t i = 0; i != m; ++i, ctr_block += B)
            c.store_a(ctr_block);
    } else {
        for (std::size_t i = 0; i != m; ++i, ctr_block += B)
            c.store_u(ctr_block);
    }
    for (std::size_t i = 0; i != l; ++i)
        ctr_block[i] = ctr;
}

template <typename T, std::size_t K>
inline void increment_safe_set(
    std::size_t n, const std::array<T, K> &ctr, std::array<T, K> *ctr_block)
{
    increment_safe_set(n, ctr, ctr_block,
        std::integral_constant<bool, M128I<T>::size() % K == 0>());
}

#else // VSMC_HAS_AVX2

template <typename T, std::size_t K>
inline void increment_safe_set(
    std::size_t n, const std::array<T, K> &ctr, std::array<T, K> *ctr_block)
{
    increment_safe_set(n, ctr, ctr_block, std::false_type());
}

#endif // VSMC_HAS_AVX2

template <typename T, std::size_t K>
inline void increment_safe(
    std::size_t n, std::array<T, K> &ctr, std::array<T, K> *ctr_block)
{
    increment(ctr);
    increment_safe_set(n, ctr, ctr_block);
    T *c = ctr_block[0].data();
    for (std::size_t i = 0; i != n; ++i, c += K)
        *c += i;
    ctr = ctr_block[n - 1];
}

template <typename T, std::size_t K>
inline void increment(
    std::size_t n, std::array<T, K> &ctr, std::array<T, K> *ctr_block)
{
    if (n == 0)
        return;

    const std::uint64_t m =
        static_cast<std::uint64_t>(std::numeric_limits<T>::max());
    const std::uint64_t l = static_cast<std::uint64_t>(ctr.front());
    const std::uint64_t k = static_cast<std::uint64_t>(n);
    if (k < m && l < m - k) {
        increment_safe(n, ctr, ctr_block);
    } else {
        for (std::size_t i = 0; i != n; ++i) {
            increment(ctr);
            ctr_block[i] = ctr;
        }
    }
}

} // namespace vsmc::internal

/// \brief Counter based RNG engine
/// \ingroup RNG
template <typename Generator>
class CounterEngine
{
    public:
    using result_type = typename Generator::result_type;
    using ctr_type = typename Generator::ctr_type;
    using key_type = typename Generator::key_type;

    explicit CounterEngine(result_type s = 0) : index_(M_) { seed(s); }

    template <typename SeedSeq>
    explicit CounterEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, result_type,
            key_type, CounterEngine<Generator>>::value>::type * = nullptr)
        : index_(M_)
    {
        seed(seq);
    }

    explicit CounterEngine(const key_type &k) : index_(M_) { seed(k); }

    void seed(result_type s)
    {
        key_.fill(0);
        key_.front() = s;
        reset();
    }

    template <typename SeedSeq>
    void seed(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          result_type, key_type>::value>::type * = nullptr)
    {
        seq.generator(key_.begin(), key_.end());
        reset();
    }

    void seed(const key_type &k)
    {
        key_ = k;
        reset();
    }

    const ctr_type &ctr() const { return ctr_; }

    const key_type &key() const { return key_; }

    void ctr(const ctr_type &c)
    {
        ctr_ = c;
        index_ = M_;
    }

    void key(const key_type &k)
    {
        key_ = k;
        reset();
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generator_(ctr_, key_, buffer_);
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void operator()(std::size_t n, result_type *r)
    {
        if (n * sizeof(result_type) <= 32) {
            for (std::size_t i = 0; i != n; ++i)
                r[i] = operator()();
            return;
        }

        std::size_t p = 32 -
            static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(r) % 32);
        if (p % sizeof(result_type) == 0) {
            p /= sizeof(result_type);
            for (std::size_t i = 0; i != p; ++i)
                r[i] = operator()();
            n -= p;
            r += p;
        }

        const std::size_t q = generator_(ctr_, key_, n, r);
        n -= q;
        r += q;

        const std::size_t m = n / M_;
        std::array<result_type, M_> *s =
            reinterpret_cast<std::array<result_type, M_> *>(r);
        for (std::size_t i = 0; i != m; ++i)
            generator_(ctr_, key_, s[i]);
        n -= m * M_;
        r += m * M_;

        for (std::size_t i = 0; i != n; ++i)
            r[i] = operator()();
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

    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }

    friend bool operator==(const CounterEngine<Generator> &eng1,
        const CounterEngine<Generator> &eng2)
    {
        if (eng1.buffer_ != eng2.buffer_)
            return false;
        if (eng1.ctr_ != eng2.ctr_)
            return false;
        if (eng1.key_ != eng2.key_)
            return false;
        if (eng1.index_ != eng2.index_)
            return false;
        return true;
    }

    friend bool operator!=(const CounterEngine<Generator> &eng1,
        const CounterEngine<Generator> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const CounterEngine<Generator> &eng)
    {
        if (!os.good())
            return os;

        os << eng.buffer_ << ' ';
        os << eng.ctr_ << ' ';
        os << eng.key_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, CounterEngine<Generator> &eng)
    {
        if (!is.good())
            return is;

        CounterEngine<Generator> eng_tmp;
        is >> std::ws >> eng_tmp.buffer_;
        is >> std::ws >> eng_tmp.ctr_;
        is >> std::ws >> eng_tmp.key_;
        is >> std::ws >> eng_tmp.index_;

        if (is.good()) {
            eng_tmp.generator_.reset(eng_tmp.key_);
            eng = std::move(eng_tmp);
        }

        return is;
    }

    private:
    static constexpr std::size_t M_ = Generator::size();

    alignas(32) std::array<result_type, M_> buffer_;
    ctr_type ctr_;
    key_type key_;
    Generator generator_;
    std::size_t index_;

    void reset()
    {
        ctr_.fill(0);
        generator_.reset(key_);
        index_ = M_;
    }
}; // class CounterEngine

template <typename Generator>
inline void rng_rand(CounterEngine<Generator> &rng, std::size_t n,
    typename CounterEngine<Generator>::result_type *r)
{
    rng(n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_COUNTER_HPP
