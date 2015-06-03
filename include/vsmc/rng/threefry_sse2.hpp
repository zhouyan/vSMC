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
#include <vsmc/rng/internal/threefry_defines.hpp>
#include <vsmc/rng/m128i.hpp>

namespace vsmc
{

namespace internal
{

template <typename ResultType, std::size_t K>
class ThreefryParPackSSE2
{
    public:
    typedef std::array<ResultType, K + 1> par_type;
    typedef std::array<M128I<ResultType>, K + 1> par128_type;

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
    typedef std::array<M128I<ResultType>, K> state_type;
    typedef std::array<ResultType, K> ctr_type;
    typedef std::array<ctr_type, M128I<ResultType>::size()> ctr_block_type;

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
    typedef ResultType result_type;
    typedef std::array<ResultType, K> key_type;
    typedef std::array<ResultType, K> ctr_type;

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
    typedef std::array<M128I<ResultType>, K + 1> par_type;
    typedef std::array<M128I<ResultType>, K> state_type;

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
