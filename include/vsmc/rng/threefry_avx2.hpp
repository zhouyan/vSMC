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

template <typename ResultType, std::size_t K>
struct ThreefryParPackAVX2 {
    typedef std::array<ResultType, K + 1> par_type;
    typedef std::array<internal::M256I<ResultType>, K + 1> par256_type;

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
        std::get<N>(par256) = std::get<N>(par);
        pack<N + 1>(
            par, par256, std::integral_constant<bool, N + 1 < K + 1>());
    }
}; // struct ThreefryParPackAVX2

template <typename, std::size_t>
struct ThreefryCtrPackAVX2;

template <std::size_t K>
struct ThreefryCtrPackAVX2<std::uint32_t, K> {
    typedef std::array<internal::M256I<std::uint32_t>, K> state_type;
    typedef std::array<std::uint32_t, K> ctr_type;

    static void eval(ctr_type &ctr, state_type &state)
    {
        std::array<ctr_type, 8> ctr_blocks;
        Counter<ctr_type>::increment(ctr, ctr_blocks);
        pack<0>(ctr_blocks, state, std::integral_constant<bool, 0 < K>());
    }

    private:
    template <std::size_t N>
    static void pack(
        const std::array<ctr_type, 8> &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    static void pack(
        std::array<ctr_type, 8> &ctr_blocks, state_type &state, std::true_type)
    {
        std::get<N>(state) = _mm256_set_epi32(
            static_cast<int>(std::get<N>(std::get<0>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<1>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<2>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<3>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<4>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<5>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<6>(ctr_blocks))),
            static_cast<int>(std::get<N>(std::get<7>(ctr_blocks))));
        pack<N + 1>(
            ctr_blocks, state, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryCtrPackAVX2

template <std::size_t K>
struct ThreefryCtrPackAVX2<std::uint64_t, K> {
    typedef std::array<internal::M256I<std::uint64_t>, K> state_type;
    typedef std::array<std::uint64_t, K> ctr_type;

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
    static void pack(const std::array<ctr_type, 4> &ctr_blocks,
        state_type &state, std::true_type)
    {
        std::get<N>(state) = _mm256_set_epi64x(
            static_cast<VSMC_INT64>(std::get<N>(std::get<0>(ctr_blocks))),
            static_cast<VSMC_INT64>(std::get<N>(std::get<1>(ctr_blocks))),
            static_cast<VSMC_INT64>(std::get<N>(std::get<2>(ctr_blocks))),
            static_cast<VSMC_INT64>(std::get<N>(std::get<3>(ctr_blocks))));
        pack<N + 1>(
            ctr_blocks, state, std::integral_constant<bool, N + 1 < K>());
    }
}; // struct ThreefryCtrPackAVX2

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
    typedef std::array<internal::M256I<ResultType>, K> state_type;
    typedef std::array<internal::M256I<ResultType>, K + 1> par_type;

    static constexpr std::size_t M_ = internal::M256I<ResultType>::size();

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
        std::memcpy(buffer_.data(), state.data(), sizeof(ResultType) * K * M_);
    }

    template <std::size_t>
    void generate_buffer(const par_type &, state_type &, std::false_type)
    {
    }

    template <std::size_t N>
    void generate_buffer(
        const par_type &par, state_type &state, std::true_type)
    {
        internal::ThreefryRotate<internal::M256I<ResultType>, K, N>::eval(
            state);
        internal::ThreefryInsertKey<internal::M256I<ResultType>, K, N>::eval(
            state, par);
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
