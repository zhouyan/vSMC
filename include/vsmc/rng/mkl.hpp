//============================================================================
// vSMC/include/vsmc/rng/mkl.hpp
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

#ifndef VSMC_RNG_MKL_HPP
#define VSMC_RNG_MKL_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/utility/mkl.hpp>

namespace vsmc
{

namespace internal
{

class MKLOffsetZero
{
    public:
    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return 0; }
    static void set(MKL_INT) {}
    static constexpr MKL_INT get() { return 0; }
}; // class OffsetZero

template <MKL_INT MaxOffset>
class MKLOffsetDynamic
{
    public:
    MKLOffsetDynamic() : offset_(0) {}

    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return MaxOffset; }

    void set(MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_MKL_VSL_OFFSET(n);
        offset_ = n % MaxOffset;
    }

    MKL_INT get() const { return offset_; }

    private:
    MKL_INT offset_;
}; // class OffsetDynamic

template <MKL_INT>
class MKLOffset
{
    public:
    using type = MKLOffsetZero;
}; // class MKLOffset

template <>
class MKLOffset<Dynamic>
{
    public:
    using type =
        MKLOffsetDynamic<std::numeric_limits<MKL_INT>::max VSMC_MNE()>;
}; // class MKLOffset

template <>
class MKLOffset<VSL_BRNG_MT2203>
{
    public:
    using type = MKLOffsetDynamic<6024>;
}; // class MKLOffset

template <>
class MKLOffset<VSL_BRNG_WH>
{
    public:
    using type = MKLOffsetDynamic<273>;
}; // class MKLOffset

template <std::size_t>
class MKLResultTypeTrait;

template <>
class MKLResultTypeTrait<32>
{
    public:
    using type = unsigned;
}; // class MKLResultTypeTrait

template <>
class MKLResultTypeTrait<64>
{
    public:
    using type = unsigned MKL_INT64;
}; // class MKLResultTypeTrait

template <std::size_t Bits>
using MKLResultType = typename MKLResultTypeTrait<Bits>::type;

template <std::size_t>
class MKLUniformBits;

template <>
class MKLUniformBits<32>
{
    public:
    static void eval(MKLStream &stream, MKL_INT n, unsigned *r)
    {
        stream.uniform_bits32(n, r);
    }
}; // class MKLUniformBits

template <>
class MKLUniformBits<64>
{
    public:
    static void eval(MKLStream &stream, MKL_INT n, unsigned MKL_INT64 *r)
    {
        stream.uniform_bits64(n, r);
    }
}; // class MKLUniformBits

class MKLDiscardSkipAhead
{
    public:
    static void eval(MKLStream &stream, long long nskip)
    {
        stream.skip_ahead(nskip);
    }
}; // class DiscardSkipAhead

template <MKL_INT BRNG, std::size_t Bits>
class MKLDiscardGeneral
{
    public:
    static void eval(MKLStream &stream, long long nskip)
    {
        if (nskip == 0)
            return;

        std::array<MKLResultType<Bits>, 1000> buffer;
        const MKL_INT k = static_cast<MKL_INT>(buffer.size());
        while (nskip > k) {
            MKLUniformBits<Bits>::eval(stream, k, buffer.data());
            nskip -= k;
        }
        MKLUniformBits<Bits>::eval(
            stream, static_cast<MKL_INT>(nskip), buffer.data());
    }
}; // class DiscardGeneral

template <MKL_INT BRNG, std::size_t Bits>
class MKLDiscard
{
    public:
    using type = MKLDiscardGeneral<BRNG, Bits>;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_MCG59, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_MT19937, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_SFMT19937, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

} // namespace vsmc::internal

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, std::size_t Bits>
class MKLEngine
{
    public:
    using result_type = internal::MKLResultType<Bits>;

    explicit MKLEngine(MKL_UINT s = 1) : index_(M_) { seed(s); }

    template <typename SeedSeq>
    explicit MKLEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
        : index_(M_)
    {
        seed(seq);
    }

    MKLEngine(MKL_UINT s, MKL_INT offset) { seed(s, offset); }

    void seed(MKL_UINT s) { seed(s, 0); }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
    {
        MKL_UINT s;
        seq.generate(&s, &s + 1);
        seed(s, 0);
    }

    void seed(MKL_UINT s, MKL_INT offset)
    {
        typename internal::MKLOffset<BRNG>::type off;
        off.set(offset);
        stream_.reset(BRNG + off.get(), s);
        index_ = M_;
    }

    result_type operator()()
    {
        if (index_ == M_) {
            internal::MKLUniformBits<Bits>::eval(
                stream_, static_cast<MKL_INT>(M_), buffer_.data());
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void discard(long long nskip)
    {
        internal::MKLDiscard<BRNG, Bits>::eval(stream_, nskip);
        index_ = M_;
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    MKLStream &stream() { return stream_; }
    const MKLStream &stream() const { return stream_; }

    private:
    static constexpr std::size_t M_ = 1000;

    MKLStream stream_;
    std::array<result_type, M_> buffer_;
    std::size_t index_;
}; // class MKLEngine

/// \brief A 59-bits multiplicative congruential generator
/// \ingroup MKLRNG
using MKL_MCG59 = MKLEngine<VSL_BRNG_MCG59, 32>;

/// \brief A 59-bits multiplicative congruential generator (64-bits)
/// \ingroup MKLRNG
using MKL_MCG59_64 = MKLEngine<VSL_BRNG_MCG59, 64>;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT19937 = MKLEngine<VSL_BRNG_MT19937, 32>;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bits)
/// \ingroup MKLRNG
using MKL_MT19937_64 = MKLEngine<VSL_BRNG_MT19937, 64>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT2203 = MKLEngine<VSL_BRNG_MT2203, 32>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// (64-bits)
/// \ingroup MKLRNG
using MKL_MT2203_64 = MKLEngine<VSL_BRNG_MT2203, 64>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number
/// genertor
/// \ingroup MKLRNG
using MKL_SFMT19937 = MKLEngine<VSL_BRNG_SFMT19937, 32>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number
/// genertor
/// (64-bits)
/// \ingroup MKLRNG
using MKL_SFMT19937_64 = MKLEngine<VSL_BRNG_SFMT19937, 64>;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
using MKL_NONDETERM = MKLEngine<VSL_BRNG_NONDETERM, 32>;

/// \brief A non-determinstic random number generator (64-bits)
/// \ingroup MKLRNG
using MKL_NONDETERM_64 = MKLEngine<VSL_BRNG_NONDETERM, 64>;

#if INTEL_MKL_VERSION >= 110300

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_ARS5 = MKLEngine<VSL_BRNG_ARS5, 32>;

/// \brief A counter-based random number generator (64-bits)
/// \ingroup MKLRNG
using MKL_ARS5_64 = MKLEngine<VSL_BRNG_ARS5, 64>;

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_PHILOX4X32X10 = MKLEngine<VSL_BRNG_PHILOX4X32X10, 32>;

/// \brief A counter-based random number generator (64-bits)
/// \ingroup MKLRNG
using MKL_PHILOX4X32X10_64 = MKLEngine<VSL_BRNG_PHILOX4X32X10, 64>;

#endif // INTEL_MKL_VERSION >= 110300

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void rng_rand(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    typename MKLEngine<BRNG, Bits>::result_type *r)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > static_cast<std::size_t>(k)) {
        internal::MKLUniformBits<Bits>::eval(
            rng.stream(), static_cast<MKL_INT>(k), r);
        n -= static_cast<std::size_t>(k);
    }
    internal::MKLUniformBits<Bits>::eval(
        rng.stream(), static_cast<MKL_INT>(n), r);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void cauchy_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    RealType *r, RealType a, RealType b)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > static_cast<std::size_t>(k)) {
        rng.stream().cauchy(k, r, a, b);
        n -= static_cast<std::size_t>(k);
    }
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, RealType *r, RealType lambda)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().exponential(static_cast<MKL_INT>(k), r, 0, 1 / lambda);
        n -= k;
        r += k;
    }
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void laplace_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    RealType *r, RealType location, RealType scale)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().laplace(static_cast<MKL_INT>(k), r, location, scale);
        n -= k;
        r += k;
    }
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void lognormal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    RealType *r, RealType m, RealType s)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().lognormal(static_cast<MKL_INT>(k), r, m, s, 0, 1);
        n -= k;
        r += k;
    }
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void normal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    RealType *r, RealType mean, RealType stddev)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().gaussian(static_cast<MKL_INT>(k), r, mean, stddev);
        n -= k;
        r += k;
    }
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void u01_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, RealType *r)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().uniform(static_cast<MKL_INT>(k), r, 0, 1);
        n -= k;
        r += k;
    }
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, std::size_t Bits, typename RealType>
inline void uniform_real_distribution(MKLEngine<BRNG, Bits> &rng,
    std::size_t n, RealType *r, RealType a, RealType b)
{
    const std::size_t k =
        static_cast<std::size_t>(std::numeric_limits<MKL_INT>::max VSMC_MNE());
    while (n > k) {
        rng.stream().uniform(static_cast<MKL_INT>(k), r, a, b);
        n -= k;
        r += k;
    }
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

} // namespace vsmc

#endif // VSMC_RNG_MKL_HPP
