//============================================================================
// vSMC/include/vsmc/rng/mkl.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#define VSMC_RUNTIME_ASSERT_RNG_MKL_OFFSET(offset)                            \
    VSMC_RUNTIME_ASSERT((offset < max()),                                     \
        "**MKLOffsetDynamic** "                                               \
        "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS")

namespace vsmc
{

namespace internal
{

class MKLOffsetZero
{
    public:
    static constexpr MKL_INT min() { return 0; }
    static constexpr MKL_INT max() { return 0; }
    static void set(MKL_INT) {}
    static constexpr MKL_INT get() { return 0; }
}; // class OffsetZero

template <MKL_INT MaxOffset>
class MKLOffsetDynamic
{
    public:
    MKLOffsetDynamic() : offset_(0) {}

    static constexpr MKL_INT min() { return 0; }
    static constexpr MKL_INT max() { return MaxOffset; }

    void set(MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_RNG_MKL_OFFSET(n);
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
    using type = MKLOffsetDynamic<std::numeric_limits<MKL_INT>::max()>;
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

template <int>
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

template <int Bits>
using MKLResultType = typename MKLResultTypeTrait<Bits>::type;

template <int>
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

template <MKL_INT BRNG, int Bits>
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

template <MKL_INT BRNG, int Bits>
class MKLDiscard
{
    public:
    using type = MKLDiscardGeneral<BRNG, Bits>;
}; // clas MKLDiscard

template <int Bits>
class MKLDiscard<VSL_BRNG_MCG59, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <int Bits>
class MKLDiscard<VSL_BRNG_MT19937, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <int Bits>
class MKLDiscard<VSL_BRNG_SFMT19937, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

} // namespace vsmc::internal

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, int Bits>
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
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, MKL_UINT>::value>::type * = nullptr)
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

    void operator()(std::size_t n, result_type *r)
    {
        internal::MKLUniformBits<Bits>::eval(
            stream_, static_cast<MKL_INT>(n), r);
    }

    void discard(long long nskip)
    {
        internal::MKLDiscard<BRNG, Bits>::eval(stream_, nskip);
        index_ = M_;
    }

    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }

    MKLStream &stream() { return stream_; }
    const MKLStream &stream() const { return stream_; }

    friend bool operator==(
        const MKLEngine<BRNG, Bits> &eng1, const MKLEngine<BRNG, Bits> &eng2)
    {
        if (eng1.stream_.get_brng() != eng2.stream_.get_brng())
            return false;
        std::size_t n = static_cast<std::size_t>(eng1.stream_.get_size());
        Vector<char> s1(n);
        Vector<char> s2(n);
        eng1.stream_.save_m(s1.data());
        eng2.stream_.save_m(s2.data());
        if (s1 != s2)
            return false;
        if (eng1.buffer_ != eng2.buffer_)
            return false;
        if (eng1.index_ != eng2.index_)
            return false;
        return true;
    }

    friend bool operator!=(
        const MKLEngine<BRNG, Bits> &eng1, const MKLEngine<BRNG, Bits> &eng2)
    {
        return !(eng1 == eng2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const MKLEngine<BRNG, Bits> &eng)
    {
        if (!os.good())
            return os;

        os << eng.stream_.get_brng() << ' ';
        std::size_t n = static_cast<std::size_t>(eng.stream_.get_size());
        if (n % sizeof(std::uint64_t) != 0)
            n += sizeof(std::uint64_t) - n % sizeof(std::uint64_t);
        n /= sizeof(std::uint64_t);
        Vector<std::uint64_t> s(n);
        eng.stream_.save_m(reinterpret_cast<char *>(s.data()));
        for (std::size_t i = 0; i != n; ++i)
            os << s[i] << ' ';
        os << eng.buffer_ << ' ';
        os << eng.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, MKLEngine<BRNG, Bits> &eng)
    {
        if (!is.good())
            return is;

        MKL_INT brng = 0;
        MKLStream stream(BRNG, 1);
        std::array<result_type, M_> buffer;
        std::size_t index = 0;

        is >> std::ws >> brng;
        if (is.good())
            stream.reset(brng, 1);
        else
            return is;

        std::size_t n = static_cast<std::size_t>(eng.stream_.get_size());
        if (n % sizeof(std::uint64_t) != 0)
            n += sizeof(std::uint64_t) - n % sizeof(std::uint64_t);
        n /= sizeof(std::uint64_t);
        Vector<std::uint64_t> s(n);
        for (std::size_t i = 0; i != n; ++i)
            is >> std::ws >> s[i];
        if (is.good())
            stream.load_m(reinterpret_cast<const char *>(s.data()));
        else
            return is;

        is >> std::ws >> buffer;
        is >> std::ws >> index;

        if (is.good()) {
            eng.stream_ = stream;
            eng.buffer_ = buffer;
            eng.index_ = index;
        }

        return is;
    }

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

template <MKL_INT BRNG, int Bits>
inline void rng_rand(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    typename MKLEngine<BRNG, Bits>::result_type *r)
{
    rng(n, r);
}

template <MKL_INT BRNG, int Bits>
inline void bernoulli_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, int *r, double p)
{
    rng.stream().bernoulli(static_cast<MKL_INT>(n), r, p);
}

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float alpha, float beta)
{
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float lambda)
{
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double lambda)
{
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float alpha, float beta)
{
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float location, float scale)
{
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double location, double scale)
{
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float m, float s)
{
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double m, double s)
{
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float mean, float stddev)
{
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double mean, double stddev)
{
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, std::size_t m, const float *mean, const float *chol)
{
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, std::size_t m, const double *mean, const double *chol)
{
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float sigma)
{
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<float>() * sigma);
}

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double sigma)
{
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<double>() * sigma);
}

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r)
{
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r)
{
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

} // namespace vsmc

#endif // VSMC_RNG_MKL_HPP
