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

#define VSMC_RUNTIME_WARNING_RNG_MKL_OFFSET                                   \
    VSMC_RUNTIME_WARNING((offset < MaxOffset),                                \
        "**MKLEngine** EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS")

namespace vsmc
{

namespace internal
{

template <MKL_INT BRNG>
class MKLMaxOffset : public std::integral_constant<MKL_INT, 0>
{
}; // MKLMaxOffset;

template <>
class MKLMaxOffset<VSL_BRNG_MT2203>
    : public std::integral_constant<MKL_INT, 6024>
{
}; // MKLMaxOffset

template <>
class MKLMaxOffset<VSL_BRNG_WH> : public std::integral_constant<MKL_INT, 273>
{
}; // MKLMaxOffset

template <MKL_INT BRNG, MKL_INT MaxOffset = MKLMaxOffset<BRNG>::value>
class MKLOffset
{
    public:
    static MKL_INT eval(MKL_INT offset)
    {
        VSMC_RUNTIME_WARNING_RNG_MKL_OFFSET;

        return BRNG + offset % MaxOffset;
    }
}; // class MKLOffset

template <MKL_INT BRNG>
class MKLOffset<BRNG, 0>
{
    public:
    static MKL_INT eval(MKL_INT) { return BRNG; }
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

} // namespace vsmc::internal

/// \brief MKL RNG generator
///
/// \details
/// This class is almost like an C++11 engine except its constructors and
/// seeding methods. The RNG type `BRNG` is provided at runtime.
template <int Bits>
class MKLGenerator
{
    static_assert(Bits == 32 || Bits == 64,
        "**MKLGenerator** USED WITH Bits OTHER THAN 32 OR 64");

    public:
    using result_type = internal::MKLResultType<Bits>;

    explicit MKLGenerator(MKL_INT brng, MKL_UINT s = 1) : index_(M_)
    {
        seed(brng, s);
    }

    MKLGenerator(MKL_INT brng, MKL_INT n, unsigned *params) : index_(M_)
    {
        seed(brng, n, params);
    }

    template <typename SeedSeq>
    explicit MKLGenerator(MKL_INT brng, SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLGenerator<Bits>>::value>::type * = nullptr)
        : index_(M_), stream_(brng, 0)
    {
        seed(brng, seq);
    }

    void seed(MKL_INT brng, MKL_UINT s)
    {
        stream_.reset(brng, s);
        index_ = M_;
    }

    void seed(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        stream_.reset(brng, n, params);
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(MKL_INT brng, SeedSeq &seq,
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, MKL_UINT>::value>::type * = nullptr)
    {
        ::VSLBRngProperties properties;
        MKLStream::get_brng_properties(brng, &properties);
        MKL_INT n = properties.NSeeds;
        Vector<unsigned> params(static_cast<std::size_t>(n));
        seq.generate(params.begin(), params.end());
        seed(brng, n, params.data());
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generate();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void operator()(std::size_t n, result_type *r)
    {
        std::size_t remain = M_ - index_;

        if (n < remain) {
            std::memcpy(r, buffer_.data() + index_, sizeof(result_type) * n);
            index_ += n;
            return;
        }

        std::memcpy(r, buffer_.data() + index_, sizeof(result_type) * remain);
        r += remain;
        n -= remain;
        index_ = M_;

        const std::size_t m = n / M_;
        const std::size_t l = n % M_;
        for (std::size_t i = 0; i != m; ++i) {
            internal::MKLUniformBits<Bits>::eval(
                stream_, static_cast<MKL_INT>(M_), r);
            r += M_;
            n -= M_;
        }
        generate();
        std::memcpy(r, buffer_.data(), sizeof(result_type) * l);
        index_ = l;
    }

    void discard(long long nskip)
    {
        std::size_t n = static_cast<std::size_t>(nskip);

        if (n == 0)
            return;

        std::size_t remain = M_ - index_;
        if (n <= remain) {
            index_ += n;
            return;
        }
        n -= remain;
        index_ = M_;

        long long m = static_cast<long long>(n / M_ * M_);
        switch (stream_.get_brng()) {
            case VSL_BRNG_MCG31: stream_.skip_ahead(m); break;
            case VSL_BRNG_MRG32K3A: stream_.skip_ahead(m); break;
            case VSL_BRNG_MCG59: stream_.skip_ahead(m); break;
            case VSL_BRNG_WH: stream_.skip_ahead(m); break;
            case VSL_BRNG_MT19937: stream_.skip_ahead(m); break;
            case VSL_BRNG_SFMT19937: stream_.skip_ahead(m); break;
            case VSL_BRNG_SOBOL: stream_.skip_ahead(m); break;
            case VSL_BRNG_NIEDERR: stream_.skip_ahead(m); break;
            case VSL_BRNG_PHILOX4X32X10: stream_.skip_ahead(m); break;
            case VSL_BRNG_ARS5: stream_.skip_ahead(m); break;
            default:
                while (n > M_) {
                    operator()();
                    n -= M_;
                }
                break;
        };
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

    MKLStream &stream() { return stream_; }
    const MKLStream &stream() const { return stream_; }

    friend bool operator==(
        const MKLGenerator<Bits> &gen1, const MKLGenerator<Bits> &gen2)
    {
        if (gen1.stream_.get_brng() != gen2.stream_.get_brng())
            return false;
        std::size_t n = static_cast<std::size_t>(gen1.stream_.get_size());
        Vector<char> s1(n);
        Vector<char> s2(n);
        gen1.stream_.save_m(s1.data());
        gen2.stream_.save_m(s2.data());
        if (s1 != s2)
            return false;
        if (gen1.buffer_ != gen2.buffer_)
            return false;
        if (gen1.index_ != gen2.index_)
            return false;
        return true;
    }

    friend bool operator!=(
        const MKLGenerator<Bits> &gen1, const MKLGenerator<Bits> &gen2)
    {
        return !(gen1 == gen2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const MKLGenerator<Bits> &gen)
    {
        if (!os.good())
            return os;

        os << gen.stream_.get_brng() << ' ';

        std::size_t n = static_cast<std::size_t>(gen.stream_.get_size());
        if (n % sizeof(std::uint64_t) != 0)
            n += sizeof(std::uint64_t) - n % sizeof(std::uint64_t);
        n /= sizeof(std::uint64_t);
        Vector<std::uint64_t> s(n);
        gen.stream_.save_m(reinterpret_cast<char *>(s.data()));
        os << s << ' ';

        os << gen.buffer_ << ' ';
        os << gen.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, MKLGenerator<Bits> &gen)
    {
        if (!is.good())
            return is;

        MKL_INT brng = 0;
        is >> std::ws >> brng;
        if (!is.good())
            return is;

        MKLStream stream(brng, 1);
        Vector<std::uint64_t> s;
        is >> std::ws >> s;
        if (!is.good())
            return is;

        stream.load_m(reinterpret_cast<const char *>(s.data()));
        Vector<result_type> buffer(M_);
        std::size_t index = 0;
        is >> std::ws >> buffer;
        is >> std::ws >> index;

        if (is.good()) {
            gen.stream_ = std::move(stream);
            gen.buffer_ = std::move(buffer);
            gen.index_ = index;
        }

        return is;
    }

    private:
    static constexpr std::size_t M_ = 1024;

    Vector<result_type> buffer_;
    std::size_t index_;
    MKLStream stream_;

    void generate()
    {
        buffer_.resize(M_);
        internal::MKLUniformBits<Bits>::eval(
            stream_, static_cast<MKL_INT>(M_), buffer_.data());
    }
}; // class MKLGenerator

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, int Bits>
class MKLEngine : public MKLGenerator<Bits>
{
    public:
    explicit MKLEngine(MKL_UINT s = 1) : MKLGenerator<Bits>(BRNG, s) {}

    template <typename SeedSeq>
    explicit MKLEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
        : MKLGenerator<Bits>(BRNG, seq)
    {
    }

    MKLEngine(MKL_UINT s, MKL_INT offset)
        : MKLGenerator<Bits>(internal::MKLOffset<BRNG>::eval(offset), s)
    {
        static_assert(internal::MKLMaxOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");
    }

    void seed(MKL_UINT s) { MKLGenerator<Bits>::seed(s); }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, MKL_UINT>::value>::type * = nullptr)
    {
        MKLGenerator<Bits>::seed(this->stream().get_brng(), seq);
    }

    void seed(MKL_UINT s, MKL_INT offset)
    {
        static_assert(internal::MKLOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");

        MKLGenerator<Bits>::seed(internal::MKLOffset<BRNG>::eval(offset), s);
    }
}; // class MKLEngine

/// \brief A 59-bit multiplicative congruential generator
/// \ingroup MKLRNG
using MKL_MCG59 = MKLEngine<VSL_BRNG_MCG59, 32>;

/// \brief A 59-bit multiplicative congruential generator (64-bit)
/// \ingroup MKLRNG
using MKL_MCG59_64 = MKLEngine<VSL_BRNG_MCG59, 64>;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT19937 = MKLEngine<VSL_BRNG_MT19937, 32>;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
using MKL_MT19937_64 = MKLEngine<VSL_BRNG_MT19937, 64>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT2203 = MKLEngine<VSL_BRNG_MT2203, 32>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
using MKL_MT2203_64 = MKLEngine<VSL_BRNG_MT2203, 64>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number
/// genertor
/// \ingroup MKLRNG
using MKL_SFMT19937 = MKLEngine<VSL_BRNG_SFMT19937, 32>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bit)
/// \ingroup MKLRNG
using MKL_SFMT19937_64 = MKLEngine<VSL_BRNG_SFMT19937, 64>;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
using MKL_NONDETERM = MKLEngine<VSL_BRNG_NONDETERM, 32>;

/// \brief A non-determinstic random number generator (64-bit)
/// \ingroup MKLRNG
using MKL_NONDETERM_64 = MKLEngine<VSL_BRNG_NONDETERM, 64>;

#if INTEL_MKL_VERSION >= 110300

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_ARS5 = MKLEngine<VSL_BRNG_ARS5, 32>;

/// \brief A counter-based random number generator (64-bit)
/// \ingroup MKLRNG
using MKL_ARS5_64 = MKLEngine<VSL_BRNG_ARS5, 64>;

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_PHILOX4X32X10 = MKLEngine<VSL_BRNG_PHILOX4X32X10, 32>;

/// \brief A counter-based random number generator (64-bit)
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
