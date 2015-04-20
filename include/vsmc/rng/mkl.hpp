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
#include <mkl.h>

#define VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Dist)    \
    VSMC_STATIC_ASSERT((std::is_same<FPType, float>::value ||                \
                           std::is_same<FPType, double>::value),             \
        USE_MKL##Dist##Distribution_##WITH_A_RESULT_TYPE_OTHER_THAN_float_OR_double)

#define VSMC_RUNTIME_ASSERT_RNG_MKL_VSL_OFFSET(offset)                       \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MNE()),                           \
        ("**MKLOffsetDynamic** "                                             \
         "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS"))

#ifndef VSMC_RNG_MKL_VSL_BUFFER_SIZE
#define VSMC_RNG_MKL_VSL_BUFFER_SIZE 1024
#endif

// clang-format off

#define VSMC_DEFINE_RNG_MKL_VSL_BRNG(BRNG)                                   \
    case BRNG: return #BRNG

#define VSMC_DEFINE_RNG_MKL_VSL_ERR(STATUS)                                  \
    case STATUS: return #STATUS

// clang-format on

namespace vsmc
{

template <MKL_INT> class MKLStream;
template <typename, typename> class MKLDistribution;
template <MKL_INT, typename> class MKLEngine;

class MKLUniformBits32Distribution;
class MKLUniformBits64Distribution;

template <typename, MKL_INT = VSL_RNG_METHOD_UNIFORM_STD>
class MKLUniformDistribution;

template <MKL_INT = VSL_RNG_METHOD_BERNOULLI_ICDF>
class MKLBernoulliDistribution;
template <MKL_INT = VSL_RNG_METHOD_GEOMETRIC_ICDF>
class MKLGeometricDistribution;
template <MKL_INT = VSL_RNG_METHOD_BINOMIAL_BTPE>
class MKLBinomialDistribution;
template <MKL_INT = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE>
class MKLHypergeometricDistribution;
template <MKL_INT = VSL_RNG_METHOD_POISSON_PTPE> class MKLPoissonDistribution;
template <MKL_INT = VSL_RNG_METHOD_NEGBINOMIAL_NBAR>
class MKLNegBinomialDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2>
class MKLGaussianDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_EXPONENTIAL_ICDF>
class MKLExponentialDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_LAPLACE_ICDF>
class MKLLaplaceDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_WEIBULL_ICDF>
class MKLWeibullDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_CAUCHY_ICDF>
class MKLCauchyDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_RAYLEIGH_ICDF>
class MKLRayleighDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2>
class MKLLognormalDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GUMBEL_ICDF>
class MKLGumbelDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_GAMMA_GNORM>
class MKLGammaDistribution;
template <typename = double, MKL_INT = VSL_RNG_METHOD_BETA_CJA>
class MKLBetaDistribution;

namespace internal
{

inline std::string mkl_vsl_brng_str(MKL_INT BRNG)
{
    switch (BRNG) {
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_MCG31);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_R250);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_MRG32K3A);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_WH);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_MCG59);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_MT19937);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_MT2203);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_SFMT19937);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_SOBOL);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_NIEDERR);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_IABSTRACT);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_DABSTRACT);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_SABSTRACT);
        VSMC_DEFINE_RNG_MKL_VSL_BRNG(VSL_BRNG_NONDETERM);
        default: return "Unknown";
    }
}

inline std::string mkl_vsl_error_str(int status)
{
    switch (status) {
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_OK);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_BADARGS);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_CPU_NOT_SUPPORTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_FEATURE_NOT_IMPLEMENTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_MEM_FAILURE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_NULL_PTR);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_ERROR_UNKNOWN);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_FILE_FORMAT);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_MEM_FORMAT);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_NBITS);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_NSEEDS);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_STREAM);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_UPDATE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BAD_WORD_SIZE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BRNG_NOT_SUPPORTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BRNG_TABLE_FULL);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_BRNGS_INCOMPATIBLE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_FILE_CLOSE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_FILE_OPEN);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_FILE_READ);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_FILE_WRITE);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_INVALID_BRNG_INDEX);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_NO_NUMBERS);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_UNSUPPORTED_FILE_VER);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED);
        VSMC_DEFINE_RNG_MKL_VSL_ERR(
            VSL_RNG_ERROR_NONDETERM_NRETRIES_EXCEEDED);
        default: return "UNKNOWN";
    }
}

#if VSMC_NO_RUNTIME_ASSERT
inline void mkl_vsl_error_check(MKL_INT, int, const char *, const char *) {}
#else
inline void mkl_vsl_error_check(
    MKL_INT BRNG, int status, const char *func, const char *mklf)
{
    if (status == VSL_ERROR_OK)
        return;

    std::string msg("**vsmc::");
    msg += func;
    msg += " failure";
    msg += "; MKL function: ";
    msg += mklf;

    msg += "; BRNG: ";
    msg += mkl_vsl_brng_str(BRNG);
    msg += "; Error code: ";
    msg += mkl_vsl_error_str(status);

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());
} // error_check
#endif

} // namespace vsmc::internal

namespace traits
{

/// \brief MKLEngine uniform bits trait
/// \ingroup Traits
///
/// \details
/// To use MKLEngine with those MKL BRNG that has not been typedefed by vSMC,
/// one need to specialize this trait, which has member type `type`, and this
/// type has member
/// `operator() (MKLStream<BRNG> &, MKL_INT, ResultType *)` such that given
/// the
/// stream object, it is able to generate uniform integers.
///
/// This traits also need to have two static constant member data, `min` and
/// `max`
template <MKL_INT, typename> struct MKLUniformBitsTrait;

/// \brief Default uniform bits generator for MKLEngine with `unsigned` output
/// \ingroup Traits
template <MKL_INT BRNG> struct MKLUniformBitsTrait<BRNG, unsigned> {
    typedef MKLUniformBits32Distribution type;
    static constexpr const unsigned min VSMC_MNE = 0;
    static constexpr const unsigned max VSMC_MNE =
        static_cast<unsigned>(~(static_cast<unsigned>(0)));
}; // struct MKLUniformBitsTrait

/// \brief Default uniform bits generator for MKLEngine with
/// `unsigned MKL_INT64` output
/// \ingroup Traits
template <MKL_INT BRNG> struct MKLUniformBitsTrait<BRNG, unsigned MKL_INT64> {
    typedef MKLUniformBits64Distribution type;
    static constexpr const unsigned MKL_INT64 min VSMC_MNE = 0;
    static constexpr const unsigned MKL_INT64 max VSMC_MNE =
        static_cast<unsigned MKL_INT64>(
            ~(static_cast<unsigned MKL_INT64>(0)));
}; // struct MKLUniformBitsTrait

/// \brief Default seed for MKL RNG
/// \ingroup Traits
template <MKL_INT>
struct MKLSeedTrait : public std::integral_constant<MKL_UINT, 101> {
};

/// \brief Default seed for MKL Sobol quasi-RNG
template <>
struct MKLSeedTrait<VSL_BRNG_SOBOL>
    : public std::integral_constant<MKL_UINT, 10> {
};

/// \brief Default seed for MKL Niederr quasi-RNG
template <>
struct MKLSeedTrait<VSL_BRNG_NIEDERR>
    : public std::integral_constant<MKL_UINT, 10> {
};

} // namespace traits

namespace internal
{

struct MKLOffsetZero {
    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return 0; }
    static void offset(MKL_INT) {}
    static constexpr MKL_INT offset() { return 0; }
}; // struct OffsetZero

template <MKL_INT MaxOffset> struct MKLOffsetDynamic {
    MKLOffsetDynamic() : offset_(0) {}

    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return MaxOffset; }

    void offset(MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_RNG_MKL_VSL_OFFSET(n);
        offset_ = n;
    }

    MKL_INT offset() const { return offset_; }

    private:
    MKL_INT offset_;
}; // struct OffsetDynamic

template <MKL_INT> struct MKLOffset {
    typedef MKLOffsetZero type;
};

template <> struct MKLOffset<VSL_BRNG_MT2203> {
    typedef MKLOffsetDynamic<6024> type;
};

template <> struct MKLOffset<VSL_BRNG_WH> {
    typedef MKLOffsetDynamic<273> type;
};

struct MKLSkipAheadVSL {
    typedef long long size_type;

    template <MKL_INT BRNG>
    void operator()(MKLStream<BRNG> &stream, size_type nskip)
    {
        if (nskip == 0)
            return;

        int status = ::vslSkipAheadStream(stream.ptr(), nskip);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLSkipAheadVSL::skip", "::vslSkipAheadStream");
    }

    static void buffer_size(MKL_INT) {}
    static MKL_INT buffer_size() { return 0; }
}; // struct SkipAheadVSL

template <MKL_INT BRNG, typename ResultType> struct MKLSkipAheadForce {
    typedef MKL_INT size_type;

    MKLSkipAheadForce() : buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE) {}

    void operator()(MKLStream<BRNG> &stream, size_type nskip)
    {
        if (nskip == 0)
            return;

        if (nskip < buffer_size_) {
            if (buffer_.size() < nskip)
                buffer_.resize(nskip);
            uniform_bits_(stream, nskip, &buffer_[0]);
        } else {
            buffer_.resize(buffer_size_);
            size_type repeat = nskip / buffer_size_;
            size_type remain = nskip - repeat * buffer_size_;
            for (size_type r = 1; r != repeat + 1; ++r) {
                size_type n = r * buffer_size_;
                uniform_bits_(stream, n, &buffer_[0]);
            }
            uniform_bits_(stream, remain, &buffer_[0]);
        }
    }

    void buffer_size(MKL_INT size)
    {
        buffer_size_ = size > 0 ? size : VSMC_RNG_MKL_VSL_BUFFER_SIZE;
    }

    MKL_INT buffer_size() { return buffer_size_; }

    private:
    std::vector<ResultType, AlignedAllocator<ResultType>> buffer_;
    typename traits::MKLUniformBitsTrait<BRNG, ResultType>::type
        uniform_bits_;
    MKL_INT buffer_size_;
}; // strut SkipAheadForce

template <MKL_INT BRNG, typename ResultType> struct MKLSkipAhead {
    typedef MKLSkipAheadForce<BRNG, ResultType> type;
};

template <typename ResultType>
struct MKLSkipAhead<VSL_BRNG_MCG31, ResultType> {
    typedef MKLSkipAheadVSL type;
};

template <typename ResultType>
struct MKLSkipAhead<VSL_BRNG_MCG59, ResultType> {
    typedef MKLSkipAheadVSL type;
};

template <typename ResultType>
struct MKLSkipAhead<VSL_BRNG_MRG32K3A, ResultType> {
    typedef MKLSkipAheadVSL type;
};

template <typename ResultType>
struct MKLSkipAhead<VSL_BRNG_SOBOL, ResultType> {
    typedef MKLSkipAheadVSL type;
};

template <typename ResultType>
struct MKLSkipAhead<VSL_BRNG_NIEDERR, ResultType> {
    typedef MKLSkipAheadVSL type;
};

} // namespace vsmc::internal

/// \brief MKL RNG C++11 engine stream
/// \ingroup MKLRNG
template <MKL_INT BRNG>
class MKLStream : public internal::MKLOffset<BRNG>::type
{
    public:
    explicit MKLStream(
        MKL_UINT s = traits::MKLSeedTrait<BRNG>::value, MKL_INT offset = 0)
        : seed_(s), stream_ptr_(nullptr), property_()
    {
        this->offset(offset);
        int status = VSL_ERROR_OK;

        status = ::vslNewStream(&stream_ptr_, BRNG + this->offset(), seed_);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLStream::Stream", "::vslNewStream");

        status = ::vslGetBrngProperties(BRNG, &property_);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLStream::Stream", "::vslGetBrngProperties");
    }

    template <typename SeedSeq>
    explicit MKLStream(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLStream<BRNG>>::value>::type * = nullptr)
        : seed_(0), stream_ptr_(nullptr), property_()
    {
        seq.generate(&seed_, &seed_ + 1);
        int status = VSL_ERROR_OK;

        status = ::vslNewStream(&stream_ptr_, BRNG + this->offset(), seed_);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLStream::Stream", "::vslNewStream");

        status = ::vslGetBrngProperties(BRNG, &property_);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLStream::Stream", "::vslGetBrngProperties");
    }

    MKLStream(const MKLStream<BRNG> &other)
        : internal::MKLOffset<BRNG>::type(other),
          seed_(other.seed_),
          property_(other.property_)
    {
        int status = ::vslCopyStream(&stream_ptr_, other.stream_ptr_);
        internal::mkl_vsl_error_check(
            BRNG, status, "MKLStream::Stream", "::vslCopyStream");
    }

    MKLStream<BRNG> &operator=(const MKLStream<BRNG> &other)
    {
        if (this != &other) {
            internal::MKLOffset<BRNG>::type::operator=(other);
            int status = ::vslCopyStreamState(stream_ptr_, other.stream_ptr_);
            internal::mkl_vsl_error_check(
                BRNG, status, "MKLStream::operator=", "::vslCopyStreamState");
            property_ = other.property_;
        }

        return *this;
    }

    MKLStream(MKLStream<BRNG> &&other)
        : internal::MKLOffset<BRNG>::type(std::move(other)),
          seed_(other.seed_),
          stream_ptr_(other.stream_ptr_),
          property_(other.property_)
    {
        other.stream_ptr_ = nullptr;
    }

    MKLStream<BRNG> &operator=(MKLStream<BRNG> &&other)
    {
        using std::swap;

        if (this != other) {
            internal::MKLOffset<BRNG>::type::operator=(std::move(other));
            swap(seed_, other.seed_);
            swap(stream_ptr_, other.stream_ptr_);
            swap(property_, other.property_);
        }

        return *this;
    }

    ~MKLStream()
    {
        if (stream_ptr_ != nullptr)
            ::vslDeleteStream(&stream_ptr_);
    }

    void seed(MKL_UINT s)
    {
        seed_ = s;
        int status = VSL_ERROR_OK;

        if (stream_ptr_ == nullptr) {
            status = ::vslNewStream(&stream_ptr_, BRNG + this->offset(), s);
            internal::mkl_vsl_error_check(
                BRNG, status, "MKLStream::seed", "::vslNewStream");
        } else {
            VSLStreamStatePtr new_stream_ptr;

            status =
                ::vslNewStream(&new_stream_ptr, BRNG + this->offset(), s);
            internal::mkl_vsl_error_check(
                BRNG, status, "MKLStream::seed", "::vslNewStream");

            status = ::vslCopyStreamState(stream_ptr_, new_stream_ptr);
            internal::mkl_vsl_error_check(
                BRNG, status, "MKLStream::seed", "::vslCopyStreamState");

            status = ::vslDeleteStream(&new_stream_ptr);
            internal::mkl_vsl_error_check(
                BRNG, status, "MKLStream::seed", "::vslDeleteStream");
        }
    }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLStream<BRNG>>::value>::type * = nullptr)
    {
        seq.generate(&seed_, &seed_ + 1);
        seed(seed_);
    }

    VSLStreamStatePtr ptr() const { return stream_ptr_; }

    const VSLBRngProperties &property() const { return property_; }

    private:
    MKL_UINT seed_;
    VSLStreamStatePtr stream_ptr_;
    VSLBRngProperties property_;
}; // class MKLStream

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
template <MKL_INT BRNG, typename ResultType> class MKLEngine
{
    public:
    typedef ResultType result_type;
    typedef MKLStream<BRNG> stream_type;

    explicit MKLEngine(
        MKL_UINT s = traits::MKLSeedTrait<BRNG>::value, MKL_INT offset = 0)
        : stream_(s, offset),
          buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE),
          index_(buffer_size_)
    {
    }

    template <typename SeedSeq>
    explicit MKLEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, ResultType>>::value>::type * = nullptr)
        : stream_(seq),
          buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE),
          index_(buffer_size_)
    {
    }

    void seed(MKL_UINT s) { stream_.seed(s); }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, ResultType>>::value>::type * = nullptr)
    {
        stream_.seed(seq);
    }

    result_type operator()()
    {
        if (index_ == buffer_size_) {
            buffer_.resize(static_cast<std::size_t>(buffer_size_));
            uniform_bits_(stream_, buffer_size_, &buffer_[0]);
            index_ = 0;
        }

        return buffer_[static_cast<std::size_t>(index_++)];
    }

    /// \brief Discard results
    ///
    /// \details
    /// The the behavior is slightly different from that in C++11 standard.
    /// Calling `discard(nskip)` is not equivalent to call `operator()`
    /// `nskip`
    /// times. Instead, it ensures that at least `nskip` results are
    /// discarded.
    /// There may be a few more than `nskip` also discarded.
    void discard(std::size_t nskip)
    {
        skip_ahead_(stream_, static_cast<typename internal::MKLSkipAhead<BRNG,
                                 ResultType>::type::size_type>(nskip));
        index_ = buffer_size_;
    }

    static constexpr const result_type _Min =
        traits::MKLUniformBitsTrait<BRNG, ResultType>::min VSMC_MNE;
    static constexpr const result_type _Max =
        traits::MKLUniformBitsTrait<BRNG, ResultType>::max VSMC_MNE;

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    stream_type &stream() { return stream_; }
    const stream_type &stream() const { return stream_; }

    /// \brief Set the buffer size, zero or negative value restore the default
    void buffer_size(MKL_INT size)
    {
        buffer_size_ = size > 0 ? size : VSMC_RNG_MKL_VSL_BUFFER_SIZE;
    }

    MKL_INT buffer_size() { return buffer_size_; }

    private:
    stream_type stream_;
    typename internal::MKLSkipAhead<BRNG, ResultType>::type skip_ahead_;
    typename traits::MKLUniformBitsTrait<BRNG, ResultType>::type
        uniform_bits_;
    std::vector<result_type, AlignedAllocator<result_type>> buffer_;
    MKL_INT buffer_size_;
    MKL_INT index_;
}; // class MKLEngine

/// \brief A 59-bits multiplicative congruential generator
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MCG59, unsigned> MKL_MCG59;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT19937, unsigned> MKL_MT19937;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bits)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT19937, unsigned MKL_INT64> MKL_MT19937_64;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT2203, unsigned> MKL_MT2203;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// (64-bits)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_MT2203, unsigned MKL_INT64> MKL_MT2203_64;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_SFMT19937, unsigned> MKL_SFMT19937;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bits)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_SFMT19937, unsigned MKL_INT64> MKL_SFMT19937_64;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_NONDETERM, unsigned> MKL_NONDETERM;

/// \brief A non-determinstic random number generator (64-bits)
/// \ingroup MKLRNG
typedef MKLEngine<VSL_BRNG_NONDETERM, unsigned MKL_INT64> MKL_NONDETERM_64;

/// \brief Base class of MKL distribution
/// \ingroup MKLRNG
template <typename ResultType, typename Derived> class MKLDistribution
{
    public:
    typedef ResultType result_type;

    MKLDistribution()
        : buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE), index_(buffer_size_)
    {
    }

    template <MKL_INT BRNG> result_type operator()(MKLStream<BRNG> &stream)
    {
        if (index_ == buffer_size_) {
            buffer_.resize(static_cast<std::size_t>(buffer_size_));
            static_cast<Derived *>(this)->generate(
                stream, buffer_size_, &buffer_[0]);
            index_ = 0;
        }

        return buffer_[index_++];
    }

    template <MKL_INT BRNG>
    void operator()(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        static_cast<Derived *>(this)->generate(stream, n, r);
    }

    void reset() { index_ = buffer_size_; }

    /// \brief Set the buffer size, zero or negative value restore the default
    void buffer_size(MKL_INT size)
    {
        buffer_size_ = size > 0 ? size : VSMC_RNG_MKL_VSL_BUFFER_SIZE;
    }

    MKL_INT buffer_size() const { return buffer_size_; }

    protected:
    template <MKL_INT BRNG>
    void generate_error_check(int status, const char *name)
    {
        if (status == VSL_ERROR_OK)
            return;

        std::string dist_name(name);
        dist_name = "MKL" + dist_name + "Distribution::generate";
        std::string mkl_vsl_name(
            mkl_vsl_name_prefix(static_cast<result_type>(0)));
        mkl_vsl_name += "Rng";
        mkl_vsl_name += name;
        internal::mkl_vsl_error_check(
            BRNG, status, dist_name.c_str(), mkl_vsl_name.c_str());
    }

    private:
    std::vector<result_type, AlignedAllocator<result_type>> buffer_;
    MKL_INT buffer_size_;
    MKL_INT index_;

    std::string mkl_vsl_name_prefix(MKL_INT) { return "vi"; }
    std::string mkl_vsl_name_prefix(unsigned) { return "vi"; }
    std::string mkl_vsl_name_prefix(unsigned MKL_INT64) { return "vi"; }
    std::string mkl_vsl_name_prefix(float) { return "vs"; }
    std::string mkl_vsl_name_prefix(double) { return "vd"; }
}; // class MKLDistribution

/// \brief MKL uniform bits distribution (32-bits)
/// \ingroup MKLRNG
class MKLUniformBits32Distribution
    : public MKLDistribution<unsigned, MKLUniformBits32Distribution>
{
    public:
    typedef unsigned result_type;

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits32(
            VSL_RNG_METHOD_UNIFORMBITS32_STD, stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "UniformBits32");
    }
}; // class MKLUniformBits32Distribution

/// \brief MKL uniform bits distribution (64-bits)
/// \ingroup MKLRNG
class MKLUniformBits64Distribution
    : public MKLDistribution<unsigned MKL_INT64, MKLUniformBits64Distribution>
{
    public:
    typedef unsigned MKL_INT64 result_type;

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngUniformBits64(
            VSL_RNG_METHOD_UNIFORMBITS64_STD, stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "UniformBits64");
    }
}; // class MKLUniformBits64Distribution

/// \brief MKL Uniform distribution
/// \ingroup MKLRNG
template <typename ResultType, MKL_INT Method>
class MKLUniformDistribution : public MKLDistribution<ResultType,
                                   MKLUniformDistribution<ResultType, Method>>
{
    public:
    typedef ResultType result_type;

    explicit MKLUniformDistribution(result_type a = 0, result_type b = 1)
        : a_(a), b_(b)
    {
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Uniform");
    }

    private:
    result_type a_;
    result_type b_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, MKL_INT *r)
    {
        return ::viRngUniform(Method, ptr, n, r, a_, b_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngUniform(Method, ptr, n, r, a_, b_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngUniform(Method, ptr, n, r, a_, b_);
    }
}; // class MKLUniformDistribution

/// \brief MKL Bernoulli distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLBernoulliDistribution
    : public MKLDistribution<MKL_INT, MKLBernoulliDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    explicit MKLBernoulliDistribution(double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngBernoulli(Method, stream.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Bernoulli");
    }

    private:
    double p_;
}; // class MKLBernoulliDistribution

/// \brief MKL Geometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLGeometricDistribution
    : public MKLDistribution<MKL_INT, MKLGeometricDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    explicit MKLGeometricDistribution(double p = 0.5) : p_(p) {}

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngGeometric(Method, stream.ptr(), n, r, p_);
        this->template generate_error_check<BRNG>(status, "Geometric");
    }

    private:
    double p_;
}; // class MKLGeometricDistribution

/// \brief MKL Binomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLBinomialDistribution
    : public MKLDistribution<MKL_INT, MKLBinomialDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    explicit MKLBinomialDistribution(result_type ntrial = 1, double p = 0.5)
        : ntrial_(ntrial), p_(p)
    {
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngBinomial(Method, stream.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "Binomial");
    }

    private:
    result_type ntrial_;
    double p_;
}; // class MKLBinomialDistribution

/// \brief MKL Hypergeometric distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLHypergeometricDistribution
    : public MKLDistribution<MKL_INT, MKLHypergeometricDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    MKLHypergeometricDistribution(
        result_type population, result_type sample, result_type mask)
        : l_(population), s_(sample), m_(mask)
    {
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status =
            ::viRngHypergeometric(Method, stream.ptr(), n, r, l_, s_, m_);
        this->template generate_error_check<BRNG>(status, "Hypergeometric");
    }

    private:
    result_type l_;
    result_type s_;
    result_type m_;
}; // class MKLHypergeometricDistribution

/// \brief MKL Poisson distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLPoissonDistribution
    : public MKLDistribution<MKL_INT, MKLPoissonDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    explicit MKLPoissonDistribution(double lambda = 1) : lambda_(lambda) {}

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = ::viRngPoisson(Method, stream.ptr(), n, r, lambda_);
        this->template generate_error_check<BRNG>(status, "Poisson");
    }

    private:
    double lambda_;
}; // class MKLPoissonDistribution

/// \brief MKL NegBinomial distribution
/// \ingroup MKLRNG
template <MKL_INT Method>
class MKLNegBinomialDistribution
    : public MKLDistribution<MKL_INT, MKLNegBinomialDistribution<Method>>
{
    public:
    typedef MKL_INT result_type;

    explicit MKLNegBinomialDistribution(double ntrial = 1, double p = 0.5)
        : ntrial_(ntrial), p_(p)
    {
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status =
            ::viRngNegBinomial(Method, stream.ptr(), n, r, ntrial_, p_);
        this->template generate_error_check<BRNG>(status, "NegBinomial");
    }

    private:
    double ntrial_;
    double p_;
}; // class MKLNegBinomialDistribution

/// \brief MKL Gaussian distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGaussianDistribution
    : public MKLDistribution<FPType, MKLGaussianDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLGaussianDistribution(result_type mean = 0, result_type sd = 1)
        : mean_(mean), sd_(sd)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Gaussian);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gaussian");
    }

    private:
    result_type mean_;
    result_type sd_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngGaussian(Method, ptr, n, r, mean_, sd_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngGaussian(Method, ptr, n, r, mean_, sd_);
    }
}; // class MKLGaussianDistribution

/// \brief MKL Exponential distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLExponentialDistribution
    : public MKLDistribution<FPType,
          MKLExponentialDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLExponentialDistribution(
        result_type displacement = 0, result_type scale = 1)
        : disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(
            FPType, Exponential);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Exponential");
    }

    private:
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngExponential(Method, ptr, n, r, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngExponential(Method, ptr, n, r, disp_, scale_);
    }
}; // class MKLExponentialDistribution

/// \brief MKL Laplace distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLLaplaceDistribution
    : public MKLDistribution<FPType, MKLLaplaceDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLLaplaceDistribution(
        result_type mean = 0, result_type scale = 1)
        : mean_(mean), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Laplace);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Laplace");
    }

    private:
    result_type mean_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngLaplace(Method, ptr, n, r, mean_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngLaplace(Method, ptr, n, r, mean_, scale_);
    }
}; // class MKLLaplaceDistribution

/// \brief MKL Weibull distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLWeibullDistribution
    : public MKLDistribution<FPType, MKLWeibullDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLWeibullDistribution(result_type shape = 1,
        result_type displacement = 0, result_type scale = 1)
        : shape_(shape), disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Weibull);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Weibull");
    }

    private:
    result_type shape_;
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngWeibull(Method, ptr, n, r, shape_, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngWeibull(Method, ptr, n, r, shape_, disp_, scale_);
    }
}; // class MKLWeibullDistribution

/// \brief MKL Cauchy distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLCauchyDistribution
    : public MKLDistribution<FPType, MKLCauchyDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLCauchyDistribution(
        result_type displacement = 0, result_type scale = 1)
        : disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Cauchy);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Cauchy");
    }

    private:
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngCauchy(Method, ptr, n, r, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngCauchy(Method, ptr, n, r, disp_, scale_);
    }
}; // class MKLCauchyDistribution

/// \brief MKL Rayleigh distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLRayleighDistribution
    : public MKLDistribution<FPType, MKLRayleighDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLRayleighDistribution(
        result_type displacement = 0, result_type scale = 1)
        : disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Rayleigh);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Rayleigh");
    }

    private:
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngRayleigh(Method, ptr, n, r, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngRayleigh(Method, ptr, n, r, disp_, scale_);
    }
}; // class MKLRayleighDistribution

/// \brief MKL Lognormal distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLLognormalDistribution
    : public MKLDistribution<FPType, MKLLognormalDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLLognormalDistribution(result_type mean = 0,
        result_type sd = 1, result_type displacement = 0,
        result_type scale = 1)
        : mean_(mean), sd_(sd), disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(
            FPType, Lognormal);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Lognormal");
    }

    private:
    result_type mean_;
    result_type sd_;
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngLognormal(Method, ptr, n, r, mean_, sd_, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngLognormal(Method, ptr, n, r, mean_, sd_, disp_, scale_);
    }
}; // class MKLLognormalDistribution

/// \brief MKL Gumbel distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGumbelDistribution
    : public MKLDistribution<FPType, MKLGumbelDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLGumbelDistribution(
        result_type displacement = 0, result_type scale = 1)
        : disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Gumbel);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gumbel");
    }

    private:
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngGumbel(Method, ptr, n, r, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngGumbel(Method, ptr, n, r, disp_, scale_);
    }
}; // class MKLGumbelDistribution

/// \brief MKL Gamma distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLGammaDistribution
    : public MKLDistribution<FPType, MKLGammaDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLGammaDistribution(result_type shape = 1,
        result_type displacement = 0, result_type scale = 1)
        : shape_(shape), disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Gamma);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Gamma");
    }

    private:
    result_type shape_;
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngGamma(Method, ptr, n, r, shape_, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngGamma(Method, ptr, n, r, shape_, disp_, scale_);
    }
}; // class MKLGammaDistribution

/// \brief MKL Beta distribution
/// \ingroup MKLRNG
template <typename FPType, MKL_INT Method>
class MKLBetaDistribution
    : public MKLDistribution<FPType, MKLBetaDistribution<FPType, Method>>
{
    public:
    typedef FPType result_type;

    explicit MKLBetaDistribution(result_type shape1 = 1,
        result_type shape2 = 1, result_type displacement = 0,
        result_type scale = 1)
        : shape1_(shape1), shape2_(shape2), disp_(displacement), scale_(scale)
    {
        VSMC_STATIC_ASSERT_RNG_MKL_VSL_DISTRIBUTION_FP_TYPE(FPType, Beta);
    }

    template <MKL_INT BRNG>
    void generate(MKLStream<BRNG> &stream, MKL_INT n, result_type *r)
    {
        int status = generate(stream.ptr(), n, r);
        this->template generate_error_check<BRNG>(status, "Beta");
    }

    private:
    result_type shape1_;
    result_type shape2_;
    result_type disp_;
    result_type scale_;

    int generate(VSLStreamStatePtr ptr, MKL_INT n, float *r)
    {
        return ::vsRngBeta(
            Method, ptr, n, r, shape1_, shape2_, disp_, scale_);
    }

    int generate(VSLStreamStatePtr ptr, MKL_INT n, double *r)
    {
        return ::vdRngBeta(
            Method, ptr, n, r, shape1_, shape2_, disp_, scale_);
    }
}; // class MKLBetaDistribution

} // namespace vsmc

#endif // VSMC_RNG_MKL_HPP
