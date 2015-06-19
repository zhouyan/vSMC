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
#include <vsmc/utility/aligned_memory.hpp>
#include <vsmc/utility/mkl.hpp>

#ifndef VSMC_RNG_MKL_VSL_BUFFER_SIZE
#define VSMC_RNG_MKL_VSL_BUFFER_SIZE 1024
#endif

namespace vsmc
{

namespace internal
{

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
    template <MKL_INT BRNG>
    static void eval(MKLStream<BRNG> &stream, MKL_INT n, unsigned *r)
    {
        int status = ::viRngUniformBits32(
            VSL_RNG_METHOD_UNIFORMBITS32_STD, stream.ptr(), n, r);
        mkl_error_check(
            status, "MKLUniformBits::eval", "::viRngUniformBits32");
    }
}; // class MKLUniformBits

template <>
class MKLUniformBits<64>
{
    public:
    template <MKL_INT BRNG>
    static void eval(MKLStream<BRNG> &stream, MKL_INT n, unsigned MKL_INT64 *r)
    {
        int status = ::viRngUniformBits64(
            VSL_RNG_METHOD_UNIFORMBITS64_STD, stream.ptr(), n, r);
        mkl_error_check(
            status, "MKLUniformBits::eval", "::viRngUniformBits64");
    }
}; // class MKLUniformBits

class MKLDiscardSkipAhead
{
    public:
    using size_type = long long;

    template <MKL_INT BRNG>
    void operator()(MKLStream<BRNG> &stream, size_type nskip)
    {
        if (nskip == 0)
            return;

        int status = ::vslSkipAheadStream(stream.ptr(), nskip);
        mkl_error_check(
            status, "MKLDiscardSkipAhead::skip", "::vslSkipAheadStream");
    }

    static void buffer_size(MKL_INT) {}
    static MKL_INT buffer_size() { return 0; }
}; // class DiscardSkipAhead

template <MKL_INT BRNG, std::size_t Bits>
class MKLDiscardGeneral
{
    public:
    using size_type = MKL_INT;

    MKLDiscardGeneral() : buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE) {}

    void operator()(MKLStream<BRNG> &stream, size_type nskip)
    {
        if (nskip == 0)
            return;

        buffer_.resize(buffer_size_);
        while (nskip > buffer_size_) {
            MKLUniformBits<Bits>::eval(stream, buffer_size_, buffer_.data());
            nskip -= buffer_size_;
        }
        MKLUniformBits<Bits>::eval(stream, nskip, buffer_.data());
    }

    void buffer_size(MKL_INT size)
    {
        buffer_size_ = size > 0 ? size : VSMC_RNG_MKL_VSL_BUFFER_SIZE;
    }

    MKL_INT buffer_size() { return buffer_size_; }

    private:
    Vector<MKLResultType<Bits>> buffer_;
    MKL_INT buffer_size_;
}; // class DiscardGeneral

template <MKL_INT BRNG, std::size_t Bits>
class MKLDiscard
{
    public:
    using type = MKLDiscardGeneral<BRNG, Bits>;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_MCG31, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_MCG59, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_MRG32K3A, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_SOBOL, Bits>
{
    public:
    using type = MKLDiscardSkipAhead;
}; // clas MKLDiscard

template <std::size_t Bits>
class MKLDiscard<VSL_BRNG_NIEDERR, Bits>
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
    using stream_type = MKLStream<BRNG>;

    explicit MKLEngine(MKL_UINT s = MKLSeed<BRNG>::value, MKL_INT offset = 0)
        : stream_(s, offset)
        , buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE)
        , index_(buffer_size_)
    {
    }

    template <typename SeedSeq>
    explicit MKLEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
        : stream_(seq)
        , buffer_size_(VSMC_RNG_MKL_VSL_BUFFER_SIZE)
        , index_(buffer_size_)
    {
    }

    void seed(MKL_UINT s) { stream_.seed(s); }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
    {
        stream_.seed(seq);
    }

    result_type operator()()
    {
        if (index_ == buffer_size_) {
            buffer_.resize(static_cast<std::size_t>(buffer_size_));
            internal::MKLUniformBits<Bits>::eval(
                stream_, buffer_size_, buffer_.data());
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
        discard_(stream_, static_cast<typename internal::MKLDiscard<BRNG,
                              Bits>::type::size_type>(nskip));
        index_ = buffer_size_;
    }

    static constexpr result_type _Min = 0;
    static constexpr result_type _Max = VSMC_MAX_UINT(result_type);

    static constexpr result_type min VSMC_MNE() { return _Min; }
    static constexpr result_type max VSMC_MNE() { return _Max; }

    stream_type &stream() { return stream_; }
    const stream_type &stream() const { return stream_; }

    /// \brief Set the buffer size, zero or negative value restore the
    /// default
    void buffer_size(MKL_INT size)
    {
        buffer_size_ = size > 0 ? size : VSMC_RNG_MKL_VSL_BUFFER_SIZE;
    }

    MKL_INT buffer_size() { return buffer_size_; }

    private:
    stream_type stream_;
    internal::MKLDiscard<BRNG, Bits> discard_;
    Vector<result_type> buffer_;
    MKL_INT buffer_size_;
    MKL_INT index_;
}; // class MKLEngine

/// \brief A 59-bits multiplicative congruential generator
/// \ingroup MKLRNG
using MKL_MCG59 = MKLEngine<VSL_BRNG_MCG59, 32>;

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

} // namespace vsmc

#endif // VSMC_RNG_MKL_HPP
