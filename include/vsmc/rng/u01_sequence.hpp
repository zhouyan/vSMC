//============================================================================
// vSMC/include/vsmc/rng/u01_sequence.hpp
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

#ifndef VSMC_RNG_U01_SEQUENCE_HPP
#define VSMC_RNG_U01_SEQUENCE_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_U01_SEQUENCE(Method)                          \
    VSMC_RUNTIME_ASSERT((n < N_ && (n == n_ || n == n_ + 1 || n_ == N_)),     \
        "**U01Sequence" #Method "::operator[]** INVALID INDEX")

namespace vsmc
{

/// \brief Tranform a sequence of standard uniform random numbers to sorted
/// sequence
/// \ingroup U01Sequence
///
/// \details
/// This function does not acutally sort the sequence `u01`, instead `u01seq`
/// is a sequence generated such that it comes from the same distribution of
/// the sorted sequence.
template <typename RealType>
inline void u01_sorted(std::size_t N, const RealType *u01, RealType *u01seq)
{
    log(N, u01, u01seq);
    RealType lmax = 0;
    for (std::size_t i = 0; i != N; ++i) {
        lmax += u01seq[i] / (N - i);
        u01seq[i] = 1 - std::exp(lmax);
    }
}

/// \brief Transform a sequence of standard uniform random numbers to a
/// stratified sequence
/// \ingroup U01Sequence
template <typename RealType>
inline void u01_stratified(
    std::size_t N, const RealType *u01, RealType *u01seq)
{
    RealType delta = 1 / static_cast<RealType>(N);
    for (std::size_t i = 0; i != N; ++i)
        u01seq[i] = u01[i] * delta + i * delta;
}

/// \brief Transform a single standard uniform random number to a systematic
/// sequence
/// \ingroup U01Sequence
template <typename RealType>
inline void u01_systematic(std::size_t N, RealType u01, RealType *u01seq)
{
    if (N == 0)
        return;

    RealType delta = 1.0 / static_cast<RealType>(N);
    u01seq[0] = u01 * delta;
    for (std::size_t i = 1; i != N; ++i)
        u01seq[i] = u01seq[i - 1] + delta;
}

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by sorting.
/// \ingroup U01Sequence
///
/// \details
/// This is primarily used in resampling algorithms within the library but can
/// be used for other purposes. It deals with the usage case similar to the
/// following,
/// ~~~{.cpp}
/// const std::size_t N = 1000;
/// std::mt19937 rng;
/// std::uniform_real_distribution<double> runif;
/// std::vector<double> u01(N);
/// for (std::size_t i = 0; i != N; ++i)
///     u01[i] = runif(rng);
/// std::sort(u01.begin(), u01.end());
/// for (std::size_t i = 0; i != N; ++i)
///     do_something_with_u01(u01[i]);
/// ~~~
/// In the above example, one want N uniform random variates, and having them
/// sorted. The sorted sequence is accessed sequentially. There are two
/// undesired effects. First the program has a runtime cost \f$O(N\log N)\f$.
/// Second, it has a memory cost \f$O(N)\f$. The presented class can be used
/// in
/// place of the temporary vector in the following fashion.
/// ~~~{.cpp}
/// const std::size_t N = 1000;
/// std::mt19937 rng;
/// U01SequenceSorted<std::mt19937> u01(N, rng);
/// for (std::size_t i = 0; i != N; ++i)
///     do_something_with_u01(u01[i]);
/// ~~~
/// The above program will has a runtime cost \f$O(N)\f$ and a memory cost
/// \f$O(1)\f$. However, the overloaded `operator[]` has a few restrictions.
/// Most importantly, it is a forward one-pass operation. More specifically,
/// let `n` be the calling argument and `nlast` be the calling argument of the
/// last time it is called.
/// - `n` can only be either `0`, `nlast`, or `nlast + 1`
/// - `n` can be at most `N - 1`.
template <typename RNGType, typename RealType = double>
class U01SequenceSorted
{
    public:
    using result_type = RealType;

    U01SequenceSorted(std::size_t N, RNGType &rng)
        : N_(N), n_(N), u_(0), lmax_(0), rng_(rng)
    {
    }

    result_type operator[](std::size_t n)
    {
        VSMC_RUNTIME_ASSERT_RNG_U01_SEQUENCE(Sorted)

        if (n == n_)
            return u_;

        lmax_ += std::log(runif_(rng_)) / (N_ - n);
        n_ = n;
        u_ = 1 - std::exp(lmax_);

        return u_;
    }

    result_type operator()(std::size_t n) { return operator[](n); }

    private:
    std::size_t N_;
    std::size_t n_;
    result_type u_;
    result_type lmax_;
    RNGType &rng_;
    U01Distribution<double> runif_;
}; // U01SequenceSorted

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by stratified sampling.
/// \ingroup U01Sequence
///
/// \details
/// This is similar to U01SequenceSorted except that, instead of generating
/// the
/// sequence as if by sorting. It is done by generating
/// \f$u_i = U_i / N + (i - 1) / N\f$ where \f$U_i\f$ is uniform \f$[0,1)\f$
/// random variates.
template <typename RNGType, typename RealType = double>
class U01SequenceStratified
{
    public:
    using result_type = RealType;

    U01SequenceStratified(std::size_t N, RNGType &rng)
        : N_(N)
        , n_(N)
        , u_(0)
        , delta_(1 / static_cast<result_type>(N))
        , rng_(rng)
    {
    }

    result_type operator[](std::size_t n)
    {
        VSMC_RUNTIME_ASSERT_RNG_U01_SEQUENCE(Stratified)

        if (n == n_)
            return u_;

        n_ = n;
        u_ = runif_(rng_) * delta_ + n * delta_;

        return u_;
    }

    result_type operator()(std::size_t n) { return operator[](n); }

    private:
    std::size_t N_;
    std::size_t n_;
    result_type u_;
    result_type delta_;
    RNGType &rng_;
    U01Distribution<double> runif_;
}; // class U01SequenceStratified

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by systematic sampling.
/// \ingroup U01Sequence
///
/// \details
/// This is similar to U01SequenceSorted except that, instead of generating
/// the
/// sequence as if by sorting. It is done by generating
/// \f$u_i = U / N + (i - 1) / N\f$ where \f$U\f$ is uniform \f$[0,1)\f$
/// random variates and it is generated only once for all \f$i\f$.
template <typename RNGType, typename RealType = double>
class U01SequenceSystematic
{
    public:
    using result_type = RealType;

    U01SequenceSystematic(std::size_t N, RNGType &rng)
        : N_(N), n_(N), u_(0), u0_(0), delta_(1 / static_cast<result_type>(N))
    {
        U01Distribution<double> runif;
        u0_ = runif(rng) * delta_;
    }

    result_type operator[](std::size_t n)
    {
        VSMC_RUNTIME_ASSERT_RNG_U01_SEQUENCE(Systematic)

        if (n == n_)
            return u_;

        n_ = n;
        u_ = u0_ + n * delta_;

        return u_;
    }

    result_type operator()(std::size_t n) { return operator[](n); }

    private:
    std::size_t N_;
    std::size_t n_;
    result_type u_;
    result_type u0_;
    result_type delta_;
}; // class U01SequenceSystematic

} // namespace vsmc

#endif // VSMC_RNG_U01_SEQUENCE_HPP
