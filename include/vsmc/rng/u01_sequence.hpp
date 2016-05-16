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

namespace internal
{

template <std::size_t K, typename RealType>
inline void u01_trans_sorted_impl(std::size_t n0, std::size_t n,
    const RealType *u01, RealType *r, std::size_t N, RealType &lmax)
{
    if (n0 == n)
        return;

    Array<RealType, K> s;
    std::size_t j = 0;
    std::size_t m = N - n0;
    log(n - n0, u01, r);
    for (std::size_t i = n0; i != n; ++i, ++j, --m) {
        lmax += r[j] / m;
        s[j] = lmax;
    }
    exp(n - n0, s.data(), s.data());
    sub(n - n0, const_one<RealType>(), s.data(), r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_rand_sorted_impl(RNGType &rng, std::size_t n0, std::size_t n,
    RealType *r, std::size_t N, RealType &lmax)
{
    if (n0 == n)
        return;

    u01_distribution(rng, n - n0, r);
    u01_trans_sorted_impl<K>(n0, n, r, r, N, lmax);
}

template <typename RealType>
inline void u01_trans_stratified_impl(std::size_t n0, std::size_t n,
    const RealType *u01, RealType *r, RealType delta)
{
    if (n0 == n)
        return;

    std::size_t j = 0;
    for (std::size_t i = n0; i != n; ++i, ++j)
        r[j] = u01[j] + static_cast<RealType>(i);
    mul(n - n0, delta, r, r);
}

template <typename RealType, typename RNGType>
inline void u01_rand_stratified_impl(
    RNGType &rng, std::size_t n0, std::size_t n, RealType *r, RealType delta)
{
    if (n0 == n)
        return;

    u01_distribution(rng, n - n0, r);
    u01_trans_stratified_impl(n0, n, r, r, delta);
}

template <typename RealType>
inline void u01_trans_systematic_impl(
    std::size_t n0, std::size_t n, RealType u, RealType *r, RealType delta)
{
    if (n0 == n)
        return;

    std::size_t j = 0;
    for (std::size_t i = n0; i != n; ++i, ++j)
        r[j] = static_cast<RealType>(i);
    fma(n - n0, r, delta, u, r);
}

} // namespace vsmc::internal

/// \brief Tranform a sequence of standard uniform random numbers to sorted
/// sequence
/// \ingroup U01Sequence
template <typename RealType>
inline void u01_trans_sorted(std::size_t N, const RealType *u01, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_trans_sorted** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    if (N == 0)
        return;

    const std::size_t k = internal::BufferSize<RealType>::value;
    if (N <= k) {
        if (u01 != r)
            std::copy_n(u01, N, r);
        std::sort(r, r + N);
        return;
    }

    const std::size_t m = N / k;
    std::size_t n0 = 0;
    RealType lmax = 0;
    for (std::size_t i = 0; i != m; ++i, n0 += k, u01 += k, r += k)
        internal::u01_trans_sorted_impl<k>(n0, n0 + k, u01, r, N, lmax);
    internal::u01_trans_sorted_impl<k>(n0, N, u01, r, N, lmax);
}

/// \brief Transform a sequence of standard uniform random numbers to a
/// stratified sequence
/// \ingroup U01Sequence
template <typename RealType>
inline void u01_trans_stratified(
    std::size_t N, const RealType *u01, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_trans_stratified** USED WITH RealType OTHER THAN FLOATING "
        "POINT TYPES");

    if (N == 0)
        return;

    const std::size_t k = internal::BufferSize<RealType>::value;
    const std::size_t m = N / k;
    std::size_t n0 = 0;
    const RealType delta = 1 / static_cast<RealType>(N);
    for (std::size_t i = 0; i != m; ++i, n0 += k, u01 += k, r += k)
        internal::u01_trans_stratified_impl(n0, n0 + k, u01, r, delta);
    internal::u01_trans_stratified_impl(n0, N, u01, r, delta);
}

/// \brief Transform a single standard uniform random number to a systematic
/// sequence
/// \ingroup U01Sequence
template <typename RealType>
inline void u01_trans_systematic(
    std::size_t N, const RealType *u01, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_trans_systematic** USED WITH RealType OTHER THAN FLOATING "
        "POINT TYPES");

    if (N == 0)
        return;

    const std::size_t k = internal::BufferSize<RealType>::value;
    const std::size_t m = N / k;
    std::size_t n0 = 0;
    const RealType delta = 1 / static_cast<RealType>(N);
    const RealType u = u01[0] * delta;
    for (std::size_t i = 0; i != m; ++i, n0 += k, r += k)
        internal::u01_trans_systematic_impl(n0, n0 + k, u, r, delta);
    internal::u01_trans_systematic_impl(n0, N, u, r, delta);
}

/// \brief Generate sorted standard uniform numbers with \f$O(N)\f$ cost
/// \ingroup U01Sequence
template <typename RealType, typename RNGType>
inline void u01_rand_sorted(RNGType &rng, std::size_t N, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_rand_sorted** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    if (N == 0)
        return;

    const std::size_t k = internal::BufferSize<RealType>::value;
    if (N <= k) {
        u01_distribution(rng, N, r);
        std::sort(r, r + N);
        return;
    }

    const std::size_t m = N / k;
    std::size_t n0 = 0;
    RealType lmax = 0;
    for (std::size_t i = 0; i != m; ++i, n0 += k, r += k)
        internal::u01_rand_sorted_impl<k>(rng, n0, n0 + k, r, N, lmax);
    internal::u01_rand_sorted_impl<k>(rng, n0, N, r, N, lmax);
}

/// \brief Generate stratified standard uniform numbers
/// \ingroup U01Sequence
template <typename RealType, typename RNGType>
inline void u01_rand_stratified(RNGType &rng, std::size_t N, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_rand_stratified** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = internal::BufferSize<RealType>::value;
    const std::size_t m = N / k;
    std::size_t n0 = 0;
    const RealType delta = 1 / static_cast<RealType>(N);
    for (std::size_t i = 0; i != m; ++i, n0 += k, r += k)
        internal::u01_rand_stratified_impl(rng, n0, n0 + k, r, delta);
    internal::u01_rand_stratified_impl(rng, n0, N, r, delta);
}

/// \brief Generate systematic standard uniform numbers
/// \ingroup U01Sequence
template <typename RealType, typename RNGType>
inline void u01_rand_systematic(RNGType &rng, std::size_t N, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_rand_systematic** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    U01Distribution<RealType> ru01;
    RealType u01 = ru01(rng);
    u01_trans_systematic(N, &u01, r);
}

/// \brief Sorted of standard uniform numbers
/// \ingroup U01Sequence
class U01SequenceSorted
{
    public:
    template <typename RealType>
    void operator()(std::size_t N, const RealType *u01, RealType *r) const
    {
        u01_trans_sorted(N, u01, r);
    }

    template <typename RealType, typename RNG>
    void operator()(RNG &rng, std::size_t N, RealType *r) const
    {
        u01_rand_sorted(rng, N, r);
    }
}; // U01SequenceSorted

/// \brief Stratified standard uniform numbers
/// \ingroup U01Sequence
class U01SequenceStratified
{
    public:
    template <typename RealType>
    void operator()(std::size_t N, const RealType *u01, RealType *r) const
    {
        u01_trans_stratified(N, u01, r);
    }

    template <typename RealType, typename RNG>
    void operator()(RNG &rng, std::size_t N, RealType *r) const
    {
        u01_rand_stratified(rng, N, r);
    }
}; // class U01SequenceStratified

/// \brief Systematic standard uniform numbers
/// \ingroup U01Sequence
class U01SequenceSystematic
{
    public:
    template <typename RealType>
    void operator()(std::size_t N, const RealType *u01, RealType *r) const
    {
        u01_trans_systematic(N, u01, r);
    }

    template <typename RealType, typename RNG>
    void operator()(RNG &rng, std::size_t N, RealType *r) const
    {
        u01_rand_systematic(rng, N, r);
    }
}; // class U01SequenceSystematic

} // namespace vsmc

#endif // VSMC_RNG_U01_SEQUENCE_HPP
