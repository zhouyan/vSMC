//============================================================================
// vSMC/include/vsmc/utility/counter.hpp
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

#ifndef VSMC_UTILITY_COUNTER_HPP
#define VSMC_UTILITY_COUNTER_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_STATIC_ASSERT_UTILITY_COUNTER_SET_CTR(dst, src)                  \
    VSMC_STATIC_ASSERT((sizeof(dst) <= sizeof(src)),                          \
        "**Counter::set** SIZE OF SOURCE TOO SMALL")

namespace vsmc
{

template <typename>
class Counter;

/// \brief Using `std::array` of unsigned integers as counters
/// \ingroup Counter
///
/// \details
/// This class provides methods for using `std::array` of unsigned integers as
/// large integer counters.
///
/// It deals with two types of counters. The first type is in the form
/// `std::array<T, K>` where `T` is an unsigned integer type, treated as a
/// `sizeof(T) * K * 8` bits integer. For example, `std::array<std::uint32_t,
/// 4>` is treated as an 128-bits integer. The counter start with all elements
/// being zero. The value of the integer can be calculated as, \f$c_0 + c_1 M +
/// c_2 M^2 +\cdots + c_{K-1} M^{K - 1}\f$, where \f$c_i\f$ is the \f$i\f$th
/// element in the `std::array` and has a range from zero to \f$M - 1\f$,
/// \f$M\f$ is the largest number of type `T` plus one, that is \f$2^n\f$ with
/// \f$n\f$ being the number of bits in type `T`.
template <typename T, std::size_t K>
class Counter<std::array<T, K>>
{
    public:
    typedef std::array<T, K> ctr_type;

    /// \brief Set the counter to a given value
    static void set(ctr_type &ctr, const ctr_type &c) { ctr = c; }

    template <typename U>
    static void set(ctr_type &ctr, const U &c)
    {
        VSMC_STATIC_ASSERT_UTILITY_COUNTER_SET_CTR(ctr_type, U);
        std::memcpy(ctr.data(), &c, sizeof(ctr_type));
    }

    /// \brief Reset a counter to zero
    static void reset(ctr_type &ctr)
    {
        std::memset(ctr.data(), 0, sizeof(T) * K);
    }

    /// \brief Increment the counter by one
    static void increment(ctr_type &ctr)
    {
        increment_single<0>(ctr, std::integral_constant<bool, 1 < K>());
    }

    /// \brief Increment the counter multiple times and store the results
    template <std::size_t Blocks>
    static void increment(
        ctr_type &ctr, std::array<ctr_type, Blocks> &ctr_blocks)
    {
        increment_block<Blocks, 0>(
            ctr, ctr_blocks, std::integral_constant<bool, 0 < Blocks>());
    }

    /// \brief Increment a counter by a given value
    static void increment(ctr_type &ctr, T nskip)
    {
        if (nskip == 0)
            return;

        if (K == 1) {
            ctr.front() += nskip;
            return;
        }

        if (nskip == 1) {
            increment(ctr);
            return;
        }

        if (ctr.front() <= max_ - nskip) {
            ctr.front() += nskip;
            return;
        }

        nskip -= max_ - ctr.front();
        ctr.front() = max_;
        increment(ctr);
        ctr.front() = nskip - 1;
    }

    private:
    static constexpr T max_ = VSMC_MAX_UINT(T);

    template <std::size_t>
    static void increment_single(ctr_type &ctr, std::false_type)
    {
        ++ctr.back();
    }

    template <std::size_t N>
    static void increment_single(ctr_type &ctr, std::true_type)
    {
        if (++std::get<N>(ctr) != 0)
            return;

        increment_single<N + 1>(
            ctr, std::integral_constant<bool, N + 2 < K>());
    }

    template <std::size_t Blocks, std::size_t>
    static void increment_block(
        ctr_type &, std::array<ctr_type, Blocks> &, std::false_type)
    {
    }

    template <std::size_t Blocks, std::size_t B>
    static void increment_block(ctr_type &ctr,
        std::array<ctr_type, Blocks> &ctr_blocks, std::true_type)
    {
        increment(ctr);
        std::get<B>(ctr_blocks) = ctr;
        increment_block<Blocks, B + 1>(
            ctr, ctr_blocks, std::integral_constant<bool, B + 1 < Blocks>());
    }
}; // struct Counter

} // namespace vsmc

#endif // VSMC_UTILITY_COUNTER_HPP
