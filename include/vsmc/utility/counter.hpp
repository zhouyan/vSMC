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

namespace vsmc
{

namespace internal
{

template <typename T, bool>
struct CounterMask;

template <typename T>
struct CounterMask<T, true> {
    static constexpr T max_val = VSMC_MAX_UINT(T);

    static constexpr T mask_hi =
        static_cast<T>(static_cast<T>(max_val << 8) >> 8);

    static constexpr T mask_lo = static_cast<T>(mask_hi ^ max_val);
}; // struct CounterMask

} // namespace vsmc::internal

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
/// `sizeof(T) * K * 8` bits integer. For example, `std::array<uint32_t, 4>`
/// is
/// treated as an 128-bits integer. The counter start with all elements being
/// zero. The value of the integer can be calculated as, \f$c_0 + c_1 M + c_2
/// M^2 +\cdots + c_{K-1} M^{K - 1}\f$, where \f$c_i\f$ is the \f$i\f$th
/// element in the `std::array` and has a range from zero to \f$M - 1\f$,
/// \f$M\f$ is
/// the largest number of type `T` plus one, that is \f$2^n\f$ with \f$n\f$
/// being the number of bits in type `T`.
///
/// The second type is blocks of counters of the first type. For example,
/// `std::array<ctr_type, Blocks>`, where `ctr_type` is a counter of the first
/// type.
/// When set and incremented using methods in this class, all `Blocks`
/// counters
/// are maintained such that, they are always distinctive. This is done by
/// reserving eight bits as counter IDs. Therefore, there can be at most 256
/// blocks. The increment works by incrementing each counter the same way as
/// in
/// the first type, except that the last element, \f$c_{K-1}\f$ has a range
/// from zero to \f$2^{n - 8} - 1\f$ where \f$n\f$ is the number of bits in
/// type `T`. Therefore, in the extreme case where `ctr_type` is
/// `std::array<uint8_t, 1>`, increment won't change the counter at all.
template <typename T, std::size_t K>
class Counter<std::array<T, K>>
{
    public:
    typedef std::array<T, K> ctr_type;

    /// \brief Set the counter to a given value
    static inline void set(ctr_type &ctr, const ctr_type &c) { ctr = c; }

    /// \brief Set a block of counters given the value of the first counter
    template <std::size_t Blocks>
    static inline void set(
        std::array<ctr_type, Blocks> &ctr, const ctr_type &c)
    {
        ctr.front() = c;
        set_block<1>(ctr, std::integral_constant<bool, 1 < Blocks>());
    }

    /// \brief Reset a counter to zero
    static inline void reset(ctr_type &ctr)
    {
        std::memset(ctr.data(), 0, sizeof(T) * K);
    }

    /// \brief Reset a block of counters with the first set to zero
    template <std::size_t Blocks>
    static inline void reset(std::array<ctr_type, Blocks> &ctr)
    {
        reset(ctr.front());
        set_block<1>(ctr, std::integral_constant<bool, 1 < Blocks>());
    }

    /// \brief Increment the counter by one
    static inline void increment(ctr_type &ctr)
    {
        increment_single<0>(ctr, std::integral_constant<bool, 1 < K>());
    }

    /// \brief Increment each counter in a block by one
    template <std::size_t Blocks>
    static inline void increment(std::array<ctr_type, Blocks> &ctr)
    {
        increment_block<0>(ctr, std::true_type());
    }

    /// \brief Increment a counter by a given value
    static inline void increment(ctr_type &ctr, T nskip)
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

    /// \brief Increment each counter in a block by a given value
    template <std::size_t Blocks>
    static inline void increment(std::array<ctr_type, Blocks> &ctr, T nskip)
    {
        if (nskip == 0)
            return;

        if (nskip == 1) {
            increment(ctr);
            return;
        }

        increment_block<0>(ctr, nskip, std::true_type());
    }

    private:
    static constexpr T max_ =
        internal::CounterMask<T, std::is_unsigned<T>::value>::max_val;

    static constexpr T mask_hi_ =
        internal::CounterMask<T, std::is_unsigned<T>::value>::mask_hi;

    static constexpr T mask_lo_ =
        internal::CounterMask<T, std::is_unsigned<T>::value>::mask_lo;

    template <std::size_t>
    static inline void increment_single(ctr_type &ctr, std::false_type)
    {
        ++ctr.back();
    }

    template <std::size_t N>
    static inline void increment_single(ctr_type &ctr, std::true_type)
    {
        if (++std::get<N>(ctr) != 0)
            return;

        increment_single<N + 1>(
            ctr, std::integral_constant<bool, N + 2 < K>());
    }

    template <std::size_t, std::size_t Blocks>
    static inline void set_block(
        std::array<ctr_type, Blocks> &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t Blocks>
    static inline void set_block(
        std::array<ctr_type, Blocks> &ctr, std::true_type)
    {
        T m = std::get<B - 1>(ctr).back() & mask_lo_;
        m >>= sizeof(T) * 8 - 8;
        ++m;
        m <<= sizeof(T) * 8 - 8;

        std::get<B>(ctr) = std::get<B - 1>(ctr);
        std::get<B>(ctr).back() &= mask_hi_;
        std::get<B>(ctr).back() ^= m;
        set_block<B + 1>(ctr, std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t, std::size_t Blocks>
    static inline void increment_block(
        std::array<ctr_type, Blocks> &, std::false_type)
    {
    }

    template <std::size_t B, std::size_t Blocks>
    static inline void increment_block(
        std::array<ctr_type, Blocks> &ctr, std::true_type)
    {
        increment_block_ctr(std::get<B>(ctr));
        increment_block<B + 1>(
            ctr, std::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t, std::size_t Blocks>
    static inline void increment_block(
        std::array<ctr_type, Blocks> &, T, std::false_type)
    {
    }

    template <std::size_t B, std::size_t Blocks>
    static inline void increment_block(
        std::array<ctr_type, Blocks> &ctr, T nskip, std::true_type)
    {
        increment_block_ctr(std::get<B>(ctr), nskip);
        increment_block<B + 1>(
            ctr, nskip, std::integral_constant<bool, B + 1 < Blocks>());
    }

    static inline void increment_block_ctr(ctr_type &ctr)
    {
        increment_block_single<0>(ctr, std::integral_constant<bool, 1 < K>());
    }

    static inline void increment_block_ctr(ctr_type &ctr, T nskip)
    {
        increment_block_nskip(
            ctr, nskip, std::integral_constant<bool, 1 < K>());
    }

    template <std::size_t>
    static inline void increment_block_single(ctr_type &ctr, std::false_type)
    {
        T m = ctr.back() & mask_lo_;
        ctr.back() <<= 8;
        ctr.back() >>= 8;
        ++ctr.back();
        ctr.back() &= mask_hi_;
        ctr.back() ^= m;
    }

    template <std::size_t N>
    static inline void increment_block_single(ctr_type &ctr, std::true_type)
    {
        if (++std::get<N>(ctr) != 0)
            return;

        increment_block_single<N + 1>(
            ctr, std::integral_constant<bool, N + 2 < K>());
    }

    static inline void increment_block_nskip(
        ctr_type &ctr, T nskip, std::false_type)
    {
        T m = ctr.back() & mask_lo_;
        T b = ctr.back();
        b <<= 8;
        b >>= 8;

        if (nskip <= mask_hi_ - b) {
            b += nskip;
            b &= mask_hi_;
            b ^= m;
            ctr.back() = b;
            return;
        }

        nskip -= mask_hi_ - b - 1;
        while (nskip > mask_hi_)
            nskip -= mask_hi_;
        b = nskip;
        b &= mask_hi_;
        b ^= m;
        ctr.back() = b;
    }

    static inline void increment_block_nskip(
        ctr_type &ctr, T nskip, std::true_type)
    {
        if (nskip == 0)
            return;

        if (nskip == 1) {
            increment_block_ctr(ctr);
            return;
        }

        if (ctr.front() <= max_ - nskip) {
            ctr.front() += nskip;
            return;
        }

        nskip -= max_ - ctr.front();
        ctr.front() = max_;
        increment_block_ctr(ctr);
        ctr.front() = nskip - 1;
    }
}; // struct Counter

} // namespace vsmc

#endif // VSMC_UTILITY_COUNTER_HPP
