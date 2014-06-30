//============================================================================
// vsmc/utility/counter.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_COUNTER_HPP
#define VSMC_UTILITY_COUNTER_HPP

#include <vsmc/utility/array.hpp>

namespace  vsmc {

namespace internal {

template <typename T, bool> struct CounterMask;

template <typename T>
struct CounterMask<T, true>
{
    static VSMC_CONSTEXPR const T max_val =
        static_cast<T>(~(static_cast<T>(0)));

    static VSMC_CONSTEXPR const T mask_hi =
        static_cast<T>(static_cast<T>(max_val << 8) >> 8);

    static VSMC_CONSTEXPR const T mask_lo =
        static_cast<T>(mask_hi^max_val);
}; // struct CounterMask

} // namespace vsmc::internal

template <typename> class Counter;

/// \brief Using Array of unsigned integers as counters
/// \ingroup Counter
///
/// \details
/// This class provides methods for using Array of unsigned integers as
/// large integer counters.
///
/// It deals with two types of counters. The first type is in the form
/// `Array<T, K, Traits>` where `T` is an unsigned integer type, treated
/// as a `sizeof(T) * K * 8` bits integer. For example,
/// `Array<uint32_t, 4>` is treated as an 128-bits integer. The counter
/// start with all elements being zero. The value of the integer can be
/// calculated as, \f$c_0 + c_1 M + c_2 M^2 +\cdots + c_{K-1} M^{K - 1}\f$,
/// where \f$c_i\f$ is the \f$i\f$th element in the Array and has a
/// range from zero to \f$M - 1\f$, \f$M\f$ is the largest number of type `T`
/// plus one, that is \f$2^n\f$ with \f$n\f$ being the number of bits in type
/// `T`.
///
/// The second type is blocks of counters of the first type. For example,
/// `Array<ctr_type, Blocks, BlockTraits>`, where `ctr_type` is a
/// counter of the first type. When set and incremented using methods in this
/// class, all `Blocks` counters are maintained such that, they are always
/// distinctive. This is done by reserving eight bits as counter IDs.
/// Therefore, there can be at most 256 blocks. The eight bits reserved are the
/// 8 higher bits in the last element of each counter. In a little-endian
/// representation, such as on x86, the last bytes in the memory of counter is
/// reserved. The increment works by increment each counter the same way as in
/// the first type, except that the last element, \f$c_{K-1}\f$ has a range
/// from zero to \f$2^{n - 8} - 1\f$ where \f$n\f$ is the number of bits in
/// type `T`. Therefore, in the extreme case where `ctr_type` is
/// `Array<uint8_t, 1>`, increment won't change the counter at all.
template <typename T, std::size_t K, typename Traits>
class Counter<Array<T, K, Traits> >
{
    public :

    typedef Array<T, K, Traits> ctr_type;

    /// \brief Set the counter to a given value
    static void set (ctr_type &ctr, const ctr_type &c) {ctr = c;}

    /// \brief Set a block of counters given the value of the first counter
    template <std::size_t Blocks, typename BlockTraits>
    static void set (Array<ctr_type, Blocks, BlockTraits> &ctr,
            const ctr_type &c)
    {
        ctr.front() = c;
        set_block<1>(ctr, cxx11::integral_constant<bool, 1 < Blocks>());
    }

    /// \brief Reset a counter to zero
    static void reset (ctr_type &ctr)
    {std::memset(static_cast<void *>(ctr.data()), 0, sizeof(T) * K);}

    /// \brief Reset a block of counters with the first set to zero
    template <std::size_t Blocks, typename BlockTraits>
    static void reset (Array<ctr_type, Blocks, BlockTraits> &ctr)
    {
        reset(ctr.front());
        set_block<1>(ctr, cxx11::integral_constant<bool, 1 < Blocks>());
    }

    /// \brief Increment the counter by one
    static void increment (ctr_type &ctr)
    {increment_single<0>(ctr, cxx11::integral_constant<bool, 1 < K>());}

    /// \brief Increment each counter in a block by one
    template <std::size_t Blocks, typename BlockTraits>
    static void increment (Array<ctr_type, Blocks, BlockTraits> &ctr)
    {increment_block<0>(ctr, cxx11::true_type());}

    /// \brief Increment a counter by a given value
    static void increment (ctr_type &ctr, T nskip)
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
    template <std::size_t Blocks, typename BlockTraits>
    static void increment (Array<ctr_type, Blocks, BlockTraits> &ctr,
            T nskip)
    {
        if (nskip == 0)
            return;

        if (nskip == 1) {
            increment(ctr);
            return;
        }

        increment_block<0>(ctr, nskip, cxx11::true_type());
    }

    private :

    static VSMC_CONSTEXPR const T max_ =
        internal::CounterMask<T, cxx11::is_unsigned<T>::value>::max_val;

    static VSMC_CONSTEXPR const T mask_hi_ =
        internal::CounterMask<T, cxx11::is_unsigned<T>::value>::mask_hi;

    static VSMC_CONSTEXPR const T mask_lo_ =
        internal::CounterMask<T, cxx11::is_unsigned<T>::value>::mask_lo;

    template <std::size_t>
    static void increment_single (ctr_type &ctr, cxx11::false_type)
    {++ctr.back();}

    template <std::size_t N>
    static void increment_single (ctr_type &ctr, cxx11::true_type)
    {
        if (++ctr[Position<N>()] != 0)
            return;

        increment_single<N + 1>(ctr,
                cxx11::integral_constant<bool, N + 2 < K>());
    }

    template <std::size_t, std::size_t Blocks, typename BlockTraits>
    static void set_block (Array<ctr_type, Blocks, BlockTraits> &,
            cxx11::false_type) {}

    template <std::size_t B, std::size_t Blocks, typename BlockTraits>
    static void set_block (Array<ctr_type, Blocks, BlockTraits> &ctr,
            cxx11::true_type)
    {
        T m = ctr[Position<B - 1>()].back() & mask_lo_;
        m >>= sizeof(T) * 8 - 8;
        ++m;
        m <<= sizeof(T) * 8 - 8;

        ctr[Position<B>()] = ctr[Position<B - 1>()];
        ctr[Position<B>()].back() &= mask_hi_;
        ctr[Position<B>()].back() ^= m;
        set_block<B + 1>(ctr,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t, std::size_t Blocks, typename BlockTraits>
    static void increment_block (
            Array<ctr_type, Blocks, BlockTraits> &,
            cxx11::false_type) {}

    template <std::size_t B, std::size_t Blocks, typename BlockTraits>
    static void increment_block (
            Array<ctr_type, Blocks, BlockTraits> &ctr,
            cxx11::true_type)
    {
        increment_block_ctr(ctr[Position<B>()]);
        increment_block<B + 1>(ctr,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    template <std::size_t, std::size_t Blocks, typename BlockTraits>
    static void increment_block (
            Array<ctr_type, Blocks, BlockTraits> &,
            T, cxx11::false_type) {}

    template <std::size_t B, std::size_t Blocks, typename BlockTraits>
    static void increment_block (
            Array<ctr_type, Blocks, BlockTraits> &ctr,
            T nskip, cxx11::true_type)
    {
        increment_block_ctr(ctr[Position<B>()], nskip);
        increment_block<B + 1>(ctr, nskip,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    static void increment_block_ctr (ctr_type &ctr)
    {increment_block_single<0>(ctr, cxx11::integral_constant<bool, 1 < K>());}

    static void increment_block_ctr (ctr_type &ctr, T nskip)
    {
        increment_block_nskip(ctr, nskip,
                cxx11::integral_constant<bool, 1 < K>());
    }

    template <std::size_t>
    static void increment_block_single (ctr_type &ctr, cxx11::false_type)
    {
        T m = ctr.back() & mask_lo_;
        ctr.back() <<= 8;
        ctr.back() >>= 8;
        ++ctr.back();
        ctr.back() &= mask_hi_;
        ctr.back() ^= m;
    }

    template <std::size_t N>
    static void increment_block_single (ctr_type &ctr, cxx11::true_type)
    {
        if (++ctr[Position<N>()] != 0)
            return;

        increment_block_single<N + 1>(ctr,
                cxx11::integral_constant<bool, N + 2 < K>());
    }

    static void increment_block_nskip (ctr_type &ctr, T nskip,
            cxx11::false_type)
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

    static void increment_block_nskip (ctr_type &ctr, T nskip,
            cxx11::true_type)
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
