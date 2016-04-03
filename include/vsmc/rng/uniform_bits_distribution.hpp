//============================================================================
// vSMC/include/vsmc/rng/uniform_bits_distribution.hpp
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

#ifndef VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP
#define VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>

namespace vsmc
{

/// \brief Generate uniform bits of given type
/// \ingroup RNG
///
/// \details
/// For a given unsigned integer type `UIntType` with \f$W\f$ bits, the output
/// will be unsigned integers on the set \f$\{0,\dots,2^W - 1\}\f$ regardless
/// of the range of the input `RNGType`
template <typename UIntType>
class UniformBits
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**UniformBits** USED WITH UIntType OTHER THAN UNSIGNED INTEGER "
        "TYPES");

    public:
    template <typename RNGType>
    static UIntType eval(RNGType &rng)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int p = RNGBits<RNGType>::value;

        return eval(rng, std::integral_constant<bool, w <= p>());
    }

    private:
    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::true_type)
    {
        static constexpr int r = RNGMinBits<RNGType>::value;

        return static_cast<UIntType>(rng() >> r);
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::false_type)
    {
        return patch<0>(rng, std::true_type());
    }

    template <int, typename RNGType>
    static UIntType patch(RNGType &, std::false_type)
    {
        return 0;
    }

    template <int N, typename RNGType>
    static UIntType patch(RNGType &rng, std::true_type)
    {
        static constexpr int w = std::numeric_limits<UIntType>::digits;
        static constexpr int v =
            std::numeric_limits<typename RNGType::result_type>::digits;
        static constexpr int l = v - RNGMaxBits<RNGType>::value;
        static constexpr int r = l + RNGMinBits<RNGType>::value;
        static constexpr int p = N * RNGBits<RNGType>::value;
        static constexpr int q = p + RNGBits<RNGType>::value;

        UIntType u = static_cast<UIntType>((rng() << l) >> r);

        return (u << p) +
            patch<N + 1>(rng, std::integral_constant<bool, (q < w)>());
    }
}; // class UniformBits

/// \brief Uniform bits distribution
/// \ingroup Distribution
///
/// \details
/// For a given unsigned integer type `UIntType` with \f$W\f$ bits, the output
/// will be unsigned integers on the set \f$\{0,\dots,2^W - 1\}\f$ regardless
/// of the range of the input `RNGType`
template <typename UIntType>
class UniformBitsDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        UniformBits, uniform_bits, UIntType, unsigned, UNSIGNED)

    public:
    result_type min() const { return std::numeric_limits<result_type>::min(); }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return UniformBits<UIntType>::eval(rng);
    }
}; // class UniformBitsDistribution

namespace internal
{

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl_neq(
    RNGType &rng, std::size_t n, UIntType *r, std::false_type)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = UniformBits<UIntType>::eval(rng);
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl_neq(
    RNGType &rng, std::size_t n, UIntType *r, std::true_type)
{
    static constexpr std::uintptr_t alignment =
        static_cast<std::uintptr_t>(alignof(typename RNGType::result_type));
    static constexpr std::size_t rbits = RNGBits<RNGType>::value;
    static constexpr std::size_t ubits = std::numeric_limits<UIntType>::digits;
    static constexpr std::size_t k = rbits / ubits;

    while (reinterpret_cast<std::uintptr_t>(r) % alignment != 0) {
        *r = UniformBits<UIntType>::eval(rng);
        ++r;
        --n;
    }
    const std::size_t m = n / k;
    rng_rand(rng, m, reinterpret_cast<typename RNGType::result_type *>(r));
    r += m * k;
    n -= m * k;
    for (std::size_t i = 0; i != n; ++i)
        r[i] = UniformBits<UIntType>::eval(rng);
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(
    RNGType &rng, std::size_t n, UIntType *r, std::false_type)
{
    static constexpr bool zero_min = RNGMinBits<RNGType>::value == 0;
    static constexpr std::size_t rbits = RNGBits<RNGType>::value;
    static constexpr std::size_t ubits = std::numeric_limits<UIntType>::digits;

    uniform_bits_distribution_impl_neq(
        rng, n, r, std::integral_constant<bool,
                       (zero_min && rbits >= ubits && rbits % ubits == 0)>());
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(
    RNGType &rng, std::size_t n, UIntType *r, std::true_type)
{
    rng_rand(rng, n, reinterpret_cast<typename RNGType::result_type *>(r));
}

} // namespace vsmc::internal

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution(RNGType &rng, std::size_t n, UIntType *r)
{
    static_assert(std::is_unsigned<UIntType>::value,
        "**uniform_bits_distribution** USED WITH UIntType OTHER THAN UNSIGNED "
        "TYPES");

    static constexpr bool zero_min = RNGMinBits<RNGType>::value == 0;
    static constexpr bool eq_bits =
        RNGBits<RNGType>::value == std::numeric_limits<UIntType>::digits;
    static constexpr bool eq_size =
        sizeof(typename RNGType::result_type) == sizeof(UIntType);
    static constexpr bool eq_align =
        alignof(typename RNGType::result_type) == alignof(UIntType);

    internal::uniform_bits_distribution_impl(
        rng, n, r, std::integral_constant<bool,
                       (zero_min && eq_bits && eq_size && eq_align)>());
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(UniformBits, uniform_bits, UIntType)

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP
