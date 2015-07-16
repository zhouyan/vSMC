//============================================================================
// vSMC/include/vsmc/rng/uniform_bits_distribution.hpp
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

#ifndef VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP
#define VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>

namespace vsmc
{

namespace internal
{

template <typename UIntType, int Bits>
class UniformBits
{
    public:
    template <typename RNGType>
    static UIntType eval(RNGType &rng)
    {
        return eval(rng,
            std::integral_constant<bool, RNGMinBits<RNGType>::value == 0>(),
            std::integral_constant<bool, RNGBits<RNGType>::value >= Bits>());
    }

    private:
    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::true_type, std::true_type)
    {
        return static_cast<UIntType>(rng());
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::false_type, std::true_type)
    {
        return static_cast<UIntType>(rng() >> RNGMinBits<RNGType>::value);
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::true_type, std::false_type)
    {
        return static_cast<UIntType>(
            patch<0, RNGBits<RNGType>::value, RNGMinBits<RNGType>::value>(
                rng, std::true_type()));
    }

    template <typename RNGType>
    static UIntType eval(RNGType &rng, std::false_type, std::false_type)
    {
        return eval(rng, std::true_type(), std::false_type());
    }

    template <int, int, int, typename RNGType>
    static UIntType patch(RNGType &, std::false_type)
    {
        return 0;
    }

    template <int N, int B, int R, typename RNGType>
    static UIntType patch(RNGType &rng, std::true_type)
    {
        return static_cast<UIntType>((rng() >> R) << (B * N)) +
            patch<N + 1, B, R>(
                   rng, std::integral_constant<bool, (N * B + B) < Bits>());
    }
}; // class UniformBits

} // namespace vsmc::internal

/// \brief Uniform bits distribution
/// \ingroup Distribution
template <typename UIntType>
class UniformBitsDistribution
{
    public:
    using result_type = UIntType;
    using distribution_type = UniformBitsDistribution<UIntType>;

    class param_type
    {
        public:
        using result_type = UIntType;
        using distribution_type = UniformBitsDistribution<UIntType>;

        friend bool operator==(const param_type &, const param_type &)
        {
            return true;
        }

        friend bool operator!=(const param_type &, const param_type &)
        {
            return false;
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &)
        {
            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &)
        {
            return is;
        }
    }; // class param_type

    UniformBitsDistribution() {}
    explicit UniformBitsDistribution(const param_type &) {}

    result_type min() const { return std::numeric_limits<UIntType>::min(); }
    result_type max() const { return std::numeric_limits<UIntType>::max(); }

    void reset() {}

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return internal::UniformBits<UIntType,
            internal::IntBits<UIntType>::value>::eval(rng);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const param_type &)
    {
        return operator()(rng);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        uniform_bits_distribution(rng, n, r);
    }

    template <typename RNGType>
    void operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &)
    {
        uniform_bits_distribution(rng, n, r);
    }

    friend bool operator==(const UniformBitsDistribution<UIntType> &,
        const UniformBitsDistribution<UIntType> &)
    {
        return true;
    }

    friend bool operator!=(const UniformBitsDistribution<UIntType> &,
        const UniformBitsDistribution<UIntType> &)
    {
        return false;
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const UniformBitsDistribution<UIntType> &)
    {
        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is,
        UniformBitsDistribution<UIntType> &)
    {
        return is;
    }
}; // class UniformBitsDistribution

namespace internal
{

template <typename UIntType, typename RNGType, bool B1, bool B2>
inline void uniform_bits_distribution_impl(RNGType &rng, std::size_t n,
    UIntType *r, std::false_type, std::integral_constant<bool, B1>,
    std::integral_constant<bool, B2>)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = UniformBits<UIntType, IntBits<UIntType>::value>::eval(rng);
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(RNGType &rng, std::size_t n,
    UIntType *r, std::true_type, std::true_type, std::true_type)
{
    rng_rand(rng, n, reinterpret_cast<typename RNGType::result_type *>(r));
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(RNGType &rng, std::size_t n,
    UIntType *r, std::true_type, std::true_type, std::false_type)
{
    const std::size_t k =
        sizeof(typename RNGType::result_type) / sizeof(UIntType);
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    rng_rand(rng, m, reinterpret_cast<typename RNGType::result_type *>(r));
    n -= m * k;
    r += m * k;
    for (std::size_t i = 0; i != l; ++i)
        r[i] = UniformBits<UIntType, IntBits<UIntType>::value>::eval(rng);
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(RNGType &rng, std::size_t n,
    UIntType *r, std::true_type, std::false_type, std::true_type)
{
    const std::size_t k =
        sizeof(UIntType) / sizeof(typename RNGType::result_type);
    const std::size_t m = n * k;
    rng_rand(rng, m, reinterpret_cast<typename RNGType::result_type *>(r));
}

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution_impl(RNGType &rng, std::size_t n,
    UIntType *r, std::true_type, std::false_type, std::false_type)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = UniformBits<UIntType, IntBits<UIntType>::value>::eval(rng);
}

} // namespace vsmc::rng_type

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution(RNGType &rng, std::size_t n, UIntType *r)
{
    const int mbits = internal::RNGMinBits<RNGType>::value;
    const int rbits = internal::RNGBits<RNGType>::value;
    const int ubits = internal::IntBits<UIntType>::value;
    internal::uniform_bits_distribution_impl(rng, n, r,
        std::integral_constant<bool, mbits == 0>(),
        std::integral_constant < bool,
        rbits >= ubits && rbits % ubits == 0 > (),
        std::integral_constant < bool,
        ubits >= rbits && ubits % rbits == 0 > ());
}

template <typename UIntType, typename RNGType>
inline void rng_rand(RNGType &rng, UniformBitsDistribution<UIntType> &dist,
    std::size_t n, UIntType *r)
{
    dist(rng, n, r);
}

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_BITS_DISTRIBUTION_HPP
