//============================================================================
// vSMC/include/vsmc/rng/u01_distribution.hpp
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

#ifndef VSMC_RNG_U01_DISTRIBUTION_HPP
#define VSMC_RNG_U01_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.hpp>
#include <vsmc/rng/uniform_bits_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RNGType>
using U01UIntType = typename std::conditional<(RNGBits<RNGType>::value >= 64),
    std::uint64_t, std::uint32_t>::type;

} // namespace vsmc::internal

/// \brief Standard uniform distribution
/// \ingroup Distribution
template <typename RealType>
class U01Distribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        U01, u01, RealType, floating_point, FLOATING_POINT)

    public:
    result_type min() const { return 0; }

    result_type max() const { return 1; }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return generate(rng,
            std::integral_constant<bool, RNGBits<RNGType>::value >= 32>());
    }

    template <typename RNGType>
    result_type generate(RNGType &rng, std::true_type)
    {
        return u01<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }

    template <typename RNGType>
    result_type generate(RNGType &rng, std::false_type)
    {
        std::uniform_real_distribution<RealType> u01;

        return u01(rng);
    }
}; // class U01Distribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = u01<U01UIntType<RNGType>, RealType>(s[i]);
}

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_distribution_impl<k, RealType>(rng, k, r);
    internal::u01_distribution_impl<k, RealType>(rng, l, r);
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01, u01, RealType)

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
