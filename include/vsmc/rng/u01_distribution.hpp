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
using U01UIntType = typename std::conditional<(RNGBits<RNGType>::value > 32),
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
        return u01_co<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }
}; // class U01Distribution

/// \brief Standard uniform distribution on [0, 1]
/// \ingroup Distribution
template <typename RealType>
class U01CCDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        U01CC, u01_cc, RealType, floating_point, FLOATING_POINT)

    public:
    result_type min() const { return 0; }

    result_type max() const { return 1; }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return u01_cc<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }
}; // class U01CODistribution

/// \brief Standard uniform distribution on [0, 1)
/// \ingroup Distribution
template <typename RealType>
class U01CODistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        U01CO, u01_co, RealType, floating_point, FLOATING_POINT)

    public:
    result_type min() const { return 0; }

    result_type max() const { return 1; }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return u01_co<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }
}; // class U01CODistribution

/// \brief Standard uniform distribution on (0, 1]
/// \ingroup Distribution
template <typename RealType>
class U01OCDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        U01OC, u01_oc, RealType, floating_point, FLOATING_POINT)

    public:
    result_type min() const { return 0; }

    result_type max() const { return 1; }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return u01_oc<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }
}; // class U01CODistribution

/// \brief Standard uniform distribution on (0, 1)
/// \ingroup Distribution
template <typename RealType>
class U01OODistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_0(
        U01OO, u01_oo, RealType, floating_point, FLOATING_POINT)

    public:
    result_type min() const { return 0; }

    result_type max() const { return 1; }

    void reset() {}

    private:
    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &)
    {
        return u01_oo<internal::U01UIntType<RNGType>, result_type>(
            UniformBits<internal::U01UIntType<RNGType>>::eval(rng));
    }
}; // class U01CODistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    u01_co<U01UIntType<RNGType>, RealType>(n, s, r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_cc_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    u01_cc<U01UIntType<RNGType>, RealType>(n, s, r);
    for (std::size_t i = 0; i != n; ++i)
        r[i] = u01_cc<U01UIntType<RNGType>, RealType>(s[i]);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_co_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    u01_co<U01UIntType<RNGType>, RealType>(n, s, r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_oc_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    u01_oc<U01UIntType<RNGType>, RealType>(n, s, r);
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_oo_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    U01UIntType<RNGType> s[K];
    uniform_bits_distribution(rng, n, s);
    u01_oo<U01UIntType<RNGType>, RealType>(n, s, r);
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

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_distribution_impl<k>(rng, k, r);
    internal::u01_distribution_impl<k>(rng, l, r);
}

/// \brief Generate standard uniform random variates on [0, 1]
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_cc_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_cc_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_cc_distribution_impl<k>(rng, k, r);
    internal::u01_cc_distribution_impl<k>(rng, l, r);
}

/// \brief Generate standard uniform random variates on [0, 1)
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_co_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_co_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_co_distribution_impl<k>(rng, k, r);
    internal::u01_co_distribution_impl<k>(rng, l, r);
}

/// \brief Generate standard uniform random variates on (0, 1]
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_oc_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_oc_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_oc_distribution_impl<k>(rng, k, r);
    internal::u01_oc_distribution_impl<k>(rng, l, r);
}

/// \brief Generate standard uniform random variates on (0, 1)
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void u01_oo_distribution(RNGType &rng, std::size_t n, RealType *r)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**u01_oo_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = 1024;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::u01_oo_distribution_impl<k>(rng, k, r);
    internal::u01_oo_distribution_impl<k>(rng, l, r);
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01, u01, RealType)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01CC, u01_cc, RealType)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01CO, u01_co, RealType)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01OC, u01_oc, RealType)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01OO, u01_oo, RealType)

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
