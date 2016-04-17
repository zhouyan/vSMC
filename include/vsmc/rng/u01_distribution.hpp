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

/// \brief Default U01 distribution using fixed point conversion
/// \ingroup Config
#ifndef VSMC_RNG_U01_USE_FIXED_POINT
#define VSMC_RNG_U01_USE_FIXED_POINT 1
#endif

/// \brief Use 64-bits intermediate random integers for double precison output
/// \ingroup Config
#ifndef VSMC_RNG_U01_USE_64BITS_DOUBLE
#define VSMC_RNG_U01_USE_64BITS_DOUBLE 0
#endif

#define VSMC_DEFINE_U01_DISTRIBUTION(Name, name)                              \
    template <typename RealType>                                              \
    class Name##Distribution                                                  \
    {                                                                         \
        VSMC_DEFINE_RNG_DISTRIBUTION_0(                                       \
            Name, name, RealType, floating_point, FLOATING_POINT)             \
                                                                              \
        public:                                                               \
        result_type min() const { return 0; }                                 \
                                                                              \
        result_type max() const { return 1; }                                 \
                                                                              \
        void reset() {}                                                       \
                                                                              \
        private:                                                              \
        template <typename RNGType>                                           \
        result_type generate(RNGType &rng, const param_type &)                \
        {                                                                     \
            using UIntType =                                                  \
                typename internal::U01UIntType<RNGType, RealType>;            \
                                                                              \
            return name<UIntType, result_type>(                               \
                UniformBits<UIntType>::eval(rng));                            \
        }                                                                     \
    };

#define VSMC_DEFINE_U01_DISTRIBUTION_IMPL(name)                               \
    template <std::size_t K, typename RealType, typename RNGType>             \
    inline void name##_distribution_impl(                                     \
        RNGType &rng, std::size_t n, RealType *r)                             \
    {                                                                         \
        using UIntType = U01UIntType<RNGType, RealType>;                      \
                                                                              \
        Array<UIntType, K> s;                                                 \
        uniform_bits_distribution(rng, n, s.data());                          \
        name<UIntType, RealType>(n, s.data(), r);                             \
    }

namespace vsmc
{

namespace internal
{

#if VSMC_RNG_U01_USE_64BITS_DOUBLE

template <typename RNGType, typename RealType>
using U01UIntType =
    typename std::conditional<(RNGTraits<RNGType>::bits >= 64 ||
                                  std::is_same<RealType, long double>::value ||
                                  std::is_same<RealType, double>::value),
        std::uint64_t, std::uint32_t>::type;

#else // VSMC_RNG_U01_USE_64BITS_DOUBLE

template <typename RNGType, typename RealType>
using U01UIntType =
    typename std::conditional<(RNGTraits<RNGType>::bits >= 64 ||
                                  std::is_same<RealType, long double>::value),
        std::uint64_t, std::uint32_t>::type;

#endif // VSMC_RNG_U01_USE_64BITS_DOUBLE

} // namespace vsmc::internal

/// \brief Standard uniform distribution on [0, 1]
/// \ingroup Distribution
VSMC_DEFINE_U01_DISTRIBUTION(U01CC, u01_cc)

/// \brief Standard uniform distribution on [0, 1)
/// \ingroup Distribution
VSMC_DEFINE_U01_DISTRIBUTION(U01CO, u01_co)

/// \brief Standard uniform distribution on (0, 1]
/// \ingroup Distribution
VSMC_DEFINE_U01_DISTRIBUTION(U01OC, u01_oc)

/// \brief Standard uniform distribution on (0, 1)
/// \ingroup Distribution
VSMC_DEFINE_U01_DISTRIBUTION(U01OO, u01_oo)

#if VSMC_RNG_U01_USE_FIXED_POINT

template <typename RealType>
class U01Distribution : public U01CODistribution<RealType>
{
}; // class U01Distribution

#else // VSMC_RNG_U01_USE_FIXED_POINT

/// \brief Standard uniform distribution on [0, 1)
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
        return generate<0>(rng, std::true_type());
    }

    template <std::size_t, typename RNGType>
    result_type generate(RNGType &, std::false_type)
    {
        return 0;
    }

    template <std::size_t N, typename RNGType>
    result_type generate(RNGType &rng, std::true_type)
    {
        using UIntType = internal::U01UIntType<RNGType, RealType>;

        static constexpr int W = std::numeric_limits<UIntType>::digits;
        static constexpr int M = std::numeric_limits<RealType>::digits;
        static constexpr int P = (W + M - 1) / W;
        static constexpr int Q = 1 > P ? 1 : P;

        return static_cast<RealType>(UniformBits<UIntType>::eval(rng)) *
            internal::U01Pow2Inv<RealType, (Q - N) * W>::value +
            generate<N + 1>(rng, std::integral_constant<bool, N + 1 < Q>());
    }
}; // class U01Distribution

#endif // VSMC_RNG_U01_USE_FIXED_POINT

namespace internal
{

VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_cc)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_co)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_oc)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_oo)

#if VSMC_RNG_U01_USE_FIXED_POINT

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    u01_co_distribution_impl<K>(rng, n, r);
}

#else // VSMC_RNG_U01_USE_FIXED_POINT

template <std::size_t, typename RealType, typename UIntType>
inline RealType u01_distribution_impl(const UIntType *, std::false_type)
{
    return 0;
}

template <std::size_t N, typename RealType, typename UIntType>
inline RealType u01_distribution_impl(const UIntType *u, std::true_type)
{
    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = (M + W - 1) / W;
    static constexpr int Q = 1 > P ? 1 : P;

    return static_cast<RealType>(u[N]) *
        U01Pow2Inv<RealType, (Q - N) * W>::value +
        u01_distribution_impl<N + 1, RealType>(
               u, std::integral_constant<bool, N + 1 < Q>());
}

template <std::size_t K, typename RealType, typename RNGType>
inline void u01_distribution_impl(RNGType &rng, std::size_t n, RealType *r)
{
    using UIntType = U01UIntType<RNGType, RealType>;

    static constexpr int W = std::numeric_limits<UIntType>::digits;
    static constexpr int M = std::numeric_limits<RealType>::digits;
    static constexpr int P = (M + W - 1) / W;
    static constexpr int Q = 1 > P ? 1 : P;

    Array<UIntType, K * Q> s;
    uniform_bits_distribution(rng, n * Q, s.data());
    const UIntType *u = s.data();
    for (std::size_t i = 0; i != n; ++i, u += Q)
        r[i] = u01_distribution_impl<0, RealType>(u, std::true_type());
}

#endif // VSMC_RNG_U01_USE_FIXED_POINT

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates on [0, 1]
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01_cc)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01CC, u01_cc, RealType)

/// \brief Generate standard uniform random variates on [0, 1)
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01_co)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01CO, u01_co, RealType)

/// \brief Generate standard uniform random variates on (0, 1]
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01_oc)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01OC, u01_oc, RealType)

/// \brief Generate standard uniform random variates on (0, 1)
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01_oo)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01OO, u01_oo, RealType)

/// \brief Generate standard uniform random variates
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01, u01, RealType)

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
