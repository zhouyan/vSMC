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
            return name<internal::U01UIntType<RNGType>, result_type>(         \
                UniformBits<internal::U01UIntType<RNGType>>::eval(rng));      \
        }                                                                     \
    };

#define VSMC_DEFINE_U01_DISTRIBUTION_IMPL(name)                               \
    template <std::size_t K, typename RealType, typename RNGType>             \
    inline void name##_distribution_impl(                                     \
        RNGType &rng, std::size_t n, RealType *r)                             \
    {                                                                         \
        using UIntType = U01UIntType<RNGType>;                                \
                                                                              \
        alignas(32) UIntType s[K];                                            \
        uniform_bits_distribution(rng, n, s);                                 \
        name<UIntType, RealType>(n, s, r);                                    \
    }

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
VSMC_DEFINE_U01_DISTRIBUTION(U01, u01)

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

namespace internal
{

VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_cc)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_co)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_oc)
VSMC_DEFINE_U01_DISTRIBUTION_IMPL(u01_oo)

} // namespace vsmc::internal

/// \brief Generate standard uniform random variates
/// \ingroup Distribution
VSMC_DEFINE_RNG_DISTRIBUTION_IMPL_0(u01)
VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(U01, u01, RealType)

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

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
