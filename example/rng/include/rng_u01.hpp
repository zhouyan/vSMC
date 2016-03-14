//============================================================================
// vSMC/example/rng/include/rng_u01.hpp
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

#ifndef VSMC_EXAMPLE_RNG_U01_HPP
#define VSMC_EXAMPLE_RNG_U01_HPP

#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/u01.hpp>
#include <vsmc/rngc/u01.h>

#define VSMC_DEFINE_RNGC_U01_LR_TEST(                                         \
    ubits, fsuffix, lr, Left, Right, RealType)                                \
    template <>                                                               \
    inline RealType rng_u01_lr_c<std::uint##ubits##_t, RealType, vsmc::Left,  \
        vsmc::Right>(std::uint##ubits##_t u)                                  \
    {                                                                         \
        return vsmc_u01_##lr##_u##ubits##fsuffix(u);                          \
    }

template <typename>
inline std::string rng_u01_type_name();

template <>
inline std::string rng_u01_type_name<float>()
{
    return "float";
}

template <>
inline std::string rng_u01_type_name<double>()
{
    return "double";
}

template <>
inline std::string rng_u01_type_name<long double>()
{
    return "long double";
}

template <>
inline std::string rng_u01_type_name<vsmc::Closed>()
{
    return "Closed";
}

template <>
inline std::string rng_u01_type_name<vsmc::Open>()
{
    return "Open";
}

template <typename UIntType, typename RealType, typename, typename>
inline RealType rng_u01_lr_c(UIntType u);

VSMC_DEFINE_RNGC_U01_LR_TEST(32, f, cc, Closed, Closed, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, f, cc, Closed, Closed, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, d, cc, Closed, Closed, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, d, cc, Closed, Closed, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, l, cc, Closed, Closed, long double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, l, cc, Closed, Closed, long double)

VSMC_DEFINE_RNGC_U01_LR_TEST(32, f, co, Closed, Open, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, f, co, Closed, Open, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, d, co, Closed, Open, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, d, co, Closed, Open, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, l, co, Closed, Open, long double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, l, co, Closed, Open, long double)

VSMC_DEFINE_RNGC_U01_LR_TEST(32, f, oc, Open, Closed, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, f, oc, Open, Closed, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, d, oc, Open, Closed, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, d, oc, Open, Closed, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, l, oc, Open, Closed, long double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, l, oc, Open, Closed, long double)

VSMC_DEFINE_RNGC_U01_LR_TEST(32, f, oo, Open, Open, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, f, oo, Open, Open, float)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, d, oo, Open, Open, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, d, oo, Open, Open, double)
VSMC_DEFINE_RNGC_U01_LR_TEST(32, l, oo, Open, Open, long double)
VSMC_DEFINE_RNGC_U01_LR_TEST(64, l, oo, Open, Open, long double)

template <typename RealType>
inline void rng_u01_lb(RealType x)
{
    if (vsmc::internal::is_equal(x, static_cast<RealType>(0)))
        std::cout << "Left:  " << std::setw(37) << std::left << 0;
    else
        std::cout << "Left:  2^" << std::setw(35) << std::left << std::log2(x);

    if (x < static_cast<RealType>(0))
        std::cout << "< 0";
    else if (x > static_cast<RealType>(0))
        std::cout << "Open";
    else
        std::cout << "Closed";
    std::cout << std::endl;
}

template <typename RealType>
inline void rng_u01_rb(RealType x)
{
    if (vsmc::internal::is_equal(x, static_cast<RealType>(1))) {
        std::cout << "Right: " << std::setw(37) << std::left << 1;
    } else {
        std::cout << "Right: 1 - 2^" << std::setw(31) << std::left
                  << std::log2(static_cast<RealType>(1) - x);
    }

    if (x > static_cast<RealType>(1))
        std::cout << "< 0";
    else if (x < static_cast<RealType>(1))
        std::cout << "Open";
    else
        std::cout << "Closed";
    std::cout << std::endl;
}

template <typename UIntType, typename RealType, typename Left, typename Right>
inline void rng_u01_lr()
{
    std::cout << std::string(50, '=') << std::endl;
    std::cout << "u01<uint" << std::numeric_limits<UIntType>::digits << "_t, "
              << rng_u01_type_name<RealType>() << ", "
              << rng_u01_type_name<Left>() << ", "
              << rng_u01_type_name<Right>() << ">" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    rng_u01_lb(vsmc::u01_lr<UIntType, RealType, Left, Right>(
        std::numeric_limits<UIntType>::min()));
    rng_u01_rb(vsmc::u01_lr<UIntType, RealType, Left, Right>(
        std::numeric_limits<UIntType>::max()));

    bool pass = true;
    std::size_t n = 100000000;
    vsmc::ThreefryEngine<UIntType, 4> rng;
    for (std::size_t i = 0; i != n; ++i) {
        UIntType u = rng();
        RealType c = rng_u01_lr_c<UIntType, RealType, Left, Right>(u);
        RealType cpp = vsmc::u01_lr<UIntType, RealType, Left, Right>(u);
        if (!vsmc::internal::is_equal(c, cpp)) {
            pass = false;
            break;
        }
    }
    std::cout << std::setw(44) << std::left
              << "C API: " << (pass ? "Passed" : "Failed") << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_U01_HPP
