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

#define VSMC_DEFINE_RNGC_U01_TEST(ubits, fsuffix, lr, Left, Right, RealType)  \
    template <>                                                               \
    inline RealType rng_u01_c<std::uint##ubits##_t, RealType, vsmc::Left,     \
        vsmc::Right>(std::uint##ubits##_t u)                                  \
    {                                                                         \
        return vsmc_u01_##lr##_u##ubits##fsuffix(u);                          \
    }

#define VSMC_RNG_U01_TEST(Left, Right)                                        \
    rng_u01<std::uint32_t, float, vsmc::Left, vsmc::Right>(N, M);             \
    rng_u01<std::uint64_t, float, vsmc::Left, vsmc::Right>(N, M);             \
    rng_u01<std::uint32_t, double, vsmc::Left, vsmc::Right>(N, M);            \
    rng_u01<std::uint64_t, double, vsmc::Left, vsmc::Right>(N, M);            \
    rng_u01<std::uint32_t, long double, vsmc::Left, vsmc::Right>(N, M);       \
    rng_u01<std::uint64_t, long double, vsmc::Left, vsmc::Right>(N, M);

template <typename UIntType, typename RealType, typename, typename>
inline RealType rng_u01_c(UIntType u);

VSMC_DEFINE_RNGC_U01_TEST(32, f, cc, Closed, Closed, float)
VSMC_DEFINE_RNGC_U01_TEST(64, f, cc, Closed, Closed, float)
VSMC_DEFINE_RNGC_U01_TEST(32, d, cc, Closed, Closed, double)
VSMC_DEFINE_RNGC_U01_TEST(64, d, cc, Closed, Closed, double)
VSMC_DEFINE_RNGC_U01_TEST(32, l, cc, Closed, Closed, long double)
VSMC_DEFINE_RNGC_U01_TEST(64, l, cc, Closed, Closed, long double)

VSMC_DEFINE_RNGC_U01_TEST(32, f, co, Closed, Open, float)
VSMC_DEFINE_RNGC_U01_TEST(64, f, co, Closed, Open, float)
VSMC_DEFINE_RNGC_U01_TEST(32, d, co, Closed, Open, double)
VSMC_DEFINE_RNGC_U01_TEST(64, d, co, Closed, Open, double)
VSMC_DEFINE_RNGC_U01_TEST(32, l, co, Closed, Open, long double)
VSMC_DEFINE_RNGC_U01_TEST(64, l, co, Closed, Open, long double)

VSMC_DEFINE_RNGC_U01_TEST(32, f, oc, Open, Closed, float)
VSMC_DEFINE_RNGC_U01_TEST(64, f, oc, Open, Closed, float)
VSMC_DEFINE_RNGC_U01_TEST(32, d, oc, Open, Closed, double)
VSMC_DEFINE_RNGC_U01_TEST(64, d, oc, Open, Closed, double)
VSMC_DEFINE_RNGC_U01_TEST(32, l, oc, Open, Closed, long double)
VSMC_DEFINE_RNGC_U01_TEST(64, l, oc, Open, Closed, long double)

VSMC_DEFINE_RNGC_U01_TEST(32, f, oo, Open, Open, float)
VSMC_DEFINE_RNGC_U01_TEST(64, f, oo, Open, Open, float)
VSMC_DEFINE_RNGC_U01_TEST(32, d, oo, Open, Open, double)
VSMC_DEFINE_RNGC_U01_TEST(64, d, oo, Open, Open, double)
VSMC_DEFINE_RNGC_U01_TEST(32, l, oo, Open, Open, long double)
VSMC_DEFINE_RNGC_U01_TEST(64, l, oo, Open, Open, long double)

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

template <typename UIntType, typename RealType, typename Left, typename Right>
inline std::string rng_u01_function_name()
{
    std::stringstream ss;
    ss << "u01<uint";
    ss << std::numeric_limits<UIntType>::digits << "_t, ";
    ss << rng_u01_type_name<RealType>() << ", ";
    ss << rng_u01_type_name<Left>() << ", ";
    ss << rng_u01_type_name<Right>() << ">";

    return ss.str();
}

template <typename RealType>
inline void rng_u01_lb(RealType x, std::string &minimum, std::string &left)
{
    std::stringstream ss;
    if (vsmc::internal::is_equal(x, static_cast<RealType>(0)))
        ss << 0;
    else
        ss << "2^" << std::log2(x);
    minimum = ss.str();

    if (x < static_cast<RealType>(0))
        left = "< 0";
    else if (x > static_cast<RealType>(0))
        left = "Open";
    else
        left = "Closed";
}

template <typename RealType>
inline void rng_u01_rb(RealType x, std::string &maximum, std::string &right)
{
    std::stringstream ss;
    if (vsmc::internal::is_equal(x, static_cast<RealType>(1)))
        ss << 1;
    else
        ss << "1 - 2^" << std::log2(static_cast<RealType>(1) - x);
    maximum = ss.str();
    if (x > static_cast<RealType>(1))
        right = "> 1";
    else if (x < static_cast<RealType>(1))
        right = "Open";
    else
        right = "Closed";
}

template <typename UIntType, typename RealType, typename Left, typename Right>
inline void rng_u01(std::size_t N, std::size_t M)
{
    vsmc::ThreefryEngine<UIntType> rng;
    vsmc::Vector<UIntType> u(N);
    vsmc::Vector<RealType> r(N);
    vsmc::Vector<RealType> r1(N);
    vsmc::Vector<RealType> r2(N);
    bool passed1 = true;
    bool passed2 = true;
    for (std::size_t i = 0; i != M; ++i) {
        vsmc::rng_rand(rng, N, u.data());

        for (std::size_t j = 0; j != N; ++j)
            r[j] = vsmc::u01<UIntType, RealType, Left, Right>(u[j]);

        for (std::size_t j = 0; j != N; ++j)
            r1[j] = rng_u01_c<UIntType, RealType, Left, Right>(u[j]);
        passed1 = passed1 && r1 == r;

        vsmc::u01<UIntType, RealType, Left, Right>(N, u.data(), r2.data());
        passed2 = passed2 && r2 == r;
    }

    std::string function =
        rng_u01_function_name<UIntType, RealType, Left, Right>();
    std::string minimum;
    std::string maximum;
    std::string left;
    std::string right;
    rng_u01_lb(vsmc::u01<UIntType, RealType, Left, Right>(
                   std::numeric_limits<UIntType>::min()),
        minimum, left);
    rng_u01_rb(vsmc::u01<UIntType, RealType, Left, Right>(
                   std::numeric_limits<UIntType>::max()),
        maximum, right);
    std::string pass1 = passed1 ? "Passed" : "Failed";
    std::string pass2 = passed2 ? "Passed" : "Failed";
    std::cout << std::setw(50) << std::left << function;
    std::cout << std::setw(15) << std::right << minimum;
    std::cout << std::setw(15) << std::right << maximum;
    std::cout << std::setw(10) << std::right << left;
    std::cout << std::setw(10) << std::right << right;
    std::cout << std::setw(10) << std::right << pass1;
    std::cout << std::setw(10) << std::right << pass2;
    std::cout << std::endl;
}

inline void rng_u01(std::size_t N, std::size_t M)
{
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::setw(50) << std::left << "Function";
    std::cout << std::setw(15) << std::right << "Mininum";
    std::cout << std::setw(15) << std::right << "Maximum";
    std::cout << std::setw(10) << std::right << "Left";
    std::cout << std::setw(10) << std::right << "Right";
    std::cout << std::setw(10) << std::right << "C API";
    std::cout << std::setw(10) << std::right << "Batch";
    std::cout << std::endl;
    std::cout << std::string(120, '-') << std::endl;
    VSMC_RNG_U01_TEST(Closed, Closed);
    std::cout << std::string(120, '-') << std::endl;
    VSMC_RNG_U01_TEST(Closed, Open);
    std::cout << std::string(120, '-') << std::endl;
    VSMC_RNG_U01_TEST(Open, Closed);
    std::cout << std::string(120, '-') << std::endl;
    VSMC_RNG_U01_TEST(Open, Open);
    std::cout << std::string(120, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_U01_HPP
