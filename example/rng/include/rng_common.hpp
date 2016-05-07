//============================================================================
// vSMC/example/rng/include/rng_common.hpp
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

#ifndef VSMC_EXAMPLE_RNG_COMMON_HPP
#define VSMC_EXAMPLE_RNG_COMMON_HPP

#include <vsmc/rng/rng.hpp>
#include <vsmc/utility/program_option.hpp>
#include <vsmc/utility/stop_watch.hpp>

#define VSMC_EXAMPLE_RNG_MAIN(prg, NVal, MVal)                                \
    int main(int argc, char **argv)                                           \
    {                                                                         \
        std::size_t N = NVal;                                                 \
        std::size_t M = MVal;                                                 \
        vsmc::ProgramOptionMap option;                                        \
        option.add("N", "Number of samples in each run", &N, N);              \
        option.add("M", "Number of repetitions", &M, M);                      \
        option.process(argc, argv);                                           \
        if (option.count("help"))                                             \
            return 0;                                                         \
                                                                              \
        rng_##prg(N, M);                                                      \
                                                                              \
        return 0;                                                             \
    }

template <typename>
inline std::string rng_type_name();

template <>
inline std::string rng_type_name<float>()
{
    return "float";
}

template <>
inline std::string rng_type_name<double>()
{
    return "double";
}

template <>
inline std::string rng_type_name<long double>()
{
    return "long double";
}

template <>
inline std::string rng_type_name<vsmc::Closed>()
{
    return "Closed";
}

template <>
inline std::string rng_type_name<vsmc::Open>()
{
    return "Open";
}

inline std::string rng_pass(bool pass) { return pass ? "Passed" : "Failed"; }

#endif // VSMC_EXAMPLE_RNG_COMMON_HPP
