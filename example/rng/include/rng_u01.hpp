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

#define VSMC_RNG_U01_TEST(ubits, fbits, RealType, L, R, l, r, pass)           \
    {                                                                         \
        const std::size_t n = 1000000;                                        \
        vsmc::Threefry4x##ubits rng;                                          \
        bool test = true;                                                     \
        for (std::size_t i = 0; i != n; ++i) {                                \
            std::uint##ubits##_t u = rng();                                   \
            RealType fc = vsmc_u01_##l##_##r##_u##ubits##_f##fbits(u);        \
            RealType fcpp = vsmc::U01<std::uint##ubits##_t, RealType,         \
                vsmc::L, vsmc::R>::eval(u);                                   \
            if (!vsmc::internal::is_equal(fc, fcpp)) {                        \
                test = false;                                                 \
                break;                                                        \
            }                                                                 \
        }                                                                     \
        std::cout << std::left << std::setw(40)                               \
                  << "U01<uint" #ubits "_t, " #RealType ", " #L ", " #R ">";  \
        std::cout << (test ? "Passed" : "Failed");                            \
        if (!test && !pass)                                                   \
            std::cout << " (expected)";                                       \
        std::cout << std::endl;                                               \
    }

#endif // VSMC_EXAMPLE_RNG_U01_HPP
