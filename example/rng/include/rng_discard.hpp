//============================================================================
// vSMC/example/rng/include/rng_discard.hpp
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

#ifndef VSMC_EXAMPLE_RNG_DISCARD_HPP
#define VSMC_EXAMPLE_RNG_DISCARD_HPP

#include <vsmc/rng/rng.hpp>

#define VSMC_RNG_DISCARD_TEST(Name) rng_discard_test<Name>(N, m, #Name);

template <typename RNGType>
inline void rng_discard_test(
    unsigned N, std::size_t m, const std::string &name)
{
    vsmc::RNG rng;
    std::uniform_int_distribution<unsigned> runif(N / 2, N * 2);
    RNGType rng1;
    RNGType rng2;
    bool passed = true;
    unsigned extra = 0;
    for (std::size_t i = 0; i != m; ++i) {
        unsigned n = runif(rng);
        rng1.discard(n);
        auto next = rng1();
        for (unsigned j = 0; j != n; ++j)
            rng2();
        bool find = false;
        for (unsigned j = 0; j != 1000; ++j) {
            if (rng2() == next) {
                extra = std::max(extra, j);
                find = true;
                break;
            }
        }
        passed = passed && find;
    }

    std::cout << std::setw(60) << std::left << name << std::setw(20)
              << std::right << (passed ? "Passed" : "Failed") << std::endl;
    std::cout << std::setw(60) << std::left << "Extra elements"
              << std::setw(20) << std::right << extra << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

#endif // VSMC_EXAMPLE_RNG_DISCARD_HPP
