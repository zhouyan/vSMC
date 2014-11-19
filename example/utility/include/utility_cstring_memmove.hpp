//============================================================================
// vSMC/vSMCExample/utility/include/utility_cstring_memmove.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_EXAMPLE_UTILITY_CSTRING_MEMMOVE_HPP
#define VSMC_EXAMPLE_UTILITY_CSTRING_MEMMOVE_HPP

#include "utility_cstring.hpp"

inline void offsets_push_back (
        std::vector<int> &offsets1, std::vector<int> &offsets2, int dist)
{
    offsets1.push_back(dist);
    offsets2.push_back(0);
    offsets1.push_back(0);
    offsets2.push_back(dist);
}

inline void test (char *x, char *y, char *z, std::size_t B, std::size_t R)
{
    std::cout << std::setw(10) << B;
    std::cout << std::setw(10) << size_h(B);

    std::ptrdiff_t offset = x - y;
    if (offset > 0)
        std::cout << std::setw(10) << offset % 32;
    else
        std::cout << std::setw(10) << -((-offset) % 32);
    std::cout << std::setw(10) << size_h(offset);

#if VSMC_HAS_RDTSCP
    vsmc::RDTSCPCounter counter;
#endif
    vsmc::StopWatch watch;
    const double dB = static_cast<double>(B * R);

    std::memmove(x, z, B);
    watch.reset();
#if VSMC_HAS_RDTSCP
    counter.reset();
#endif
    watch.start();
#if VSMC_HAS_RDTSCP
    counter.start();
#endif
    for (std::size_t r = 0; r != R; ++r)
        vsmc::memmove(y, x, B);
#if VSMC_HAS_RDTSCP
    counter.stop();
#endif
    watch.stop();
    double tvsmc = watch.nanoseconds();
    std::memmove(x, z, B);
    vsmc::memmove(y, x, B);
#if VSMC_HAS_RDTSCP
    std::cout << std::setw(10) << std::fixed << counter.cycles() / dB;
#else
    std::cout << std::setw(10) << std::fixed << 0;
#endif
    std::cout << std::setw(10) << std::fixed << dB / tvsmc;
    std::cout << std::setw(10) << verify(y, z, B, 0);

    watch.reset();
#if VSMC_HAS_RDTSCP
    counter.reset();
#endif
    watch.start();
#if VSMC_HAS_RDTSCP
    counter.start();
#endif
    for (std::size_t r = 0; r != R; ++r)
        std::memmove(y, x, B);
#if VSMC_HAS_RDTSCP
    counter.stop();
#endif
    watch.stop();
    double tsys = watch.nanoseconds();
#if VSMC_HAS_RDTSCP
    std::cout << std::setw(10) << std::fixed << counter.cycles() / dB;
#else
    std::cout << std::setw(10) << std::fixed << 0;
#endif
    std::cout << std::setw(10) << std::fixed << dB / tsys;
    std::cout << std::setw(10) << std::fixed << tsys / tvsmc;

    std::cout << std::endl;
}

#endif // VSMC_EXAMPLE_UTILITY_CSTRING_MEMMOVE_HPP
