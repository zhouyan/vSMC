//============================================================================
// vSMC/vSMCExample/utility/include/utility_cstring.hpp
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

#ifndef VSMC_EXAMPLE_UTILITY_CSTRING_HPP
#define VSMC_EXAMPLE_UTILITY_CSTRING_HPP

#include <vsmc/rng/threefry.hpp>
#include <vsmc/utility/aligned_memory.hpp>
#include <vsmc/utility/cstring.hpp>
#include <vsmc/utility/stop_watch.hpp>
#if VSMC_HAS_RDTSCP
#include <vsmc/utility/rdtsc.hpp>
#endif

static const int Repeat = 10;
static const int BitsMin = 3;
static const int BitsPad = 27;
static const int BitsMax = 29;
static const std::size_t BAlloc = (1U << BitsMax) + (1U << BitsPad) * 4;
static const std::size_t BMin = (1U << BitsMin);
static const std::size_t BPad = (1U << BitsPad);
static const std::size_t BMax = (1U << BitsMax);

inline std::string verify (const char *y, const char *z,
        std::size_t B, int offset)
{
    y += offset;
    for (std::size_t b = 0; b != B; ++b)
        if (y[b] != z[b])
            return std::string("Fail");

    return std::string("Pass");
}

template <typename IntType>
inline std::string size_h (IntType B)
{
    std::ptrdiff_t BS = static_cast<std::ptrdiff_t>(B);
    std::string ret;
    if (BS <= 0) {
        ret += "-";
        BS = -BS;
    }

    std::ptrdiff_t BH = 0;
    if (BS % 1024 == 0 || BS < 1024) {
        if (BS >= 1024 * 1024 * 1024)
            BH = BS / (1024 * 1024 * 1024);
        else if (BS >= 1024 * 1024)
            BH = BS / (1024 * 1024);
        else if (BS >= 1024)
            BH = BS / 1024;
        else
            BH = BS;
    } else {
        double BD;
        if (BS >= 1024 * 1024 * 1024)
            BD = static_cast<double>(BS) / (1024 * 1024 * 1024.0);
        else if (BS >= 1024 * 1024)
            BD = static_cast<double>(BS) / (1024 * 1024.0);
        else if (BS >= 1024)
            BD = static_cast<double>(BS) / 1024.0;
        else
            BD = static_cast<double>(BS);
        BH = static_cast<std::ptrdiff_t>(BD);
    }

    std::stringstream ss;
    ss << BH;
    ret += ss.str();

    if (BS >= 1024 * 1024 * 1024)
        ret += "GB";
    else if (BS >= 1024 * 1024)
        ret += "MB";
    else if (BS >= 1024)
        ret += "KB";
    else
        ret += "B";

    return ret;
}

inline void generate (char *x, char *y, char *z)
{
    std::size_t N = BAlloc / sizeof(uint64_t);
    vsmc::Threefry4x64 eng;
    uint64_t *u64 = reinterpret_cast<uint64_t *>(x);
    for (std::size_t i = 0; i != N; ++i)
        u64[i] = eng();
    std::memcpy(y, x, BAlloc);
    std::memcpy(z, x, BAlloc);
}

#endif // VSMC_EXAMPLE_UTILITY_CSTRING_HPP
