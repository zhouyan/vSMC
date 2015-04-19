//============================================================================
// vSMC/example/utility/src/utility_cstring_memcpy.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#include "utility_cstring_memcpy.hpp"

int main(int argc, char **argv)
{
    std::vector<int> bits;
    if (argc > 1) {
        bits.push_back(std::atoi(argv[1]));
    } else {
        for (int b = BitsMax; b >= BitsMin; --b)
            bits.push_back(b);
    }

    std::vector<int> offsets;
    if (argc > 2) {
        offsets.push_back(std::atoi(argv[2]));
    } else {
        offsets.push_back(0);
        offsets.push_back(1);
        offsets.push_back(31);
    }

    vsmc::AlignedAllocator<char> allocator;
    char *x = static_cast<char *>(allocator.allocate(BAlloc));
    char *y = static_cast<char *>(allocator.allocate(BAlloc));
    char *z = static_cast<char *>(allocator.allocate(BAlloc));
    generate(x, y, z);

    std::cout.precision(5);
    std::cout << std::string(90, '=') << std::endl;
    std::cout << std::setw(50) << "vSMC";
    std::cout << std::setw(30) << "System";
    std::cout << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    std::cout << std::setw(10) << "Size";
    std::cout << std::setw(10) << "Size.H";
    std::cout << std::setw(10) << "Offset";
    std::cout << std::setw(10) << "cpB";
    std::cout << std::setw(10) << "GB/s";
    std::cout << std::setw(10) << "Verify";
    std::cout << std::setw(10) << "cpB";
    std::cout << std::setw(10) << "GB/s";
    std::cout << std::setw(10) << "Speedup";
    std::cout << std::endl;
    std::cout << std::string(90, '-') << std::endl;

    for (std::size_t i = 0; i != bits.size(); ++i) {
        const std::size_t B = 1U << bits[i];
        const std::size_t R = BMax / B * Repeat;
        for (std::size_t j = 0; j != offsets.size(); ++j)
            test(x, y, z, B, offsets[j], R);
    }

    std::cout << std::string(90, '=') << std::endl;

    allocator.deallocate(x, BAlloc);
    allocator.deallocate(y, BAlloc);
    allocator.deallocate(z, BAlloc);

    return 0;
}
