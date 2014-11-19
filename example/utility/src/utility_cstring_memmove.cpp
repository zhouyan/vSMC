//============================================================================
// vSMC/vSMCExample/utility/src/utility_cstring_memmove.cpp
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

#include "utility_cstring_memmove.hpp"

int main (int argc, char **argv)
{
    std::vector<int> bits;
    if (argc > 1) {
        bits.push_back(std::atoi(argv[1]));
    } else {
        for (int b = BitsMax; b >= BitsMin; --b)
            bits.push_back(b);
    }

    std::vector<int> offsets1;
    std::vector<int> offsets2;
    if (argc > 3){
        offsets1.push_back(std::atoi(argv[2]));
        offsets2.push_back(std::atoi(argv[3]));
    } else {
        int P = 1U << BitsPad;
        offsets_push_back(offsets1, offsets2, 32);
        offsets_push_back(offsets1, offsets2, 1);
        offsets_push_back(offsets1, offsets2, 31);
        offsets_push_back(offsets1, offsets2, P);
        offsets_push_back(offsets1, offsets2, P + 1);
        offsets_push_back(offsets1, offsets2, P - 1);
    }

    vsmc::AlignedAllocator<char> allocator;
    char *x = static_cast<char *>(allocator.allocate(BAlloc));
    char *y = static_cast<char *>(allocator.allocate(BAlloc));
    char *z = static_cast<char *>(allocator.allocate(BAlloc));
    generate(x, y, z);

    std::cout.precision(5);
    std::cout << std::string(100, '=') << std::endl;
    std::cout << std::setw(60) << "vSMC";
    std::cout << std::setw(30) << "System";
    std::cout << std::endl;
    std::cout << std::string(100, '-') << std::endl;
    std::cout << std::setw(10) << "Size";
    std::cout << std::setw(10) << "Size.H";
    std::cout << std::setw(10) << "Offset";
    std::cout << std::setw(10) << "Dist.H";
    std::cout << std::setw(10) << "cpB";
    std::cout << std::setw(10) << "GB/s";
    std::cout << std::setw(10) << "Verify";
    std::cout << std::setw(10) << "cpB";
    std::cout << std::setw(10) << "GB/s";
    std::cout << std::setw(10) << "Speedup";
    std::cout << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    for (std::size_t i = 0; i != bits.size(); ++i) {
        const std::size_t B = 1U << bits[i];
        const std::size_t R = BMax / B * Repeat;
        for (std::size_t j = 0; j != offsets1.size(); ++j)
            test(x + offsets1[j], x + offsets2[j], z, B, R);
    }

    std::cout << std::string(100, '=') << std::endl;

    allocator.deallocate(x, BAlloc);
    allocator.deallocate(y, BAlloc);
    allocator.deallocate(z, BAlloc);

    return 0;
}
