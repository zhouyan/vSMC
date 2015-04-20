//============================================================================
// vSMC/example/rng/include/rng_validation.cpp
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

#ifndef VSMC_EXAMPLE_RNG_VALIDATION_HPP
#define VSMC_EXAMPLE_RNG_VALIDATION_HPP

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

template <typename Eng>
inline void rng_validation(
    const std::vector<unsigned long long> &result, const std::string &name)
{
    bool success = true;
    Eng eng(static_cast<typename Eng::result_type>(result.back()));
    for (std::size_t i = 0; i != result.size() - 1; ++i) {
        if (static_cast<unsigned long long>(eng()) != result[i]) {
            success = false;
            break;
        }
    }

    if (success) {
        std::cout << std::left << std::setw(20) << name << " passed"
                  << std::endl;
    } else {
        std::cout << std::left << std::setw(20) << name << " failed"
                  << std::endl;
    }
}

#endif // VSMC_EXAMPLE_RNG_VALIDATION_HPP
