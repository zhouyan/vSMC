//============================================================================
// vSMC/lib/src/vsmc_rng_cast.hpp
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

#ifndef VSMC_LIB_RNG_CAST_HPP
#define VSMC_LIB_RNG_CAST_HPP

#include <vsmc/rng/engine.hpp>

namespace vsmc
{

namespace internal
{

inline RNG &rng_cast(vsmc_rng *rng_ptr)
{
    std::uintptr_t r = reinterpret_cast<std::uintptr_t>(rng_ptr->state) % 32;

    if (r == 0)
        return *(reinterpret_cast<::vsmc::RNG *>(rng_ptr->state));

    return *(reinterpret_cast<::vsmc::RNG *>(
        rng_ptr->state + (32 - r) / sizeof(unsigned)));
}

inline const RNG &rng_cast(const vsmc_rng *rng_ptr)
{
    std::uintptr_t r = reinterpret_cast<std::uintptr_t>(rng_ptr->state) % 32;

    if (r == 0)
        return *(reinterpret_cast<const ::vsmc::RNG *>(rng_ptr->state));

    return *(reinterpret_cast<const ::vsmc::RNG *>(
        rng_ptr->state + (32 - r) / sizeof(unsigned)));
}

} // namespace internal

} // namespace vsmc

#endif // VSMC_LIB_RNG_CAST_HPP
