//============================================================================
// vSMC/lib/src/libvsmcrng.hpp
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

#ifndef VSMC_LIBVSMCRNG_HPP
#define VSMC_LIBVSMCRNG_HPP

#include <vsmc/rng/rng.h>
#include <vsmc/rng/rng.hpp>

#define VSMC_RUNTIME_ASSERT_LIB_RNG_TYPE(rng1, rng2, func)                    \
    VSMC_RUNTIME_ASSERT((rng1.type == rng2.type),                             \
        "**vsmc_rng_" #func "CALLED WITH TWO RNG OF DIFFERENT TYPES")

#define VSMC_DEFINE_LIB_RNG_DIST(RNGType)                                     \
    RNGType &rng_cpp = *reinterpret_cast<RNGType *>(rng.ptr);                 \
    for (size_t i = 0; i != n; ++i)                                           \
        r[i] = dist(rng_cpp);

#endif // VSMC_LIBVSMCRNG_HPP
