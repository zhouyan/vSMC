//============================================================================
// vSMC/lib/src/rng/rng.cpp
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

#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/rng.h>

#define VSMC_RUNTIME_ASSERT_LIB_RNG_TYPE(rng1, rng2, func)                    \
    VSMC_RUNTIME_ASSERT((rng1.type == rng2.type),                             \
        "**vsmc_rng_" #func "CALLED WITH TWO RNG OF DIFFERENT TYPES")

extern "C" {

#include "rng_new.cpp"

#include "rng_delete.cpp"

#include "rng_assign.cpp"

#include "rng_seed.cpp"

#include "rng_rand.cpp"

#include "rng_discard.cpp"

#include "rng_is_eq.cpp"

#include "rng_is_neq.cpp"

#include "rng_load.cpp"

#include "rng_load_f.cpp"

#include "rng_save.cpp"

#include "rng_save_f.cpp"

} // extern "C"
