//============================================================================
// vSMC/lib/src/rng/seed.cpp
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

#include "libvsmc.hpp"

extern "C" {

int vsmc_seed_get()
{
    return static_cast<int>(::vsmc::Seed::instance().get());
}

void vsmc_seed_set(int seed)
{
    ::vsmc::Seed::instance().set(static_cast<::vsmc::Seed::result_type>(seed));
}

void vsmc_seed_modulo(int div, int rem)
{
    ::vsmc::Seed::instance().modulo(static_cast<::vsmc::Seed::skip_type>(div),
        static_cast<::vsmc::Seed::skip_type>(rem));
}

void vsmc_seed_skip(int steps)
{
    ::vsmc::Seed::instance().skip(static_cast<::vsmc::Seed::skip_type>(steps));
}

int vsmc_seed_save(void *mem)
{
    std::size_t size = sizeof(::vsmc::Seed);
    if (mem != nullptr)
        std::memcpy(mem, &::vsmc::Seed::instance(), size);

    return static_cast<int>(size);
}

void vsmc_seed_load(const void *mem)
{
    std::memcpy(&::vsmc::Seed::instance(), mem, sizeof(::vsmc::Seed));
}

void vsmc_seed_save_f(const char *filename)
{
    std::ofstream os(filename);
    os << ::vsmc::Seed::instance();
    os.close();
}

void vsmc_seed_load_f(const char *filename)
{
    std::ifstream is(filename);
    is >> ::vsmc::Seed::instance();
    is.close();
}

} // extern "C"
