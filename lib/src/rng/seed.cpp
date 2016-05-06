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

#include "libvsmcrng.hpp"

namespace vsmc
{

using SeedC = SeedGenerator<NullType, unsigned>;

} // namespace vsmc

extern "C" {

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    inline void vsmc_seed_##name(vsmc_rng rng)                                \
    {                                                                         \
        ::vsmc::SeedC::instance()(*reinterpret_cast<RNGType *>(rng.ptr));     \
    }

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

using vsmc_seed_type = void (*)(vsmc_rng);

static vsmc_seed_type vsmc_seed_dispatch[] = {

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name) vsmc_seed_##name,
#define VSMC_RNG_DEFINE_MACRO_NA(RNGType, Name, name) nullptr,

#include <vsmc/rng/internal/rng_define_macro_alias.hpp>

#include <vsmc/rng/internal/rng_define_macro.hpp>

    nullptr}; // vsmc_seed_dispatch

void vsmc_seed(vsmc_rng rng)
{
    vsmc_seed_dispatch[static_cast<std::size_t>(rng.type)](rng);
}

unsigned vsmc_seed_get(void) { return ::vsmc::SeedC::instance().get(); }

void vsmc_seed_set(unsigned seed) { ::vsmc::SeedC::instance().set(seed); }

void vsmc_seed_modulo(unsigned div, unsigned rem)
{
    ::vsmc::SeedC::instance().modulo(div, rem);
}

size_t vsmc_seed_save(void *mem)
{
    std::size_t size = sizeof(::vsmc::SeedC);
    if (mem != nullptr)
        std::memcpy(mem, &::vsmc::SeedC::instance(), size);

    return size;
}

void vsmc_seed_load(const void *mem)
{
    std::memcpy(&::vsmc::SeedC::instance(), mem, sizeof(::vsmc::SeedC));
}

void vsmc_seed_save_f(const char *filename)
{
    std::ofstream os(filename);
    os << ::vsmc::SeedC::instance() << std::endl;
    os.close();
}

void vsmc_seed_load_f(const char *filename)
{
    std::ifstream is(filename);
    is >> ::vsmc::SeedC::instance();
    is.close();
}

} // extern "C"
