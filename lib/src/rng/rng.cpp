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

#include "libvsmc.hpp"

extern "C" {

vsmc_rng vsmc_rng_new(int seed)
{
    auto ptr = new ::vsmc::RNG(static_cast<::vsmc::RNG::result_type>(seed));
    vsmc_rng rng = {ptr};

    return rng;
}

void vsmc_rng_delete(vsmc_rng *rng_ptr)
{
    delete ::vsmc::cast(rng_ptr);
    rng_ptr->ptr = nullptr;
}

void vsmc_rng_assign(vsmc_rng rng, vsmc_rng other)
{
    ::vsmc::cast(rng) = ::vsmc::cast(other);
}

void vsmc_rng_seed(vsmc_rng rng, int seed)
{
    ::vsmc::cast(rng).seed(static_cast<::vsmc::RNG::result_type>(seed));
}

void vsmc_rng_get_key(vsmc_rng rng, int n, int *key)
{
    auto k = ::vsmc::cast(rng).key();
    const std::size_t dst_size = static_cast<std::size_t>(n) * sizeof(int);
    const std::size_t src_size = sizeof(typename ::vsmc::RNG::key_type);
    const std::size_t size = src_size < dst_size ? src_size : dst_size;
    std::memcpy(key, k.data(), size);
}

void vsmc_rng_set_key(vsmc_rng rng, int n, const int *key)
{
    typename ::vsmc::RNG::key_type k;
    const std::size_t src_size = static_cast<std::size_t>(n) * sizeof(int);
    const std::size_t dst_size = sizeof(typename ::vsmc::RNG::key_type);
    const std::size_t size = src_size < dst_size ? src_size : dst_size;
    std::memcpy(k.data(), key, size);
    ::vsmc::cast(rng).key(k);
}

void vsmc_rng_get_ctr(const vsmc_rng rng, int n, int *ctr)
{
    auto c = ::vsmc::cast(rng).ctr();
    std::size_t dst_size = static_cast<std::size_t>(n) * sizeof(int);
    std::size_t src_size = sizeof(typename ::vsmc::RNG::ctr_type);
    std::size_t size = src_size < dst_size ? src_size : dst_size;
    std::memcpy(ctr, c.data(), size);
}

void vsmc_rng_set_ctr(vsmc_rng rng, int n, const int *ctr)
{
    typename ::vsmc::RNG::ctr_type c;
    std::size_t src_size = static_cast<std::size_t>(n) * sizeof(int);
    std::size_t dst_size = sizeof(typename ::vsmc::RNG::ctr_type);
    std::size_t size = src_size < dst_size ? src_size : dst_size;
    std::memcpy(c.data(), ctr, size);
    ::vsmc::cast(rng).ctr(c);
}

int vsmc_rng_rand1(vsmc_rng rng)
{
    return static_cast<int>(::vsmc::cast(rng)());
}

void vsmc_rng_rand(vsmc_rng rng, int n, int *r)
{
    ::vsmc::rng_rand(::vsmc::cast(rng), static_cast<std::size_t>(n),
        reinterpret_cast<unsigned *>(r));
}

void vsmc_rng_discard(vsmc_rng rng, int nskip)
{
    ::vsmc::cast(rng).discard(static_cast<::vsmc::RNG::result_type>(nskip));
}

int vsmc_rng_is_eq(vsmc_rng rng1, vsmc_rng rng2)
{
    return static_cast<int>(::vsmc::cast(rng1) == ::vsmc::cast(rng2));
}

int vsmc_rng_is_neq(vsmc_rng rng1, vsmc_rng rng2)
{
    return static_cast<int>(::vsmc::cast(rng1) != ::vsmc::cast(rng2));
}

int vsmc_rng_save(vsmc_rng rng, void *mem)
{
    std::size_t size = sizeof(::vsmc::RNG);
    if (mem != nullptr)
        std::memcpy(mem, rng.ptr, size);

    return static_cast<int>(size);
}

void vsmc_rng_load(vsmc_rng rng, void *mem)
{
    std::memcpy(rng.ptr, mem, sizeof(::vsmc::RNG));
}

void vsmc_rng_save_f(vsmc_rng rng, const char *filename)
{
    std::ofstream os(filename);
    os << ::vsmc::cast(rng) << std::endl;
    os.close();
}

void vsmc_rng_load_f(vsmc_rng rng, const char *filename)
{
    std::ifstream is(filename);
    is >> ::vsmc::cast(rng);
    is.close();
}

} // extern "C"
