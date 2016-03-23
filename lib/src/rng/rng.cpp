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

void vsmc_rng_malloc(vsmc_rng *rng_ptr, int seed)
{
    auto ptr = ::vsmc::Allocator<::vsmc::RNG>::allocate(1);
    new (ptr)::vsmc::RNG(static_cast<::vsmc::RNG::result_type>(seed));
    rng_ptr->ptr = ptr;
}

void vsmc_rng_free(vsmc_rng *rng_ptr)
{
    ::vsmc::Allocator<::vsmc::RNG>::deallocate(::vsmc::cast(rng_ptr), 1);
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

void vsmc_rng_rand(vsmc_rng rng, int n, int *r)
{
    ::vsmc::rng_rand(::vsmc::cast(rng), static_cast<std::size_t>(n),
        reinterpret_cast<unsigned *>(r));
}

} // extern "C"
