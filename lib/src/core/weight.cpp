//============================================================================
// vSMC/lib/src/core/weight.cpp
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

vsmc_weight vsmc_weight_new(int n)
{
    auto ptr = new ::vsmc::Weight(static_cast<std::size_t>(n));
    vsmc_weight weight = {ptr};

    return weight;
}

void vsmc_weight_delete(vsmc_weight *weight_ptr)
{
    delete ::vsmc::cast(weight_ptr);
    weight_ptr->ptr = nullptr;
}

void vsmc_weight_assign(vsmc_weight weight, vsmc_weight other)
{
    ::vsmc::cast(weight) = ::vsmc::cast(other);
}

int vsmc_weight_size(vsmc_weight weight)
{
    return static_cast<int>(::vsmc::cast(weight).size());
}

void vsmc_weight_resize(vsmc_weight weight, int N)
{
    ::vsmc::cast(weight).resize(static_cast<std::size_t>(N));
}

void vsmc_weight_reserve(vsmc_weight weight, int N)
{
    ::vsmc::cast(weight).reserve(static_cast<std::size_t>(N));
}

void vsmc_weight_shrink_to_fit(vsmc_weight weight)
{
    ::vsmc::cast(weight).shrink_to_fit();
}

double vsmc_weight_ess(vsmc_weight weight)
{
    return ::vsmc::cast(weight).ess();
}

void vsmc_weight_set_equal(vsmc_weight weight)
{
    ::vsmc::cast(weight).set_equal();
}

void vsmc_weight_set(vsmc_weight weight, const double *first, int stride)
{
    ::vsmc::cast(weight).set(first, stride);
}

void vsmc_weight_mul(vsmc_weight weight, const double *first, int stride)
{
    ::vsmc::cast(weight).mul(first, stride);
}

void vsmc_weight_set_log(vsmc_weight weight, const double *first, int stride)
{
    ::vsmc::cast(weight).set_log(first, stride);
}

void vsmc_weight_add_log(vsmc_weight weight, const double *first, int stride)
{
    ::vsmc::cast(weight).add_log(first, stride);
}

int vsmc_weight_draw(vsmc_weight weight, vsmc_rng rng)
{
    return static_cast<int>(::vsmc::cast(weight).draw(::vsmc::cast(rng)));
}

} // extern "C"
