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

void vsmc_weight_malloc(vsmc_weight *weight_ptr, int n)
{
    ::vsmc::Weight *ptr = new Weight(static_cast<std::size_t>(n));
    weight_ptr->ptr = ptr;
}

void vsmc_weight_free(vsmc_weight *weight_ptr)
{
    delete &::vsmc::cast(weight_ptr);
    weight_ptr->ptr = nullptr;
}

int vsmc_weight_size(const vsmc_weight *weight_ptr)
{
    return static_cast<int>(::vsmc::cast(weight_ptr).size());
}

double vsmc_weight_ess(const vsmc_weight *weight_ptr)
{
    return ::vsmc::cast(weight_ptr).ess();
}

const double *vsmc_weight_data(const vsmc_weight *weight_ptr)
{
    return ::vsmc::cast(weight_ptr).data();
}

void vsmc_weight_set_equal(vsmc_weight *weight_ptr)
{
    ::vsmc::cast(weight_ptr).set_equal();
}

void vsmc_weight_set(vsmc_weight *weight_ptr, const double *first, int stride)
{
    ::vsmc::cast(weight_ptr).set(first, stride);
}

void vsmc_weight_mul(vsmc_weight *weight_ptr, const double *first, int stride)
{
    ::vsmc::cast(weight_ptr).mul(first, stride);
}

void vsmc_weight_set_log(
    vsmc_weight *weight_ptr, const double *first, int stride)
{
    ::vsmc::cast(weight_ptr).set_log(first, stride);
}

void vsmc_weight_add_log(
    vsmc_weight *weight_ptr, const double *first, int stride)
{
    ::vsmc::cast(weight_ptr).add_log(first, stride);
}

int vsmc_weight_draw(vsmc_weight *weight_ptr, vsmc_rng *rng_ptr)
{
    return static_cast<int>(
        ::vsmc::cast(weight_ptr).draw(::vsmc::cast(rng_ptr)));
}
