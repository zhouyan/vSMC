//============================================================================
// vSMC/lib/src/core/particle.cpp
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

vsmc_particle vsmc_particle_new(int n, int dim)
{
    auto ptr = new ::vsmc::ParticleC(n);
    ptr->value().resize_dim(static_cast<std::size_t>(dim));
    vsmc_particle particle = {ptr};

    return particle;
}

void vsmc_particle_delete(vsmc_particle *particle_ptr)
{
    delete ::vsmc::cast(particle_ptr);
    particle_ptr->ptr = nullptr;
}

void vsmc_particle_assign(vsmc_particle particle, vsmc_particle other)
{
    ::vsmc::cast(particle) = ::vsmc::cast(other);
}

void vsmc_particle_clone(
    vsmc_particle particle, vsmc_particle other, int retain_rng)
{
    ::vsmc::cast(particle).clone(::vsmc::cast(other), retain_rng != 0);
}

int vsmc_particle_size(vsmc_particle particle)
{
    return ::vsmc::cast(particle).size();
}

void vsmc_particle_resize_by_index(vsmc_particle particle, int N, int *index)
{
    ::vsmc::cast(particle).resize_by_index(N, index);
}

void vsmc_particle_resize_by_mask(vsmc_particle particle, int N, int *mask)
{
    ::vsmc::cast(particle).resize_by_mask(N, mask);
}

void vsmc_particle_resize_by_resample(
    vsmc_particle particle, int N, vsmc_resample_type op)
{
    ::vsmc::cast(particle).resize_by_resample(N, ::vsmc::cast(op));
}

void vsmc_particle_resize_by_uniform(vsmc_particle particle, int N)
{
    ::vsmc::cast(particle).resize_by_uniform(N);
}

void vsmc_particle_resize_by_ragne(
    vsmc_particle particle, int N, int first, int last)
{
    ::vsmc::cast(particle).resize_by_range(N, first, last);
}

vsmc_state_matrix vsmc_particle_value(vsmc_particle particle)
{
    vsmc_state_matrix value = {&::vsmc::cast(particle).value()};

    return value;
}

vsmc_weight vsmc_particle_weight(vsmc_particle particle)
{
    vsmc_weight weight = {&::vsmc::cast(particle).weight()};

    return weight;
}

vsmc_rng vsmc_particle_rng(vsmc_particle particle, int id)
{
    vsmc_rng rng = {&::vsmc::cast(particle).rng(id)};

    return rng;
}

vsmc_single_particle vsmc_particle_sp(vsmc_particle particle, int id)
{
    vsmc_single_particle sp = {
        ::vsmc::cast(particle).value().row_data(static_cast<std::size_t>(id)),
        id};

    return sp;
}

} // extern "C"
