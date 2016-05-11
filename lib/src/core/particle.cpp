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

#include <vsmc/core/core.h>
#include "libvsmc.hpp"

extern "C" {

vsmc_particle vsmc_particle_new(size_t n, size_t dim)
{
    return {new ::vsmc::ParticleC(n, dim)};
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

vsmc_particle vsmc_particle_clone(vsmc_particle particle)
{
    vsmc_particle clone = vsmc_particle_new(0, 0);
    ::vsmc::cast(clone) = ::vsmc::cast(particle).clone();

    return clone;
}

size_t vsmc_particle_size(vsmc_particle particle)
{
    return ::vsmc::cast(particle).size();
}

void vsmc_particle_resize_by_index(
    vsmc_particle particle, size_t n, const size_t *index)
{
    ::vsmc::cast(particle).resize_by_index(n, index);
}

void vsmc_particle_resize_by_resample(
    vsmc_particle particle, size_t n, vSMCResampleScheme scheme)
{
    switch (scheme) {
        case vSMCMultinomial:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleMultinomial());
            break;
        case vSMCResidual:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleResidual());
            break;
        case vSMCStratified:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleStratified());
            break;
        case vSMCSystematic:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleSystematic());
            break;
        case vSMCResidualStratified:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleResidualStratified());
            break;
        case vSMCResidualSystematic:
            ::vsmc::cast(particle).resize_by_resample(
                n, ::vsmc::ResampleResidualSystematic());
            break;
    }
}

void vsmc_particle_resize_by_uniform(vsmc_particle particle, size_t n)
{
    ::vsmc::cast(particle).resize_by_uniform(n);
}

vsmc_state_matrix vsmc_particle_state(vsmc_particle particle)
{
    return {&::vsmc::cast(particle).state()};
}

vsmc_weight vsmc_particle_weight(vsmc_particle particle)
{
    return {&::vsmc::cast(particle).weight()};
}

vsmc_rng vsmc_particle_rng(vsmc_particle particle, size_t id)
{
    return {&::vsmc::cast(particle).rng(id),
        ::vsmc::rng_type<::vsmc::ParticleC::rng_type>()};
}

vsmc_single_particle vsmc_particle_sp(vsmc_particle particle, size_t id)
{
    return {::vsmc::cast(particle).state().row_data(id), id};
}

} // extern "C"
