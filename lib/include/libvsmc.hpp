//============================================================================
// vSMC/lib/src/libvsmc.hpp
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

#ifndef VSMC_LIBVSMC_HPP
#define VSMC_LIBVSMC_HPP

#include <vsmc/vsmc.h>
#include <vsmc/vsmc.hpp>

namespace vsmc
{

using StateMatrixCBase = StateMatrix<RowMajor, Dynamic, double>;

class StateMatrixC : public StateMatrixCBase
{
    public:
    using size_type = int;

    StateMatrixC(int N) : StateMatrixCBase(static_cast<std::size_t>(N)) {}
}; // class StateMatrixC

using ParticleC = Particle<StateMatrixC>;

using SamplerC = Sampler<StateMatrixC>;

using MonitorC = Monitor<StateMatrixC>;

inline RNG &cast(vsmc_rng *rng_ptr)
{
    return *(reinterpret_cast<RNG *>(rng_ptr->ptr));
}

inline const RNG &cast(const vsmc_rng *rng_ptr)
{
    return *(reinterpret_cast<const RNG *>(rng_ptr->ptr));
}

inline StateMatrixC &cast(vsmc_state_matrix *state_matrix_ptr)
{
    return *(reinterpret_cast<StateMatrixC *>(state_matrix_ptr->ptr));
}

inline const StateMatrixC &cast(const vsmc_state_matrix *state_matrix_ptr)
{
    return *(reinterpret_cast<StateMatrixC *>(state_matrix_ptr->ptr));
}

inline Weight &cast(vsmc_weight *weight_ptr)
{
    return *(reinterpret_cast<Weight *>(weight_ptr->ptr));
}

inline const Weight &cast(const vsmc_weight *weight_ptr)
{
    return *(reinterpret_cast<const Weight *>(weight_ptr->ptr));
}

inline ParticleC &cast(vsmc_particle *particle_ptr)
{
    return *(reinterpret_cast<ParticleC *>(particle_ptr->ptr));
}

inline const ParticleC &cast(const vsmc_particle *particle_ptr)
{
    return *(reinterpret_cast<ParticleC *>(particle_ptr->ptr));
}

inline MonitorC &cast(vsmc_monitor *monitor_ptr)
{
    return *(reinterpret_cast<MonitorC *>(monitor_ptr->ptr));
}

inline const MonitorC &cast(const vsmc_monitor *monitor_ptr)
{
    return *(reinterpret_cast<MonitorC *>(monitor_ptr->ptr));
}

inline SamplerC &cast(vsmc_sampler *sampler_ptr)
{
    return *(reinterpret_cast<SamplerC *>(sampler_ptr->ptr));
}

inline const SamplerC &cast(const vsmc_sampler *sampler_ptr)
{
    return *(reinterpret_cast<SamplerC *>(sampler_ptr->ptr));
}

} // namespace vsmc

#endif // VSMC_LIBVSMC_HPP
