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

inline RNG &cast(const vsmc_rng &rng)
{
    return *(reinterpret_cast<RNG *>(rng.ptr));
}

inline RNG *cast(const vsmc_rng *rng_ptr)
{
    return reinterpret_cast<RNG *>(rng_ptr->ptr);
}

inline StateMatrixC &cast(const vsmc_state_matrix &state_matrix)
{
    return *(reinterpret_cast<StateMatrixC *>(state_matrix.ptr));
}

inline StateMatrixC *cast(const vsmc_state_matrix *state_matrix_ptr)
{
    return reinterpret_cast<StateMatrixC *>(state_matrix_ptr->ptr);
}

inline Weight &cast(const vsmc_weight &weight)
{
    return *(reinterpret_cast<Weight *>(weight.ptr));
}

inline Weight *cast(const vsmc_weight *weight_ptr)
{
    return reinterpret_cast<Weight *>(weight_ptr->ptr);
}

inline ParticleC &cast(const vsmc_particle &particle)
{
    return *(reinterpret_cast<ParticleC *>(particle.ptr));
}

inline ParticleC *cast(const vsmc_particle *particle_ptr)
{
    return reinterpret_cast<ParticleC *>(particle_ptr->ptr);
}

inline MonitorC &cast(const vsmc_monitor &monitor)
{
    return *(reinterpret_cast<MonitorC *>(monitor.ptr));
}

inline MonitorC *cast(const vsmc_monitor *monitor_ptr)
{
    return reinterpret_cast<MonitorC *>(monitor_ptr->ptr);
}

inline SamplerC &cast(const vsmc_sampler &sampler)
{
    return *(reinterpret_cast<SamplerC *>(sampler.ptr));
}

inline SamplerC *cast(const vsmc_sampler *sampler_ptr)
{
    return reinterpret_cast<SamplerC *>(sampler_ptr->ptr);
}

inline ParticleC::resample_type cast(vsmc_particle_resample_type fptr)
{
    return [fptr](
        std::size_t M, std::size_t N, RNG &rng, const double *w, int *rep) {
        vsmc_rng c_rng = {&rng};
        fptr(static_cast<int>(M), static_cast<int>(N), c_rng, w, rep);
    };
}

inline MonitorC::eval_type cast(vsmc_monitor_eval_type fptr)
{
    return [fptr](
        std::size_t iter, std::size_t dim, ParticleC &particle, double *r) {
        vsmc_particle c_particle = {&particle};
        fptr(static_cast<int>(iter), static_cast<int>(dim), c_particle, r);
    };
}

inline SamplerC::init_type cast(vsmc_sampler_init_type fptr)
{
    return [fptr](ParticleC &particle, void *param) {
        vsmc_particle c_particle = {&particle};
        return static_cast<std::size_t>(fptr(c_particle, param));
    };
}

inline SamplerC::move_type cast(vsmc_sampler_move_type fptr)
{
    return [fptr](std::size_t iter, ParticleC &particle) {
        vsmc_particle c_particle = {&particle};
        return static_cast<std::size_t>(
            fptr(static_cast<int>(iter), c_particle));
    };
}

} // namespace vsmc

#endif // VSMC_LIBVSMC_HPP
