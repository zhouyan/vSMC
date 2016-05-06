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

#include <vsmc/internal/common.h>
#include <vsmc/vsmc.hpp>

namespace vsmc
{

using WeightC = Weight;

using StateMatrixC = StateMatrix<RowMajor, Dynamic, double>;

using ParticleC = Particle<StateMatrixC>;

using SingleParticleC = SingleParticle<StateMatrixC>;

using SamplerC = Sampler<StateMatrixC>;

using MonitorC = Monitor<StateMatrixC>;

inline WeightC &cast(const vsmc_weight &weight)
{
    return *(reinterpret_cast<WeightC *>(weight.ptr));
}

inline WeightC *cast(const vsmc_weight *weight_ptr)
{
    return reinterpret_cast<WeightC *>(weight_ptr->ptr);
}

inline StateMatrixC &cast(const vsmc_state_matrix &state_matrix)
{
    return *(reinterpret_cast<StateMatrixC *>(state_matrix.ptr));
}

inline StateMatrixC *cast(const vsmc_state_matrix *state_matrix_ptr)
{
    return reinterpret_cast<StateMatrixC *>(state_matrix_ptr->ptr);
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

template <typename>
inline constexpr vSMCRNGType rng_type();

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#ifdef VSMC_RNG_DEFINE_MACRO_NA
#undef VSMC_RNG_DEFINE_MACRO_NA
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, Name, name)                            \
    template <>                                                               \
    inline constexpr vSMCRNGType rng_type<RNGType>()                          \
    {                                                                         \
        return vSMC##Name;                                                    \
    }

#include <vsmc/rng/internal/rng_define_macro.hpp>

} // namespace vsmc

#endif // VSMC_LIBVSMC_HPP
