//============================================================================
// vSMC/lib/src/smp/smp.cpp
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
// CONOMPUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include <vsmc/smp/smp.h>
#include <vsmc/smp/smp.hpp>
#include "libvsmc.hpp"

namespace vsmc
{

template <typename Backend>
class SamplerEvalSMPC
    : public SamplerEvalSMP<StateMatrixC, SamplerEvalSMPC<Backend>, Backend>
{
    public:
    SamplerEvalSMPC(const vsmc_sampler_eval_smp_type &work) : work_(work) {}

    std::size_t eval(std::size_t iter, ParticleIndexC idx)
    {
        if (work_.eval == nullptr)
            return 0;

        vsmc_particle_index idx_c = {&idx(0), idx.id()};

        return work_.eval(iter, idx_c);
    }

    void eval_pre(std::size_t iter, ParticleC &particle)
    {
        if (work_.eval_pre == nullptr)
            return;

        vsmc_particle particle_c = {&particle};
        work_.eval_pre(iter, particle_c);
    }

    void eval_post(std::size_t iter, ParticleC &particle)
    {
        if (work_.eval_post == nullptr)
            return;

        vsmc_particle particle_c = {&particle};
        work_.eval_post(iter, particle_c);
    }

    private:
    vsmc_sampler_eval_smp_type work_;
}; // class SamplerEvalSMPC

template <typename Backend>
class MonitorEvalSMPC
    : public MonitorEvalSMP<StateMatrixC, MonitorEvalSMPC<Backend>, Backend>
{
    public:
    MonitorEvalSMPC(const vsmc_monitor_eval_smp_type &work) : work_(work) {}

    void eval(std::size_t iter, size_t dim, ParticleIndexC idx, double *r)
    {
        if (work_.eval == nullptr)
            return;

        vsmc_particle_index idx_c = {&idx(0), idx.id()};

        work_.eval(iter, dim, idx_c, r);
    }

    void eval_pre(std::size_t iter, ParticleC &particle)
    {
        if (work_.eval_pre == nullptr)
            return;

        vsmc_particle particle_c = {&particle};
        work_.eval_pre(iter, particle_c);
    }

    void eval_post(std::size_t iter, ParticleC &particle)
    {
        if (work_.eval_post == nullptr)
            return;

        vsmc_particle particle_c = {&particle};
        work_.eval_post(iter, particle_c);
    }

    private:
    vsmc_monitor_eval_smp_type work_;
}; // class MonitorEvalSMPC

} // namespace vsmc

#define VSMC_DEFINE_LIB_SMP(Name, name)                                       \
    inline void vsmc_sampler_eval_##name(vsmc_sampler sampler,                \
        vsmc_sampler_eval_smp_type new_eval, vSMCSamplerStage stage,          \
        int append)                                                           \
    {                                                                         \
        ::vsmc::cast(sampler).eval(                                           \
            ::vsmc::SamplerEvalSMPC<::vsmc::Backend##Name>(new_eval),         \
            static_cast<::vsmc::SamplerStage>(stage), append != 0);           \
    }                                                                         \
                                                                              \
    inline vsmc_monitor vsmc_monitor_new_##name(size_t dim,                   \
        vsmc_monitor_eval_smp_type eval, int record_only,                     \
        vSMCMonitorStage stage)                                               \
    {                                                                         \
        return {new ::vsmc::MonitorC(dim,                                     \
            ::vsmc::MonitorEvalSMPC<::vsmc::Backend##Name>(eval),             \
            record_only != 0, static_cast<::vsmc::MonitorStage>(stage))};     \
    }

extern "C" {

VSMC_DEFINE_LIB_SMP(SEQ, seq)
VSMC_DEFINE_LIB_SMP(STD, std)
#if VSMC_HAS_OMP
VSMC_DEFINE_LIB_SMP(OMP, omp)
#endif
#if VSMC_HAS_TBB
VSMC_DEFINE_LIB_SMP(TBB, tbb)
#endif

static int vsmc_backend_smp_table[] = {1, 1,
#if VSMC_HAS_OMP
    1,
#else
    0,
#endif
#if VSMC_HAS_TBB
    1,
#else
    0,
#endif
    0}; // vsmc_backend_smp

int vsmc_backend_smp_max()
{
    return static_cast<int>(sizeof(vsmc_backend_smp_table) / sizeof(int)) - 1;
}

int vsmc_backend_smp_check(vSMCBackendSMP backend)
{
    if (static_cast<int>(backend) < 0)
        return 0;

    if (static_cast<int>(backend) > vsmc_backend_smp_max())
        return 0;

    return vsmc_backend_smp_table[static_cast<std::size_t>(backend)];
}

using vsmc_sampler_eval_smp_dispatch_type = void (*)(
    vsmc_sampler, vsmc_sampler_eval_smp_type, vSMCSamplerStage, int);

static vsmc_sampler_eval_smp_dispatch_type vsmc_sampler_eval_smp_dispatch[] = {
    vsmc_sampler_eval_seq, vsmc_sampler_eval_std,
#if VSMC_HAS_OMP
    vsmc_sampler_eval_omp,
#else
    nullptr,
#endif
#if VSMC_HAS_TBB
    vsmc_sampler_eval_tbb,
#else
    nullptr,
#endif
    nullptr}; // vsmc_sampler_eval_smp_dispatch

void vsmc_sampler_eval_smp(vSMCBackendSMP backend, vsmc_sampler sampler,
    vsmc_sampler_eval_smp_type new_eval, vSMCSamplerStage stage, int append)
{
    vsmc_sampler_eval_smp_dispatch[static_cast<std::size_t>(backend)](
        sampler, new_eval, stage, append);
}

using vsmc_monitor_new_smp_dispatch_type = vsmc_monitor (*)(
    size_t, vsmc_monitor_eval_smp_type, int, vSMCMonitorStage);

static vsmc_monitor_new_smp_dispatch_type vsmc_monitor_new_smp_dispatch[] = {
    vsmc_monitor_new_seq, vsmc_monitor_new_std,
#if VSMC_HAS_OMP
    vsmc_monitor_new_omp,
#else
    nullptr,
#endif
#if VSMC_HAS_TBB
    vsmc_monitor_new_tbb,
#else
    nullptr,
#endif
    nullptr}; // vsmc_monitor_new_smp_dispatch

vsmc_monitor vsmc_monitor_new_smp(vSMCBackendSMP backend, size_t dim,
    vsmc_monitor_eval_smp_type eval, int record_only, vSMCMonitorStage stage)
{
    return vsmc_monitor_new_smp_dispatch[static_cast<std::size_t>(backend)](
        dim, eval, record_only, stage);
}

} // extern "C"
