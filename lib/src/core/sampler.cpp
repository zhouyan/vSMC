//============================================================================
// vSMC/lib/src/core/sampler.cpp
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

vsmc_sampler vsmc_sampler_new(
    int n, int dim, vSMCResampleScheme scheme, double threshold)
{
    auto ptr = new ::vsmc::SamplerC(
        n, static_cast<::vsmc::ResampleScheme>(scheme), threshold);
    ptr->particle().value().resize_dim(static_cast<std::size_t>(dim));
    vsmc_sampler sampler = {ptr};

    return sampler;
}

void vsmc_sampler_delete(vsmc_sampler *sampler_ptr)
{
    delete ::vsmc::cast(sampler_ptr);
    sampler_ptr->ptr = nullptr;
}

void vsmc_sampler_assign(vsmc_sampler sampler, vsmc_sampler other)
{
    ::vsmc::cast(sampler) = ::vsmc::cast(other);
}

void vsmc_sampler_clone(
    vsmc_sampler sampler, vsmc_sampler other, int retain_rng)
{
    ::vsmc::cast(sampler).clone(::vsmc::cast(other), retain_rng != 0);
}

int vsmc_sampler_size(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).size());
}

void vsmc_sampler_reserve(vsmc_sampler sampler, int num)
{
    ::vsmc::cast(sampler).reserve(static_cast<std::size_t>(num));
}

int vsmc_sampler_iter_size(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).iter_size());
}

int vsmc_sampler_iter_num(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).iter_num());
}

int vsmc_sampler_accept_size(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).accept_size());
}

void vsmc_sampler_resample(vsmc_sampler sampler)
{
    ::vsmc::cast(sampler).resample();
}

void vsmc_sampler_resample_scheme_f(
    vsmc_sampler sampler, vsmc_particle_resample_type op)
{
    ::vsmc::cast(sampler).resample_scheme(::vsmc::cast(op));
}

void vsmc_sampler_resample_scheme(
    vsmc_sampler sampler, vSMCResampleScheme scheme)
{
    ::vsmc::cast(sampler).resample_scheme(
        static_cast<::vsmc::ResampleScheme>(scheme));
}

double vsmc_sampler_get_threshold(vsmc_sampler sampler)
{
    return ::vsmc::cast(sampler).resample_threshold();
}

void vsmc_sampler_set_threshold(vsmc_sampler sampler, double threshold)
{
    ::vsmc::cast(sampler).resample_threshold(threshold);
}

double vsmc_sampler_resample_threshold_never()
{
    return ::vsmc::SamplerC::resample_threshold_never();
}

double vsmc_sampler_resample_threshold_always()
{
    return ::vsmc::SamplerC::resample_threshold_always();
}

int vsmc_sampler_size_history(vsmc_sampler sampler, int iter)
{
    return static_cast<int>(
        ::vsmc::cast(sampler).size_history(static_cast<std::size_t>(iter)));
}

int *vsmc_sampler_read_size_history(vsmc_sampler sampler, int *first)
{
    return ::vsmc::cast(sampler).read_size_history(first);
}

double vsmc_sampler_ess_history(vsmc_sampler sampler, int iter)
{
    return ::vsmc::cast(sampler).ess_history(static_cast<std::size_t>(iter));
}

double *vsmc_sampler_read_ess_history(vsmc_sampler sampler, double *first)
{
    return ::vsmc::cast(sampler).read_ess_history(first);
}

int vsmc_sampler_resampled_history(vsmc_sampler sampler, int iter)
{
    return ::vsmc::cast(sampler).resampled_history(
        static_cast<std::size_t>(iter));
}

int *vsmc_sampler_read_resampled_history(vsmc_sampler sampler, int *first)
{
    return ::vsmc::cast(sampler).read_resampled_history(first);
}

int vsmc_sampler_accept_history(vsmc_sampler sampler, int id, int iter)
{
    return static_cast<int>(::vsmc::cast(sampler).accept_history(
        static_cast<std::size_t>(id), static_cast<std::size_t>(iter)));
}

int *vsmc_sampler_read_accept_history(vsmc_sampler sampler, int id, int *first)
{
    return ::vsmc::cast(sampler).read_accept_history(
        static_cast<std::size_t>(id), first);
}

int *const *vsmc_sampler_read_accept_history_list(
    vsmc_sampler sampler, int *const *first)
{
    return ::vsmc::cast(sampler).read_accept_history_list(first);
}

int *vsmc_sampler_read_accept_history_matrix(
    vsmc_sampler sampler, vSMCMatrixLayout layout, int *first)
{
    return ::vsmc::cast(sampler).read_accept_history_matrix(
        static_cast<::vsmc::MatrixLayout>(layout), first);
}

vsmc_particle vsmc_sampler_particle(vsmc_sampler sampler)
{
    vsmc_particle particle = {&::vsmc::cast(sampler).particle()};

    return particle;
}

void vsmc_sampler_init(vsmc_sampler sampler, vsmc_sampler_init_type new_init)
{
    ::vsmc::cast(sampler).init(::vsmc::cast(new_init));
}

void vsmc_sampler_init_by_iter(vsmc_sampler sampler, int initialize_by_iterate)
{
    ::vsmc::cast(sampler).init_by_iter(initialize_by_iterate != 0);
}

void vsmc_sampler_init_by_move(
    vsmc_sampler sampler, vsmc_sampler_move_type new_init)
{
    ::vsmc::cast(sampler).init_by_move(::vsmc::cast(new_init));
}

void vsmc_sampler_move_queue_clear(vsmc_sampler sampler)
{
    ::vsmc::cast(sampler).move_queue_clear();
}

int vsmc_sampler_move_queue_empty(vsmc_sampler sampler)
{
    return ::vsmc::cast(sampler).move_queue_empty();
}

int vsmc_sampler_move_queue_size(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).move_queue_size());
}

void vsmc_sampler_move(
    vsmc_sampler sampler, vsmc_sampler_move_type new_move, int append)
{
    ::vsmc::cast(sampler).move(::vsmc::cast(new_move), append != 0);
}

void vsmc_sampler_mcmc_queue_clear(vsmc_sampler sampler)
{
    ::vsmc::cast(sampler).mcmc_queue_clear();
}

int vsmc_sampler_mcmc_queue_empty(vsmc_sampler sampler)
{
    return ::vsmc::cast(sampler).mcmc_queue_empty();
}

int vsmc_sampler_mcmc_queue_size(vsmc_sampler sampler)
{
    return static_cast<int>(::vsmc::cast(sampler).mcmc_queue_size());
}

void vsmc_sampler_mcmc(
    vsmc_sampler sampler, vsmc_sampler_mcmc_type new_mcmc, int append)
{
    ::vsmc::cast(sampler).mcmc(::vsmc::cast(new_mcmc), append != 0);
}

void vsmc_sampler_initialize(vsmc_sampler sampler, void *param)
{
    ::vsmc::cast(sampler).initialize(param);
}

void vsmc_sampler_iterate(vsmc_sampler sampler, int num)
{
    ::vsmc::cast(sampler).iterate(static_cast<std::size_t>(num));
}

void vsmc_sampler_set_monitor_direct(
    vsmc_sampler sampler, const char *name, vsmc_monitor mon)
{
    ::vsmc::cast(sampler).monitor(name, ::vsmc::cast(mon));
}

void vsmc_sampler_set_monitor(vsmc_sampler sampler, const char *name, int dim,
    vsmc_monitor_eval_type eval, int record_only, vSMCMonitorStage stage)
{
    ::vsmc::cast(sampler).monitor(name, static_cast<std::size_t>(dim),
        ::vsmc::cast(eval), record_only != 0,
        static_cast<vsmc::MonitorStage>(stage));
}

vsmc_monitor vsmc_sampler_get_monitor(vsmc_sampler sampler, const char *name)
{
    vsmc_monitor monitor = {&::vsmc::cast(sampler).monitor(name)};

    return monitor;
}

void vsmc_sampler_clear_monitor(vsmc_sampler sampler, const char *name)
{
    ::vsmc::cast(sampler).clear_monitor(name);
}

void vsmc_sampler_clear_monitor_all(vsmc_sampler sampler)
{
    ::vsmc::cast(sampler).clear_monitor();
}

int vsmc_sampler_save(const vsmc_sampler sampler, char *mem)
{
    std::stringstream ss;
    ss << ::vsmc::cast(sampler);
    std::string str(ss.str());
    std::size_t size = (str.size() + 1) * sizeof(char);
    if (mem != nullptr)
        std::memcpy(mem, str.c_str(), size);

    return static_cast<int>(size);
}

void vsmc_sampler_save_f(const vsmc_sampler sampler, const char *filename)
{
    std::ofstream os(filename);
    os << ::vsmc::cast(sampler);
    os.close();
}

} // extern "C"
