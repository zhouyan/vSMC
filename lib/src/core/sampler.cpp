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

void vsmc_sampler_malloc(vsmc_sampler *sampler_ptr, int n, int dim,
    vSMCResampleScheme scheme, double threshold)
{
    auto ptr = ::vsmc::AlignedAllocator<::vsmc::SamplerC>::allocate(1);
    new (ptr)::vsmc::SamplerC(
        n, static_cast<::vsmc::ResampleScheme>(scheme), threshold);
    ptr->particle().value().resize_dim(static_cast<std::size_t>(dim));
    sampler_ptr->ptr = ptr;
    sampler_ptr->particle.ptr = &ptr->particle();
}

void vsmc_sampler_free(vsmc_sampler *sampler_ptr)
{
    ::vsmc::AlignedAllocator<::vsmc::SamplerC>::deallocate(
        &::vsmc::cast(sampler_ptr), 1);
    sampler_ptr->ptr = nullptr;
}

void vsmc_sampler_assign(vsmc_sampler *dst, const vsmc_sampler *src)
{
    ::vsmc::cast(dst) = ::vsmc::cast(src);
}

void vsmc_sampler_clone(
    vsmc_sampler *sampler_ptr, const vsmc_sampler *other, int retain_rng)
{
    ::vsmc::cast(sampler_ptr).clone(::vsmc::cast(other), retain_rng != 0);
}

int vsmc_sampler_size(const vsmc_sampler *sampler_ptr)
{
    return static_cast<int>(::vsmc::cast(sampler_ptr).size());
}

void vsmc_sampler_reserve(vsmc_sampler *sampler_ptr, int num)
{
    ::vsmc::cast(sampler_ptr).reserve(static_cast<std::size_t>(num));
}

int vsmc_sampler_iter_size(const vsmc_sampler *sampler_ptr)
{
    return static_cast<int>(::vsmc::cast(sampler_ptr).iter_size());
}

int vsmc_sampler_iter_num(const vsmc_sampler *sampler_ptr)
{
    return static_cast<int>(::vsmc::cast(sampler_ptr).iter_num());
}

void vsmc_sampler_resample(vsmc_sampler *sampler_ptr)
{
    ::vsmc::cast(sampler_ptr).resample();
}

void vsmc_sampler_resample_op(
    vsmc_sampler *sampler_ptr, vsmc_particle_resample_type op)
{
    auto cpp_op = [op](std::size_t M, std::size_t N, ::vsmc::RNG &cpp_rng,
        const double *weight, int *rep) {
        vsmc_rng rng = {&cpp_rng};
        op(static_cast<int>(M), static_cast<int>(N), &rng, weight, rep);
    };

    ::vsmc::cast(sampler_ptr).resample_scheme(cpp_op);
}

void vsmc_sampler_resample_scheme(
    vsmc_sampler *sampler_ptr, vSMCResampleScheme scheme)
{
    ::vsmc::cast(sampler_ptr)
        .resample_scheme(static_cast<::vsmc::ResampleScheme>(scheme));
}

double vsmc_sampler_get_threshold(const vsmc_sampler *sampler_ptr)
{
    return ::vsmc::cast(sampler_ptr).resample_threshold();
}

void vsmc_sampler_set_threshold(vsmc_sampler *sampler_ptr, double threshold)
{
    ::vsmc::cast(sampler_ptr).resample_threshold(threshold);
}

double vsmc_sampler_resample_threshold_never()
{
    return ::vsmc::SamplerC::resample_threshold_never();
}

double vsmc_sampler_resample_threshold_always()
{
    return ::vsmc::SamplerC::resample_threshold_always();
}

void vsmc_sampler_init(
    vsmc_sampler *sampler_ptr, vsmc_sampler_init_type new_init)
{
    auto cpp_new_init = [new_init](
        ::vsmc::ParticleC &cpp_particle, void *param) {
        vsmc_particle particle = {
            &cpp_particle, {&cpp_particle.value()}, {&cpp_particle.weight()}};
        return static_cast<std::size_t>(new_init(&particle, param));
    };
    ::vsmc::cast(sampler_ptr).init(cpp_new_init);
}

void vsmc_sampler_init_by_iter(
    vsmc_sampler *sampler_ptr, int initialize_by_iterate)
{
    ::vsmc::cast(sampler_ptr).init_by_iter(initialize_by_iterate != 0);
}

void vsmc_sampler_init_by_move(
    vsmc_sampler *sampler_ptr, vsmc_sampler_move_type new_init)
{
    auto cpp_new_init = [new_init](
        std::size_t iter, ::vsmc::ParticleC &cpp_particle) {
        vsmc_particle particle = {
            &cpp_particle, {&cpp_particle.value()}, {&cpp_particle.weight()}};
        return static_cast<std::size_t>(
            new_init(static_cast<int>(iter), &particle));
    };
    ::vsmc::cast(sampler_ptr).init_by_move(cpp_new_init);
}

void vsmc_sampler_move_queue_clear(vsmc_sampler *sampler_ptr)
{
    ::vsmc::cast(sampler_ptr).move_queue_clear();
}

int vsmc_sampler_move_queue_empty(const vsmc_sampler *sampler_ptr)
{
    return ::vsmc::cast(sampler_ptr).move_queue_empty();
}

void vsmc_smapler_move(
    vsmc_sampler *sampler_ptr, vsmc_sampler_move_type new_move, int append)
{
    auto cpp_new_move = [new_move](
        std::size_t iter, ::vsmc::ParticleC &cpp_particle) {
        vsmc_particle particle = {
            &cpp_particle, {&cpp_particle.value()}, {&cpp_particle.weight()}};
        return static_cast<std::size_t>(
            new_move(static_cast<int>(iter), &particle));
    };
    ::vsmc::cast(sampler_ptr).move(cpp_new_move, append != 0);
}

void vsmc_sampler_mcmc_queue_clear(vsmc_sampler *sampler_ptr)
{
    ::vsmc::cast(sampler_ptr).mcmc_queue_clear();
}

int vsmc_sampler_mcmc_queue_empty(const vsmc_sampler *sampler_ptr)
{
    return ::vsmc::cast(sampler_ptr).mcmc_queue_empty();
}

void vsmc_smapler_mcmc(
    vsmc_sampler *sampler_ptr, vsmc_sampler_mcmc_type new_mcmc, int append)
{
    auto cpp_new_mcmc = [new_mcmc](
        std::size_t iter, ::vsmc::ParticleC &cpp_particle) {
        vsmc_particle particle = {
            &cpp_particle, {&cpp_particle.value()}, {&cpp_particle.weight()}};
        return static_cast<std::size_t>(
            new_mcmc(static_cast<int>(iter), &particle));
    };
    ::vsmc::cast(sampler_ptr).mcmc(cpp_new_mcmc, append != 0);
}

void vsmc_sampler_initialize(vsmc_sampler *sampler_ptr, void *param)
{
    ::vsmc::cast(sampler_ptr).initialize(param);
}

void vsmc_sampler_iterate(vsmc_sampler *sampler_ptr, int num)
{
    ::vsmc::cast(sampler_ptr).iterate(static_cast<std::size_t>(num));
}

void vsmc_sampler_set_monitor_direct(
    vsmc_sampler *sampler_ptr, const char *name, const vsmc_monitor *mon)
{
    ::vsmc::cast(sampler_ptr).monitor(name, ::vsmc::cast(mon));
}

void vsmc_sampler_set_monitor(vsmc_sampler *sampler_ptr, const char *name,
    int dim, vsmc_monitor_eval_type eval, int record_only,
    vSMCMonitorStage stage)
{
    auto cpp_eval = [eval](std::size_t iter, std::size_t d,
        ::vsmc::ParticleC &cpp_particle, double *res) {
        vsmc_particle particle = {
            &cpp_particle, {&cpp_particle.value()}, {&cpp_particle.weight()}};
        eval(static_cast<int>(iter), static_cast<int>(d), &particle, res);
    };
    ::vsmc::cast(sampler_ptr)
        .monitor(name, static_cast<std::size_t>(dim), cpp_eval,
            record_only != 0, static_cast<vsmc::MonitorStage>(stage));
}

vsmc_monitor vsmc_sampler_get_monitor(
    vsmc_sampler *sampler_ptr, const char *name)
{
    vsmc_monitor monitor = {&::vsmc::cast(sampler_ptr).monitor(name)};

    return monitor;
}

void vsmc_sampler_clear_monitor(vsmc_sampler *sampler_ptr, const char *name)
{
    ::vsmc::cast(sampler_ptr).clear_monitor(name);
}

void vsmc_sampler_clear_monitor_all(vsmc_sampler *sampler_ptr)
{
    ::vsmc::cast(sampler_ptr).clear_monitor();
}

int vsmc_sampler_save(const vsmc_sampler *sampler_ptr, char *mem)
{
    std::stringstream ss;
    ss << ::vsmc::cast(sampler_ptr);
    std::string str(ss.str());
    std::size_t size = (ss.str().size() + 1) * sizeof(char);
    if (mem != nullptr)
        std::memcpy(mem, str.c_str(), size);

    return static_cast<int>(size);
}

void vsmc_sampler_save_f(const vsmc_sampler *sampler_ptr, const char *filename)
{
    std::ofstream os(filename);
    os << ::vsmc::cast(sampler_ptr);
}

} // extern "C"
