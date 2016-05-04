//============================================================================
// vSMC/lib/src/smp/backend_tbb.cpp
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
// CONTBBUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include "libvsmc.hpp"

extern "C" {

void vsmc_sampler_init_tbb(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post)
{
    ::vsmc::cast(sampler).init(::vsmc::cast<::vsmc::InitializeTBB>(
        eval_sp, eval_param, eval_pre, eval_post));
}

void vsmc_sampler_move_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append)
{
    ::vsmc::cast(sampler).move(
        ::vsmc::cast<::vsmc::MoveTBB>(eval_sp, eval_pre, eval_post),
        append != 0);
}

void vsmc_sampler_mcmc_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append)
{
    ::vsmc::cast(sampler).mcmc(
        ::vsmc::cast<::vsmc::MoveTBB>(eval_sp, eval_pre, eval_post),
        append != 0);
}

void vsmc_sampler_set_monitor_tbb(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage)
{
    ::vsmc::cast(sampler).monitor(name, static_cast<std::size_t>(dim),
        ::vsmc::cast<::vsmc::MonitorEvalTBB>(eval_sp, eval_pre, eval_post),
        record_only != 0, static_cast<::vsmc::MonitorStage>(stage));
}

} // extern "C"
