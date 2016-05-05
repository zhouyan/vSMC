//============================================================================
// vSMC/include/vsmc/smp/smp.h
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

#ifndef VSMC_SMP_SMP_H
#define VSMC_SMP_SMP_H

#include <vsmc/internal/common.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_SMP_SEQ
/// @{

/// \brief `vsmc::Sampler::init` with `vsmc::InitializeSEQ` as input
void vsmc_sampler_init_seq(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/// \brief `vsmc::Sampler::move` with `vsmc::MoveSEQ` as input
void vsmc_sampler_move_seq(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::mcmc` with `vsmc::MoveSEQ` as input
void vsmc_sampler_mcmc_seq(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalSEQ` as input
void vsmc_sampler_set_monitor_seq(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/// @} C_API_SMP_SEQ

/// \addtogroup C_API_SMP_STD
/// @{

/// \brief `vsmc::Sampler::init` with `vsmc::InitializeSTD` as input
void vsmc_sampler_init_std(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/// \brief `vsmc::Sampler::move` with `vsmc::MoveSTD` as input
void vsmc_sampler_move_std(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::mcmc` with `vsmc::MoveSTD` as input
void vsmc_sampler_mcmc_std(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalSTD` as input
void vsmc_sampler_set_monitor_std(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/// @} C_API_SMP_STD

/// \addtogroup C_API_SMP_OMP
/// @{

/// \brief `vsmc::Sampler::init` with `vsmc::InitializeOMP` as input
void vsmc_sampler_init_omp(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/// \brief `vsmc::Sampler::move` with `vsmc::MoveOMP` as input
void vsmc_sampler_move_omp(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::mcmc` with `vsmc::MoveOMP` as input
void vsmc_sampler_mcmc_omp(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalOMP` as input
void vsmc_sampler_set_monitor_omp(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/// @} C_API_SMP_OMP

/// \addtogroup C_API_SMP_TBB
/// @{

/// \brief `vsmc::Sampler::init` with `vsmc::InitializeTBB` as input
void vsmc_sampler_init_tbb(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/// \brief `vsmc::Sampler::move` with `vsmc::MoveTBB` as input
void vsmc_sampler_move_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::mcmc` with `vsmc::MoveTBB` as input
void vsmc_sampler_mcmc_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/// \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalTBB` as input
void vsmc_sampler_set_monitor_tbb(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/// @} C_API_SMP_TBB

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_SMP_SMP_H
