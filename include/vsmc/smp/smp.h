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

/// \addtogroup C_API_SMP
/// @{

/// \brief The maximum of an integer that can be passed to where
/// `vSMCBackendSMP` value is expected
int vsmc_backend_smp_max();

/// \brief Check if a given SMP backend is defined within the library
int vsmc_backend_smp_check(vSMCBackendSMP backend);

/// \brief `vsmc::Sampler::eval` with `vsmc::SamplerEvalSMP` as input
void vsmc_sampler_eval_smp(vSMCBackendSMP backend, vsmc_sampler sampler,
    vsmc_sampler_eval_smp_type new_eval, vSMCSamplerStage, int append);

/// \brief `vsmc::Monitor` with `vsmc::MonitorEvalSMP` as input
vsmc_monitor vsmc_monitor_new_smp(vSMCBackendSMP backend, size_t dim,
    vsmc_monitor_eval_smp_type eval, int record_only, vSMCMonitorStage stage);

/// @} C_API_SMP_SEQ

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_SMP_SMP_H
