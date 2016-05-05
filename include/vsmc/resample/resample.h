//============================================================================
// vSMC/include/vsmc/resample/resample.h
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

#ifndef VSMC_RESAMPLE_RESAMPLE_H
#define VSMC_RESAMPLE_RESAMPLE_H

#include <vsmc/internal/common.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_Resample
/// @{

/// \brief `vsmc::resample_trans_residual`
int vsmc_resample_trans_residual(
    int n, int m, const double *weight, double *resid, int *integ);

/// \brief `vsmc::resample_trans_u01_rep`
void vsmc_resample_trans_u01_rep(
    int n, int m, const double *weight, const double *u01, int *replication);

/// \brief `vsmc::resample_trans_rep_index`
void vsmc_resample_trans_rep_index(
    int n, int m, const int *replication, int *index);

/// \brief `vsmc::ResampleMultinomial`
void vsmc_resample_multinomial(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// \brief `vsmc::ResampleStratified`
void vsmc_resample_stratified(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// \brief `vsmc::ResampleSystematic`
void vsmc_resample_systematic(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// \brief `vsmc::ResampleResidual`
void vsmc_resample_residual(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// \brief `vsmc::ResampleResidualStratified`
void vsmc_resample_residual_stratified(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// \brief `vsmc::ResampleSystematic`
void vsmc_resample_residual_systematic(
    int n, int m, vsmc_rng rng, const double *weight, int *replication);

/// @} C_API_Resample

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_RESAMPLE_RESAMPLE_H
