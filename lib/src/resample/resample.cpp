//============================================================================
// vSMC/lib/src/resample/resample.cpp
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
// SUBSTITUTE GOODS OR SERVICES{} LOSS OF USE, DATA, OR PROFITS{} OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include "libvsmc.hpp"

#define VSMC_DEFINE_C_API_RESAMPLE(Name, name)                                \
    void vsmc_resample_##name(int m, int n, vsmc_rng *rng_ptr,                \
        const double *weight, int *replication)                               \
    {                                                                         \
        ::vsmc::Resample##Name resample;                                      \
        resample(static_cast<std::size_t>(m), static_cast<std::size_t>(n),    \
            ::vsmc::cast(rng_ptr), weight, replication);                      \
    }

extern "C" {

void vsmc_resample_trans_u01_rep(
    int m, int n, const double *weight, const double *u01, int *replication)
{
    ::vsmc::resample_trans_u01_rep(static_cast<std::size_t>(m),
        static_cast<std::size_t>(n), weight, u01, replication);
}

void vsmc_resample_trans_u01_index(
    int m, int n, const double *weight, const double *u01, int *index)
{
    ::vsmc::resample_trans_u01_index(static_cast<std::size_t>(m),
        static_cast<std::size_t>(n), weight, u01, index);
}

void vsmc_resample_trans_rep_index(
    int m, int n, const int *replication, int *index)
{
    ::vsmc::resample_trans_rep_index(static_cast<std::size_t>(m),
        static_cast<std::size_t>(n), replication, index);
}

void vsmc_resample_trans_index_rep(
    int m, int n, const int *index, int *replication)
{
    ::vsmc::resample_trans_index_rep(
        static_cast<std::size_t>(m), std::size_t(n), index, replication);
}

int vsmc_resample_trans_residual(
    int m, int n, const double *weight, double *resid, int *integ)
{
    return static_cast<int>(
        ::vsmc::resample_trans_residual(static_cast<std::size_t>(m),
            static_cast<std::size_t>(n), weight, resid, integ));
}

VSMC_DEFINE_C_API_RESAMPLE(Multinomial, multinomial)
VSMC_DEFINE_C_API_RESAMPLE(Stratified, stratified)
VSMC_DEFINE_C_API_RESAMPLE(Systematic, systematic)
VSMC_DEFINE_C_API_RESAMPLE(Residual, residual)
VSMC_DEFINE_C_API_RESAMPLE(ResidualStratified, residual_stratified)
VSMC_DEFINE_C_API_RESAMPLE(ResidualSystematic, residual_systematic)

} // extern "C"

