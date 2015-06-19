//============================================================================
// vSMC/include/vsmc/rngc/random.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_RNGC_RANDOM_H
#define VSMC_RNGC_RANDOM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t state[128];
} vsmc_rng;

void vsmc_rng_init(vsmc_rng *, unsigned seed);
void vsmc_rng_seed(vsmc_rng *, unsigned seed);
void vsmc_rng_rand(vsmc_rng *, int, unsigned *);

void vsmc_rng_uniform_int(vsmc_rng *, int, int *, int, int);
void vsmc_rng_uniform_real(vsmc_rng *, int, double *, double, double);
void vsmc_rng_bernoulli(vsmc_rng *, int, int *, double);
void vsmc_rng_binomial(vsmc_rng *, int, int *, int, double);
void vsmc_rng_negative_binomial(vsmc_rng *, int, int *, int, double);
void vsmc_rng_geometric(vsmc_rng *, int, int *, double);
void vsmc_rng_poisson(vsmc_rng *, int, int *, double);
void vsmc_rng_exponential(vsmc_rng *, int, double *, double);
void vsmc_rng_gamma(vsmc_rng *, int, double *, double, double);
void vsmc_rng_weibull(vsmc_rng *, int, double *, double, double);
void vsmc_rng_extreme_value(vsmc_rng *, int, double *, double, double);
void vsmc_rng_normal(vsmc_rng *, int, double *, double, double);
void vsmc_rng_lognormal(vsmc_rng *, int, double *, double, double);
void vsmc_rng_chi_squared(vsmc_rng *, int, double *, double);
void vsmc_rng_cauchy(vsmc_rng *, int, double *, double, double);
void vsmc_rng_fisher_f(vsmc_rng *, int, double *, double, double);
void vsmc_rng_student_t(vsmc_rng *, int, double *, double);
void vsmc_rng_stable(
    vsmc_rng *, int, double *, double, double, double, double);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_RNGC_RANDOM_H
