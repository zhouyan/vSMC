//============================================================================
// vSMC/example/gmm/gmm_smc_cl.cl
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#include <vsmc/rng/normal01.h>

typedef struct {
    fp_type mu[CompNum];
    fp_type lambda[CompNum];
    fp_type weight[CompNum];
    fp_type log_prior;
    fp_type log_likelihood;
} gmm_param;

VSMC_STATIC_INLINE fp_type log_prior (gmm_param *pparam,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0)
{
    const fp_type hlf = -0.5;

    fp_type lp = 0;
    for (uint d = 0; d != CompNum; ++d) {
        fp_type resid = pparam->mu[d] - mu0;
        lp += hlf * (resid * resid) / (sd0 * sd0);
        lp += (shape0 - 1) * log(pparam->lambda[d]) -
            pparam->lambda[d] / scale0;
    }

    return pparam->log_prior = lp;
}

VSMC_STATIC_INLINE fp_type log_likelihood (gmm_param *pparam,
        __global const fp_type *obs)
{
    fp_type log_lambda[CompNum];
    for (uint d = 0; d != CompNum; ++d)
        log_lambda[d] = log(pparam->lambda[d]);

    const fp_type hlf = 0.5;
    const fp_type drift = 1e-13;

    fp_type ll = 0;
    for (uint i = 0; i != DataNum; ++i) {
        const fp_type y = obs[i];
        fp_type lli = 0;
        for (uint d = 0; d != CompNum; ++d) {
            fp_type resid = y - pparam->mu[d];
            lli += pparam->weight[d] * exp(hlf * log_lambda[d]
                    - hlf * pparam->lambda[d] * resid * resid);
        }
        ll += log(lli + drift);
    }

    return pparam->log_likelihood = ll;
}

VSMC_STATIC_INLINE fp_type log_target (gmm_param *pparam,
        __global const fp_type *obs, fp_type alpha,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0)
{
    return log_prior(pparam, mu0, sd0, shape0, scale0) +
        alpha * log_likelihood(pparam, obs);
}

VSMC_STATIC_INLINE fp_type lp_weight (const fp_type *weight)
{
    fp_type sum_lw = 1;
    fp_type sum_llw = 0;
    for (unsigned d = 0; d != CompNum - 1; ++d) {
        fp_type w = weight[d] / weight[CompNum - 1];
        sum_lw += w;
        sum_llw += log(w);
    }
    sum_llw -= CompNum * log(sum_lw);

    return sum_llw;
}

__kernel
void gmm_init (__global gmm_param *state, __global ulong *accept,
        __global const fp_type *lambda_host,
        __global const fp_type *weight_host,
        __global const fp_type *obs,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array2x32 *counter)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param  = state[id];

    cburng2x32_rng_t rng;
    cburng2x32_init(&rng, Seed + id);
    NORMAL01_2x32 rnorm;
    NORMAL01_2x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d) {
        param.mu[d] =  mu0 + NORMAL01_2x32_RAND(&rnorm, &rng) * sd0;
        param.lambda[d] = lambda_host[d * Size + id];
        param.weight[d] = weight_host[d * Size + id];
    }
    log_target(&param, obs, 0, mu0, sd0, shape0, scale0);
    state[id] = param;
    accept[id] = 0;
    counter[id] = rng.ctr;
}

__kernel
void gmm_move_smc (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global fp_type *exp_weight, fp_type alpha_inc)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    exp_weight[id] = exp(alpha_inc * state[id].log_likelihood);
    accept[id] = 0;
}

__kernel
void gmm_move_mu (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *obs,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array2x32 *counter)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type p = -(param.log_prior + alpha * param.log_likelihood);

    cburng2x32_rng_t rng;
    cburng2x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_2x32 rnorm;
    NORMAL01_2x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d)
        param.mu[d] += NORMAL01_2x32_RAND(&rnorm, &rng) * sd;
    p += log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    fp_type u = log(U01_OPEN_CLOSED_32(cburng2x32_rand(&rng)));
    ulong acc = 0;
    if (p > u) {
        acc = 1;
        state[id] = param;
    }

    accept[id] = acc;
    counter[id] = rng.ctr;
}

__kernel
void gmm_move_lambda (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *obs,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array2x32 *counter)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type p = -(param.log_prior + alpha * param.log_likelihood);
    for (uint d = 0; d != CompNum; ++d)
        p -= log(param.lambda[d]);

    cburng2x32_rng_t rng;
    cburng2x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_2x32 rnorm;
    NORMAL01_2x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d)
        param.lambda[d] *= exp(NORMAL01_2x32_RAND(&rnorm, &rng) * sd);
    p += log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    for (uint d = 0; d != CompNum; ++d)
        p += log(param.lambda[d]);
    fp_type u = log(U01_OPEN_CLOSED_32(cburng2x32_rand(&rng)));
    ulong acc = 0;
    if (p > u) {
        acc = 1;
        state[id] = param;
    }

    accept[id] = acc;
    counter[id] = rng.ctr;
}

__kernel
void gmm_move_weight (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *obs,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array2x32 *counter)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type p = -(param.log_prior + alpha * param.log_likelihood);
    p -= lp_weight(param.weight);

    cburng2x32_rng_t rng;
    cburng2x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_2x32 rnorm;
    NORMAL01_2x32_INIT(&rnorm, &rng);

    fp_type sum_weight = 1;
    for (uint d = 0; d != CompNum - 1; ++d) {
        param.weight[d] = log(param.weight[d] / param.weight[CompNum - 1]);
        param.weight[d] += NORMAL01_2x32_RAND(&rnorm, &rng) * sd;
        param.weight[d] = exp(param.weight[d]);
        sum_weight += param.weight[d];
    }
    param.weight[CompNum - 1] = 1;
    for (unsigned d = 0; d != CompNum; ++d)
        param.weight[d] /= sum_weight;

    p += log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    p += lp_weight(param.weight);
    fp_type u = log(U01_OPEN_CLOSED_32(cburng2x32_rand(&rng)));
    ulong acc = 0;
    if (p > u) {
        acc = 1;
        state[id] = param;
    }

    accept[id] = acc;
    counter[id] = rng.ctr;
}

__kernel
void gmm_path_state (ulong iter, __global gmm_param *state,
        __global fp_type *res)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    res[id] = state[id].log_likelihood;
}
