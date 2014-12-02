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
    fp_type log_target;
} gmm_param;

VSMC_STATIC_INLINE fp_type log_prior (gmm_param *pparam,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0)
{
    fp_type lp = 0;
    for (uint d = 0; d != CompNum; ++d) {
        fp_type resid = pparam->mu[d] - mu0;
        lp += -0.5F * (resid * resid) / (sd0 * sd0);
        lp += (shape0 - 1) * log(pparam->lambda[d]) -
            pparam->lambda[d] / scale0;
    }

    return pparam->log_prior = lp;
}

VSMC_STATIC_INLINE fp_type log_likelihood (gmm_param *pparam,
        __local const fp_type *obs)
{
    fp_type log_lambda[CompNum];
    for (uint d = 0; d != CompNum; ++d)
        log_lambda[d] = log(pparam->lambda[d]);

    const fp_type hlf = 0.5;
    const fp_type drift = 1e-13;

    fp_type ll = 0;
    for (uint i = 0; i != DataNum; ++i) {
        fp_type lli = 0;
        for (uint d = 0; d != CompNum; ++d) {
            fp_type resid = obs[i] - pparam->mu[d];
            lli += pparam->weight[d] * exp(hlf * log_lambda[d]
                    - hlf * pparam->lambda[d] * resid * resid);
        }
        ll += log(lli + drift);
    }

    return pparam->log_likelihood = ll;
}

VSMC_STATIC_INLINE fp_type log_target (gmm_param *pparam,
        __local const fp_type *obs, fp_type alpha,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0)
{
    fp_type lt = log_prior(pparam, mu0, sd0, shape0, scale0) +
        alpha * log_likelihood(pparam, obs);

    return pparam->log_target = lt;
}

__kernel
void query_size (__global uint *sizes)
{
    if (!get_global_id(0)) {
        gmm_param gmm_param_array[2];
        sizes[0] = sizeof(gmm_param);
        sizes[1] = 2;
        sizes[2] = sizeof(gmm_param_array);
    }
}

__kernel
void gmm_init (__global gmm_param *state, __global ulong *accept,
        __global const fp_type *lambda_host,
        __global const fp_type *weight_host,
        __global const fp_type *dat,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array4x32 *counter)
{
    __local fp_type obs[DataNum];
    barrier(CLK_LOCAL_MEM_FENCE);
    async_work_group_copy(obs, dat, DataNum, 0);
    barrier(CLK_LOCAL_MEM_FENCE);

    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param  = state[id];

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, Seed + id);
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d) {
        param.mu[d] =  mu0 + NORMAL01_4x32_RAND(&rnorm, &rng) * sd0;
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
        __global fp_type *inc_weight, fp_type alpha_inc)
{
    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    inc_weight[id] = alpha_inc * state[id].log_likelihood;
    accept[id] = 0;
}

__kernel
void gmm_move_mu (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *dat,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array4x32 *counter)
{
    __local fp_type obs[DataNum];
    barrier(CLK_LOCAL_MEM_FENCE);
    async_work_group_copy(obs, dat, DataNum, 0);
    barrier(CLK_LOCAL_MEM_FENCE);

    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type mu_old[CompNum];
    for (uint d = 0; d != CompNum; ++d)
        mu_old[d] = param.mu[d];
    const fp_type log_prior_old = param.log_prior;
    const fp_type log_likelihood_old = param.log_likelihood;
    const fp_type log_target_old =
        param.log_prior + alpha * param.log_likelihood;

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d)
        param.mu[d] += NORMAL01_4x32_RAND(&rnorm, &rng) * sd;
    log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    fp_type p = param.log_target - log_target_old;
    fp_type u = log(U01_OPEN_CLOSED_32(cburng4x32_rand(&rng)));
    ulong acc = 1;
    if (p < u) {
        for (uint d = 0; d != CompNum; ++d)
            param.mu[d] = mu_old[d];
        param.log_prior      = log_prior_old;
        param.log_likelihood = log_likelihood_old;
        param.log_target     = log_target_old;
        acc = 0;
    }

    state[id] = param;
    accept[id] = acc;
    counter[id] = rng.ctr;
}

__kernel
void gmm_move_lambda (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *dat,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array4x32 *counter)
{
    __local fp_type obs[DataNum];
    barrier(CLK_LOCAL_MEM_FENCE);
    async_work_group_copy(obs, dat, DataNum, 0);
    barrier(CLK_LOCAL_MEM_FENCE);

    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type lambda_old[CompNum];
    for (uint d = 0; d != CompNum; ++d)
        lambda_old[d] = param.lambda[d];
    const fp_type log_prior_old = param.log_prior;
    const fp_type log_likelihood_old = param.log_likelihood;
    const fp_type log_target_old =
        param.log_prior + alpha * param.log_likelihood;

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    for (uint d = 0; d != CompNum; ++d)
        param.lambda[d] *= exp(NORMAL01_4x32_RAND(&rnorm, &rng) * sd);
    log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    fp_type p = param.log_target - log_target_old;
    for (uint d = 0; d != CompNum; ++d)
        p += log(param.lambda[d]) - log(lambda_old[d]);
    fp_type u = log(U01_OPEN_CLOSED_32(cburng4x32_rand(&rng)));
    ulong acc = 1;
    if (p < u) {
        for (uint d = 0; d != CompNum; ++d)
            param.lambda[d] = lambda_old[d];
        param.log_prior      = log_prior_old;
        param.log_likelihood = log_likelihood_old;
        param.log_target     = log_target_old;
        acc = 0;
    }

    state[id] = param;
    accept[id] = acc;
    counter[id] = rng.ctr;
}

__kernel
void gmm_move_weight (ulong iter,
        __global gmm_param *state, __global ulong *accept,
        __global const fp_type *dat,
        fp_type alpha, fp_type sd,
        fp_type mu0, fp_type sd0, fp_type shape0, fp_type scale0,
        __global struct r123array4x32 *counter)
{
    __local fp_type obs[DataNum];
    barrier(CLK_LOCAL_MEM_FENCE);
    async_work_group_copy(obs, dat, DataNum, 0);
    barrier(CLK_LOCAL_MEM_FENCE);

    size_type id = get_global_id(0);
    if (id >= Size)
        return;

    gmm_param param = state[id];
    fp_type weight_old[CompNum];
    for (uint d = 0; d != CompNum; ++d)
        weight_old[d] = param.weight[d];
    const fp_type log_prior_old = param.log_prior;
    const fp_type log_likelihood_old = param.log_likelihood;
    const fp_type log_target_old =
        param.log_prior + alpha * param.log_likelihood;

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, Seed + id);
    rng.ctr = counter[id];
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    fp_type sum_weight = 1;
    for (uint d = 0; d != CompNum - 1; ++d) {
        param.weight[d] = log(param.weight[d] / param.weight[CompNum - 1]);
        param.weight[d] += NORMAL01_4x32_RAND(&rnorm, &rng) * sd;
        param.weight[d] = exp(param.weight[d]);
        sum_weight += param.weight[d];
    }
    param.weight[CompNum - 1] = 1;
    for (unsigned d = 0; d != CompNum; ++d)
        param.weight[d] /= sum_weight;

    fp_type sum_lw, sum_llw;

    sum_lw = 1;
    sum_llw = 0;
    for (unsigned d = 0; d != CompNum - 1; ++d) {
        fp_type w = param.weight[d] / param.weight[CompNum - 1];
        sum_lw += w;
        sum_llw += log(w);
    }
    sum_llw -= CompNum * log(sum_lw);
    fp_type lp = sum_llw;

    sum_lw = 1;
    sum_llw = 0;
    for (unsigned d = 0; d != CompNum - 1; ++d) {
        fp_type w = weight_old[d] / weight_old[CompNum - 1];
        sum_lw += w;
        sum_llw += log(w);
    }
    sum_llw -= CompNum * log(sum_lw);
    fp_type lp_old = sum_llw;
    log_target(&param, obs, alpha, mu0, sd0, shape0, scale0);
    fp_type p = param.log_target - log_target_old + lp - lp_old;
    fp_type u = log(U01_OPEN_CLOSED_32(cburng4x32_rand(&rng)));
    ulong acc = 1;
    if (p < u) {
        for (uint d = 0; d != CompNum; ++d)
            param.weight[d] = weight_old[d];
        param.log_prior      = log_prior_old;
        param.log_likelihood = log_likelihood_old;
        param.log_target     = log_target_old;
        acc = 0;
    }

    state[id] = param;
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
