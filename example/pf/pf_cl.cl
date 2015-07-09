//============================================================================
// vSMC/example/pf/pf_cl.cl
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

#include <vsmc/rngc/rngc.h>

typedef struct {
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
} cv;

typedef struct {
    float pos_x;
    float pos_y;
} cv_pos;

VSMC_STATIC_INLINE float4 normal01(vsmc_philox4x32 *rng)
{
    vsmc_philox4x32_gen(&rng->ctr, &rng->key, &rng->state);
    float u1 = sqrt(-2 * vsmc_u01_open_closed_u32_f32(rng->state.v[0]));
    float u2 = sqrt(-2 * vsmc_u01_open_closed_u32_f32(rng->state.v[1]));
    float v1 = 6.283185 *vsmc_u01_open_closed_u32_f32(rng->state.v[2]);
    float v2 = 6.283185 *vsmc_u01_open_closed_u32_f32(rng->state.v[3]);
    float4 r;
    r.x = u1 * cos(v1);
    r.y = u1 * sin(v1);
    r.z = u2 * cos(v2);
    r.w = u2 * sin(v2);

    return r;
}

VSMC_STATIC_INLINE float log_likelihood(
        const cv *sp, float obs_x, float obs_y)
{
    const float scale = 10;
    const float nu = 10;

    float llh_x = scale * (sp->pos_x - obs_x);
    float llh_y = scale * (sp->pos_y - obs_y);

    llh_x = log1p(llh_x * llh_x / nu);
    llh_y = log1p(llh_y * llh_y / nu);

    return -0.5F * (nu + 1) * (llh_x + llh_y);
}

__kernel void cv_init(__global cv *state, __global ulong *accept,
        __global float *log_weight, __global float *obs_x,
        __global float *obs_y)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    const float sd_pos0 = 2;
    const float sd_vel0 = 1;
    cv sp = state[i];

    vsmc_philox4x32 rng;
    vsmc_philox4x32_init(&rng, SEED + i);
    rng.ctr.v[1] = i;

    float4 r = normal01(&rng);
    sp.pos_x = r.x * sd_pos0;
    sp.pos_y = r.y * sd_pos0;
    sp.vel_x = r.z * sd_vel0;
    sp.vel_y = r.w * sd_vel0;

    state[i] = sp;
    accept[i] = 1;
    log_weight[i] = log_likelihood(&sp, obs_x[0], obs_y[0]);
}

__kernel void cv_move(ulong iter, __global cv *state, __global ulong *accept,
        __global float *inc_weight, __global float *obs_x,
        __global float *obs_y)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    const float var_pos = 0.02F;
    const float var_vel = 0.001F;
    const float sd_pos = sqrt(var_pos);
    const float sd_vel = sqrt(var_vel);
    const float delta = 0.1F;
    cv sp = state[i];

    vsmc_philox4x32 rng;
    vsmc_philox4x32_init(&rng, SEED + i);
    rng.ctr.v[1] = i;

    float4 r = normal01(&rng);
    sp.pos_x += r.x * sd_pos + delta * sp.vel_x;
    sp.pos_y += r.y * sd_pos + delta * sp.vel_y;
    sp.vel_x += r.z * sd_vel;
    sp.vel_y += r.w * sd_vel;

    state[i] = sp;
    accept[i] = 1;
    inc_weight[i] = log_likelihood(&sp, obs_x[iter], obs_y[iter]);
}

__kernel void cv_est(
        ulong iter, ulong dim, __global cv *state, __global cv_pos *est)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    est[i].pos_x = state[i].pos_x;
    est[i].pos_y = state[i].pos_y;
}
