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
    fp_type pos_x;
    fp_type pos_y;
    fp_type vel_x;
    fp_type vel_y;
} cv;

typedef struct {
    fp_type pos_x;
    fp_type pos_y;
} cv_pos;

VSMC_STATIC_INLINE fp_type log_likelihood(
    const cv *sp, fp_type obs_x, fp_type obs_y)
{
    const fp_type scale = 10;
    const fp_type nu = 10;

    fp_type llh_x = scale * (sp->pos_x - obs_x);
    fp_type llh_y = scale * (sp->pos_y - obs_y);

    llh_x = log1p(llh_x * llh_x / nu);
    llh_y = log1p(llh_y * llh_y / nu);

    return -0.5F * (nu + 1) * (llh_x + llh_y);
}

__kernel void cv_init(__global cv *state, __global ulong *accept,
    __global fp_type *log_weight, __global fp_type *obs_x,
    __global fp_type *obs_y)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    const fp_type sd_pos0 = 2;
    const fp_type sd_vel0 = 1;
    cv sp = state[i];

    vsmc_rng rng;
    vsmc_rng_init(&rng, SEED + i);
    rng.ctr.v[1] = i;
    vsmc_normal01 rnorm;
    vsmc_normal01_init(&rnorm, &rng);

    sp.pos_x = vsmc_normal01_rand(&rnorm, &rng) * sd_pos0;
    sp.pos_y = vsmc_normal01_rand(&rnorm, &rng) * sd_pos0;
    sp.vel_x = vsmc_normal01_rand(&rnorm, &rng) * sd_vel0;
    sp.vel_y = vsmc_normal01_rand(&rnorm, &rng) * sd_vel0;

    state[i] = sp;
    accept[i] = 1;
    log_weight[i] = log_likelihood(&sp, obs_x[0], obs_y[0]);
}

__kernel void cv_move(ulong iter, __global cv *state, __global ulong *accept,
    __global fp_type *inc_weight, __global fp_type *obs_x,
    __global fp_type *obs_y)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    const fp_type var_pos = 0.02F;
    const fp_type var_vel = 0.001F;
    const fp_type sd_pos = sqrt(var_pos);
    const fp_type sd_vel = sqrt(var_vel);
    const fp_type delta = 0.1F;
    cv sp = state[i];

    vsmc_rng rng;
    vsmc_rng_init(&rng, SEED + i);
    rng.ctr.v[1] = i;
    vsmc_normal01 rnorm;
    vsmc_normal01_init(&rnorm, &rng);

    sp.pos_x += vsmc_normal01_rand(&rnorm, &rng) * sd_pos + delta * sp.vel_x;
    sp.pos_y += vsmc_normal01_rand(&rnorm, &rng) * sd_pos + delta * sp.vel_y;
    sp.vel_x += vsmc_normal01_rand(&rnorm, &rng) * sd_vel;
    sp.vel_y += vsmc_normal01_rand(&rnorm, &rng) * sd_vel;

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
