//============================================================================
// vSMC/example/pf/pf_cl.cl
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
    fp_type x_pos;
    fp_type y_pos;
    fp_type x_vel;
    fp_type y_vel;
} cv;

typedef struct {
    fp_type x_pos;
    fp_type y_pos;
} cv_pos;

fp_type log_likelihood (const cv *sp, fp_type x_obs, fp_type y_obs)
{
    const fp_type scale = 10;
    const fp_type nu = 10;

    fp_type llh_x = scale * (sp->x_pos - x_obs);
    fp_type llh_y = scale * (sp->y_pos - y_obs);

    llh_x = log1p(llh_x * llh_x / nu);
    llh_y = log1p(llh_y * llh_y / nu);

    return -0.5F * (nu + 1) * (llh_x + llh_y);
}

__kernel
void cv_init (__global cv *state,
        __global ulong *accept, __global fp_type *log_weight,
        __global fp_type *x_obs, __global fp_type *y_obs,
        __global struct r123array4x32 *counter)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    const fp_type sd_pos0 = 2;
    const fp_type sd_vel0 = 1;
    cv sp = state[i];

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, SEED + i);
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    sp.x_pos = NORMAL01_4x32_RAND(&rnorm, &rng) * sd_pos0;
    sp.y_pos = NORMAL01_4x32_RAND(&rnorm, &rng) * sd_pos0;
    sp.x_vel = NORMAL01_4x32_RAND(&rnorm, &rng) * sd_vel0;
    sp.y_vel = NORMAL01_4x32_RAND(&rnorm, &rng) * sd_vel0;

    state[i] = sp;
    accept[i] = 1;
    log_weight[i] = log_likelihood(&sp, x_obs[0], y_obs[0]);
    counter[i] = rng.ctr;
}

__kernel
void cv_move (ulong iter, __global cv *state,
        __global ulong *accept, __global fp_type *inc_weight,
        __global fp_type *x_obs, __global fp_type *y_obs,
        __global struct r123array4x32 *counter)
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

    cburng4x32_rng_t rng;
    cburng4x32_init(&rng, SEED + i);
    rng.ctr = counter[i];
    NORMAL01_4x32 rnorm;
    NORMAL01_4x32_INIT(&rnorm, &rng);

    sp.x_pos += NORMAL01_4x32_RAND(&rnorm, &rng) * sd_pos + delta * sp.x_vel;
    sp.y_pos += NORMAL01_4x32_RAND(&rnorm, &rng) * sd_pos + delta * sp.y_vel;
    sp.x_vel += NORMAL01_4x32_RAND(&rnorm, &rng) * sd_vel;
    sp.y_vel += NORMAL01_4x32_RAND(&rnorm, &rng) * sd_vel;

    state[i] = sp;
    accept[i] = 1;
    inc_weight[i] = log_likelihood(&sp, x_obs[iter], y_obs[iter]);
    counter[i] = rng.ctr;
}

__kernel
void cv_est (ulong iter, ulong dim, __global cv *state, __global cv_pos *est)
{
    ulong i = get_global_id(0);
    if (i >= SIZE)
        return;

    est[i].x_pos = state[i].x_pos;
    est[i].y_pos = state[i].y_pos;
}
