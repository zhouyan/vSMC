#include <vsmc/rngc/rngc.h>

typedef struct {
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
} pf_sp;

static inline float log_likelihood(const pf_sp *sp, float obs_x, float obs_y)
{
    float llh_x = 10 * (sp->pos_x - obs_x);
    float llh_y = 10 * (sp->pos_y - obs_y);
    llh_x = log(1 + llh_x * llh_x / 10);
    llh_y = log(1 + llh_y * llh_y / 10);

    return -0.5f * (10 + 1) * (llh_x + llh_y);
}

static inline void rnorm(vsmc_threefry4x32 *rng, float *r)
{
    uint32_t u32[4];
    u32[0] = vsmc_threefry4x32_rand(rng);
    u32[1] = vsmc_threefry4x32_rand(rng);
    u32[2] = vsmc_threefry4x32_rand(rng);
    u32[3] = vsmc_threefry4x32_rand(rng);

    float u01[4];
    u01[0] = vsmc_u01_co_u32f(u32[0]);
    u01[1] = vsmc_u01_co_u32f(u32[1]);
    u01[2] = vsmc_u01_co_u32f(u32[2]);
    u01[3] = vsmc_u01_co_u32f(u32[3]);

    u01[0] = sqrt(-2 * log(u01[0]));
    u01[1] = sqrt(-2 * log(u01[1]));
    u01[2] *= 2;
    u01[3] *= 2;

    r[0] = u01[0] * sinpi(u01[2]);
    r[1] = u01[0] * cospi(u01[2]);
    r[2] = u01[1] * sinpi(u01[3]);
    r[3] = u01[1] * cospi(u01[3]);
}

static inline float init_sp(
    pf_sp *sp, vsmc_threefry4x32 *rng, float obs_x, float obs_y)
{
    vsmc_threefry4x32_init(rng, get_global_id(0));

    float r[4];
    rnorm(rng, r);
    const float sd_pos = 2.0f;
    const float sd_vel = 1.0f;
    sp->pos_x = r[0] * sd_pos;
    sp->pos_y = r[1] * sd_pos;
    sp->vel_x = r[2] * sd_vel;
    sp->vel_y = r[3] * sd_vel;

    return log_likelihood(sp, obs_x, obs_y);
}

static inline float move_sp(
    pf_sp *sp, vsmc_threefry4x32 *rng, float obs_x, float obs_y)
{
    float r[4];
    rnorm(rng, r);
    const float sd_pos = sqrt(0.02f);
    const float sd_vel = sqrt(0.001f);
    sp->pos_x += r[0] * sd_pos + 0.1f * sp->vel_x;
    sp->pos_y += r[1] * sd_pos + 0.1f * sp->vel_y;
    sp->vel_x += r[2] * sd_vel;
    sp->vel_y += r[3] * sd_vel;

    return log_likelihood(sp, obs_x, obs_y);
}

__kernel void copy(int N, __global pf_sp *state, const __global int *index)
{
    int i = get_global_id(0);
    if (i >= N)
        return;

    state[i] = state[index[i]];
}

__kernel void init(int N, __global pf_sp *state,
    __global vsmc_threefry4x32 *rng_set, __global float *weight,
    const __global float *obs_x, const __global float *obs_y)
{
    int i = get_global_id(0);
    if (i >= N)
        return;

    pf_sp sp = state[i];
    vsmc_threefry4x32 rng = rng_set[i];
    weight[i] = init_sp(&sp, &rng, obs_x[0], obs_y[0]);
    state[i] = sp;
    rng_set[i] = rng;
}

__kernel void move(int t, int N, __global pf_sp *state,
    __global vsmc_threefry4x32 *rng_set, __global float *weight,
    const __global float *obs_x, const __global float *obs_y)
{
    int i = get_global_id(0);
    if (i >= N)
        return;

    pf_sp sp = state[i];
    vsmc_threefry4x32 rng = rng_set[i];
    weight[i] = move_sp(&sp, &rng, obs_x[t], obs_y[t]);
    state[i] = sp;
    rng_set[i] = rng;
}
