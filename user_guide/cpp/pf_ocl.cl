#include <vsmc/rngc/rngc.h>

typedef struct {
    float w;
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
    float u[4];
    u[0] = sqrt(-2 * log(vsmc_u01_co_u32f(vsmc_threefry4x32_rand(rng))));
    u[1] = sqrt(-2 * log(vsmc_u01_co_u32f(vsmc_threefry4x32_rand(rng))));
    u[2] = 6.28318531f * vsmc_u01_co_u32f(vsmc_threefry4x32_rand(rng));
    u[3] = 6.28318531f * vsmc_u01_co_u32f(vsmc_threefry4x32_rand(rng));
    r[0] = u[0] * sin(u[2]);
    r[1] = u[0] * cos(u[2]);
    r[2] = u[1] * sin(u[3]);
    r[3] = u[1] * cos(u[3]);
}

static inline void init_sp(vsmc_threefry4x32 *rng, pf_sp *sp)
{
    float r[4];
    rnorm(rng, r);
    const float sd_pos = 2;
    const float sd_vel = 1;
    sp->pos_x = r[0] * sd_pos;
    sp->pos_y = r[1] * sd_pos;
    sp->vel_x = r[2] * sd_vel;
    sp->vel_y = r[3] * sd_vel;
}

static inline void move_sp(vsmc_threefry4x32 *rng, pf_sp *sp)
{
    float r[4];
    rnorm(rng, r);
    const float sd_pos = sqrt(0.02);
    const float sd_vel = sqrt(0.001);
    sp->pos_x += r[0] * sd_pos + 0.1 * sp->vel_x;
    sp->pos_y += r[1] * sd_pos + 0.1 * sp->vel_y;
    sp->vel_x += r[2] * sd_vel;
    sp->vel_y += r[3] * sd_vel;
}

kernel void init(uint N, global vsmc_threefry4x32 *rng_set,
    global pf_sp *state, const global float *obs_x, const global float *obs_y)
{
    uint32_t i = get_global_id(0);
    if (i >= N)
        return;

    vsmc_threefry4x32 rng = rng_set[i];
    vsmc_threefry4x32_init(&rng, i);
    pf_sp sp = state[i];
    init_sp(&rng, &sp);
    sp.w = log_likelihood(&sp, obs_x[0], obs_y[0]);
    rng_set[i] = rng;
    state[i] = sp;
}

kernel void move(uint t, uint N, global vsmc_threefry4x32 *rng_set,
    global pf_sp *state, const global float *obs_x, const global float *obs_y)
{
    uint32_t i = get_global_id(0);
    if (i >= N)
        return;

    vsmc_threefry4x32 rng = rng_set[i];
    pf_sp sp = state[i];
    move_sp(&rng, &sp);
    sp.w = log_likelihood(&sp, obs_x[t], obs_y[t]);
    rng_set[i] = rng;
    state[i] = sp;
}
