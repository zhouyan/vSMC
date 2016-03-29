#include <vsmc/vsmc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static const int PosX = 0;
static const int PosY = 1;
static const int VelX = 2;
static const int VelY = 3;

// Storage for data
static vsmc_vector pf_obs_x = {NULL, 0};
static vsmc_vector pf_obs_y = {NULL, 0};

// Temporaries used by pf_init and pf_move
static vsmc_vector pf_pos_x = {NULL, 0};
static vsmc_vector pf_pos_y = {NULL, 0};
static vsmc_vector pf_vel_x = {NULL, 0};
static vsmc_vector pf_vel_y = {NULL, 0};
static vsmc_vector pf_weight = {NULL, 0};

static inline double pf_log_likelihood(int t, const vsmc_single_particle *sp)
{
    double llh_x = 10 * (sp->state[PosX] - pf_obs_x.data[t]);
    double llh_y = 10 * (sp->state[PosY] - pf_obs_y.data[t]);
    llh_x = log(1 + llh_x * llh_x / 10);
    llh_y = log(1 + llh_y * llh_y / 10);

    return -0.5 * (10 + 1) * (llh_x + llh_y);
}

static inline void pf_read_data(const char *param)
{
    if (!param)
        return;

    FILE *data = fopen(param, "r");
    int n = 0;
    while (1) {
        double x;
        double y;
        int nx = fscanf(data, "%lg", &x);
        int ny = fscanf(data, "%lg", &y);
        if (nx == 1 && ny == 1)
            ++n;
        else
            break;
    }
    vsmc_vector_resize(&pf_obs_x, n);
    vsmc_vector_resize(&pf_obs_y, n);
    fseek(data, 0, SEEK_SET);
    for (int i = 0; i < n; ++i) {
        fscanf(data, "%lg", &pf_obs_x.data[i]);
        fscanf(data, "%lg", &pf_obs_y.data[i]);
    }
    fclose(data);
}

static inline void pf_normal(
    vsmc_particle particle, double sd_pos, double sd_vel)
{
    vsmc_rng rng = vsmc_particle_rng(particle, 0);
    const int size = vsmc_particle_size(particle);
    vsmc_vector_resize(&pf_pos_x, size);
    vsmc_vector_resize(&pf_pos_y, size);
    vsmc_vector_resize(&pf_vel_x, size);
    vsmc_vector_resize(&pf_vel_y, size);
    vsmc_vector_resize(&pf_weight, size);
    vsmc_normal_rand(rng, size, pf_pos_x.data, 0, sd_pos);
    vsmc_normal_rand(rng, size, pf_pos_y.data, 0, sd_pos);
    vsmc_normal_rand(rng, size, pf_vel_x.data, 0, sd_vel);
    vsmc_normal_rand(rng, size, pf_vel_y.data, 0, sd_vel);
}

static inline void pf_init_param(vsmc_particle particle, void *param)
{
    pf_read_data((const char *) param);
}

static inline void pf_init_pre(vsmc_particle particle)
{
    pf_normal(particle, 2, 1);
}

static inline int pf_init_sp(vsmc_single_particle sp)
{
    sp.state[PosX] = pf_pos_x.data[sp.id];
    sp.state[PosY] = pf_pos_y.data[sp.id];
    sp.state[VelX] = pf_vel_x.data[sp.id];
    sp.state[VelY] = pf_vel_y.data[sp.id];
    pf_weight.data[sp.id] = pf_log_likelihood(0, &sp);

    return 0;
}

static inline void pf_init_post(vsmc_particle particle)
{
    vsmc_weight_set_log(vsmc_particle_weight(particle), pf_weight.data, 1);
}

static inline void pf_move_pre(int t, vsmc_particle particle)
{
    pf_normal(particle, sqrt(0.02), sqrt(0.001));
}

static inline int pf_move_sp(int t, vsmc_single_particle sp)
{
    sp.state[PosX] += pf_pos_x.data[sp.id] + 0.1 * sp.state[VelX];
    sp.state[PosY] += pf_pos_y.data[sp.id] + 0.1 * sp.state[VelY];
    sp.state[VelX] += pf_vel_x.data[sp.id];
    sp.state[VelY] += pf_vel_y.data[sp.id];
    pf_weight.data[sp.id] = pf_log_likelihood(t, &sp);

    return 0;
}

static inline void pf_move_post(int t, vsmc_particle particle)
{
    vsmc_weight_add_log(vsmc_particle_weight(particle), pf_weight.data, 1);
}

static inline void pf_eval_sp(
    int t, int dim, vsmc_single_particle sp, double *r)
{
    r[0] = sp.state[PosX];
    r[1] = sp.state[PosY];
}

int main(int argc, char **argv)
{
    int N = 10000;
    if (argc > 1)
        N = atoi(argv[1]);

    vsmc_sampler sampler = vsmc_sampler_new(N, 4, vSMCMultinomial, 0.5);
    vsmc_sampler_init_tbb(
        sampler, pf_init_sp, pf_init_param, pf_init_pre, pf_init_post);
    vsmc_sampler_move_tbb(sampler, pf_move_sp, pf_move_pre, pf_move_post, 0);
    vsmc_sampler_set_monitor_tbb(
        sampler, "pos", 2, pf_eval_sp, NULL, NULL, 0, vSMCMonitorMCMC);

    vsmc_stop_watch watch = vsmc_stop_watch_new();
    vsmc_stop_watch_start(watch);
    vsmc_sampler_initialize(sampler, (void *) "pf.data");
    vsmc_sampler_iterate(sampler, pf_obs_x.size - 1);
    vsmc_stop_watch_stop(watch);
    printf("Time (ms): %lg\n", vsmc_stop_watch_milliseconds(watch));
    vsmc_stop_watch_delete(&watch);

    vsmc_sampler_save_f(sampler, "pf.out");

    vsmc_sampler_delete(&sampler);
    vsmc_vector_delete(&pf_obs_x);
    vsmc_vector_delete(&pf_obs_y);
    vsmc_vector_delete(&pf_pos_x);
    vsmc_vector_delete(&pf_pos_y);
    vsmc_vector_delete(&pf_vel_x);
    vsmc_vector_delete(&pf_vel_y);
    vsmc_vector_delete(&pf_weight);

    return 0;
}
