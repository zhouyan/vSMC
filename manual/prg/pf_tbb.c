#include <vsmc/vsmc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static const size_t PosX = 0;
static const size_t PosY = 1;
static const size_t VelX = 2;
static const size_t VelY = 3;

// Storage for data
static size_t pf_obs_size = 0;
static double *pf_obs_x = NULL;
static double *pf_obs_y = NULL;

// Temporaries used by pf_init and pf_move
static double *pf_pos_x = NULL;
static double *pf_pos_y = NULL;
static double *pf_vel_x = NULL;
static double *pf_vel_y = NULL;
static double *pf_weight = NULL;

static inline double pf_log_likelihood(
    size_t t, const vsmc_single_particle *sp)
{
    double llh_x = 10 * (sp->state[PosX] - pf_obs_x[t]);
    double llh_y = 10 * (sp->state[PosY] - pf_obs_y[t]);
    llh_x = log(1 + llh_x * llh_x / 10);
    llh_y = log(1 + llh_y * llh_y / 10);

    return -0.5 * (10 + 1) * (llh_x + llh_y);
}

static inline void pf_read_data(const char *param)
{
    if (!param)
        return;

    FILE *data = fopen(param, "r");
    size_t n = 0;
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
    pf_obs_size = n;
    pf_obs_x = vsmc_malloc(n * sizeof(double), 32);
    pf_obs_y = vsmc_malloc(n * sizeof(double), 32);
    fseek(data, 0, SEEK_SET);
    for (size_t i = 0; i < n; ++i) {
        fscanf(data, "%lg", &pf_obs_x[i]);
        fscanf(data, "%lg", &pf_obs_y[i]);
    }
    fclose(data);
}

static inline void pf_normal(
    vsmc_particle particle, double sd_pos, double sd_vel)
{
    vsmc_rng rng = vsmc_particle_rng(particle, 0);
    const size_t size = vsmc_particle_size(particle);
    pf_pos_x = vsmc_malloc(size * sizeof(double), 32);
    pf_pos_y = vsmc_malloc(size * sizeof(double), 32);
    pf_vel_x = vsmc_malloc(size * sizeof(double), 32);
    pf_vel_y = vsmc_malloc(size * sizeof(double), 32);
    pf_weight = vsmc_malloc(size * sizeof(double), 32);
    vsmc_rand_normal(rng, size, pf_pos_x, 0, sd_pos);
    vsmc_rand_normal(rng, size, pf_pos_y, 0, sd_pos);
    vsmc_rand_normal(rng, size, pf_vel_x, 0, sd_vel);
    vsmc_rand_normal(rng, size, pf_vel_y, 0, sd_vel);
}

static inline void pf_init_param(vsmc_particle particle, void *param)
{
    pf_read_data((const char *) param);
}

static inline void pf_init_pre(vsmc_particle particle)
{
    pf_normal(particle, 2, 1);
}

static inline size_t pf_init_sp(vsmc_single_particle sp)
{
    sp.state[PosX] = pf_pos_x[sp.id];
    sp.state[PosY] = pf_pos_y[sp.id];
    sp.state[VelX] = pf_vel_x[sp.id];
    sp.state[VelY] = pf_vel_y[sp.id];
    pf_weight[sp.id] = pf_log_likelihood(0, &sp);

    return 0;
}

static inline void pf_init_post(vsmc_particle particle)
{
    vsmc_weight_set_log(vsmc_particle_weight(particle), pf_weight, 1);
}

static inline void pf_move_pre(size_t t, vsmc_particle particle)
{
    pf_normal(particle, sqrt(0.02), sqrt(0.001));
}

static inline size_t pf_move_sp(size_t t, vsmc_single_particle sp)
{
    sp.state[PosX] += pf_pos_x[sp.id] + 0.1 * sp.state[VelX];
    sp.state[PosY] += pf_pos_y[sp.id] + 0.1 * sp.state[VelY];
    sp.state[VelX] += pf_vel_x[sp.id];
    sp.state[VelY] += pf_vel_y[sp.id];
    pf_weight[sp.id] = pf_log_likelihood(t, &sp);

    return 0;
}

static inline void pf_move_post(size_t t, vsmc_particle particle)
{
    vsmc_weight_add_log(vsmc_particle_weight(particle), pf_weight, 1);
}

static inline void pf_eval_sp(
    size_t t, size_t dim, vsmc_single_particle sp, double *r)
{
    r[0] = sp.state[PosX];
    r[1] = sp.state[PosY];
}

int main(int argc, char **argv)
{
    size_t n = 10000;
    if (argc > 1)
        n = (size_t) atoi(argv[1]);

    vsmc_sampler_init_smp_type pf_init = {
        pf_init_sp, pf_init_param, pf_init_pre, pf_init_post};
    vsmc_sampler_move_smp_type pf_move = {
        pf_move_sp, pf_move_pre, pf_move_post};
    vsmc_monitor_eval_smp_type pf_eval = {pf_eval_sp, NULL, NULL};
    vsmc_monitor pf_pos =
        vsmc_monitor_new_smp(vSMCBackendTBB, 2, pf_eval, 0, vSMCMonitorMCMC);

    vsmc_sampler sampler = vsmc_sampler_new(n, 4);
    vsmc_sampler_resample_scheme(sampler, vSMCMultinomial, 0.5);
    vsmc_sampler_init_smp(vSMCBackendTBB, sampler, pf_init, 0);
    vsmc_sampler_move_smp(vSMCBackendTBB, sampler, pf_move, 0);
    vsmc_sampler_set_monitor(sampler, "pos", pf_pos);
    vsmc_monitor_delete(&pf_pos);

    vsmc_sampler_initialize(sampler, (void *) "pf.data");
    vsmc_sampler_iterate(sampler, pf_obs_size - 1);
    vsmc_sampler_print_f(sampler, "pf.out", '\t');
    vsmc_sampler_delete(&sampler);

    vsmc_free(pf_obs_x);
    vsmc_free(pf_obs_y);
    vsmc_free(pf_pos_x);
    vsmc_free(pf_pos_y);
    vsmc_free(pf_vel_x);
    vsmc_free(pf_vel_y);
    vsmc_free(pf_weight);

    return 0;
}
