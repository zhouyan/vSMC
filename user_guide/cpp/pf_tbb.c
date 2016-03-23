#include <vsmc/vsmc.h>
#include <math.h>
#include <stdio.h>

static const int N = 1000; // Number of particles
static const int n = 100;  // Number of data points
static const int PosX = 0;
static const int PosY = 1;
static const int VelX = 2;
static const int VelY = 3;

static double pf_obs_x[n];
static double pf_obs_y[n];

// Temporaries used by pf_init_* and pf_move_*
static double pf_pos_x[N];
static double pf_pos_y[N];
static double pf_vel_x[N];
static double pf_vel_y[N];
static double pf_weight[N];

double pf_log_likelihood(int, const vsmc_single_particle *);
void pf_read_data(const char *);
int pf_init_sp(vsmc_single_particle);
void pf_init_param(vsmc_particle, void *);
void pf_init_pre(vsmc_particle);
void pf_init_post(vsmc_particle);
int pf_move_sp(int, vsmc_single_particle);
void pf_move_pre(int, vsmc_particle);
void pf_move_post(int, vsmc_particle);
void pf_eval_sp(int, int, vsmc_single_particle, double *);

double pf_log_likelihood(int t, const vsmc_single_particle *sp)
{
    double llh_x = 10 * (sp->state[PosX] - pf_obs_x[t]);
    double llh_y = 10 * (sp->state[PosY] - pf_obs_y[t]);
    llh_x = log(1 + llh_x * llh_x / 10);
    llh_y = log(1 + llh_y * llh_y / 10);

    return -0.5 * (10 + 1) * (llh_x + llh_y);
}

void pf_read_data(const char *param)
{
    if (!param)
        return;

    FILE *data = fopen(param, "r");
    for (int i = 0; i < n; ++i) {
        fscanf(data, "%lg ", pf_obs_x + i);
        fscanf(data, "%lg ", pf_obs_y + i);
    }
    fclose(data);
}

void pf_init_param(vsmc_particle particle, void *param)
{
    pf_read_data((const char *) param);
}

void pf_init_pre(vsmc_particle particle)
{
    vsmc_rng rng = vsmc_particle_rng(particle, 0);
    const int size = vsmc_particle_size(particle);
    const double sd_pos = 2;
    const double sd_vel = 1;
    vsmc_normal_distribution(rng, size, pf_pos_x, 0, sd_pos);
    vsmc_normal_distribution(rng, size, pf_pos_y, 0, sd_pos);
    vsmc_normal_distribution(rng, size, pf_vel_x, 0, sd_vel);
    vsmc_normal_distribution(rng, size, pf_vel_y, 0, sd_vel);
}

int pf_init_sp(vsmc_single_particle sp)
{
    sp.state[PosX] = pf_pos_x[sp.id];
    sp.state[PosY] = pf_pos_y[sp.id];
    sp.state[VelX] = pf_vel_x[sp.id];
    sp.state[VelY] = pf_vel_y[sp.id];
    pf_weight[sp.id] = pf_log_likelihood(0, &sp);

    return 0;
}

void pf_init_post(vsmc_particle particle)
{
    vsmc_weight_set_log(vsmc_particle_weight(particle), pf_weight, 1);
}

void pf_move_pre(int t, vsmc_particle particle)
{
    vsmc_rng rng = vsmc_particle_rng(particle, 0);
    const int size = vsmc_particle_size(particle);
    const double sd_pos = sqrt(0.02);
    const double sd_vel = sqrt(0.001);
    vsmc_normal_distribution(rng, size, pf_pos_x, 0, sd_pos);
    vsmc_normal_distribution(rng, size, pf_pos_y, 0, sd_pos);
    vsmc_normal_distribution(rng, size, pf_vel_x, 0, sd_vel);
    vsmc_normal_distribution(rng, size, pf_vel_y, 0, sd_vel);
}

int pf_move_sp(int t, vsmc_single_particle sp)
{
    sp.state[PosX] += pf_pos_x[sp.id] + 0.1 * sp.state[VelX];
    sp.state[PosY] += pf_pos_y[sp.id] + 0.1 * sp.state[VelY];
    sp.state[VelX] += pf_vel_x[sp.id];
    sp.state[VelY] += pf_vel_y[sp.id];
    pf_weight[sp.id] = pf_log_likelihood(t, &sp);

    return 0;
}

void pf_move_post(int t, vsmc_particle particle)
{
    vsmc_weight_add_log(vsmc_particle_weight(particle), pf_weight, 1);
}

void pf_eval_sp(int t, int dim, vsmc_single_particle sp, double *r)
{
    r[0] = sp.state[PosX];
    r[1] = sp.state[PosY];
}

int main()
{
    vsmc_sampler sampler;
    vsmc_sampler_malloc(&sampler, N, 4, vSMCMultinomial, 0.5);
    vsmc_sampler_init_tbb(
        sampler, pf_init_sp, pf_init_param, pf_init_pre, pf_init_post);
    vsmc_sampler_move_tbb(sampler, pf_move_sp, pf_move_pre, pf_move_post, 0);
    vsmc_sampler_set_monitor_tbb(
        sampler, "pos", 2, pf_eval_sp, NULL, NULL, 0, vSMCMonitorMCMC);

    vsmc_sampler_initialize(sampler, (void *) "pf.data");
    vsmc_sampler_iterate(sampler, n - 1);

    vsmc_sampler_save_f(sampler, "pf.out");

    return 0;
}
