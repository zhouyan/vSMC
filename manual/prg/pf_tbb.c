#include "pf.h"

static inline void pf_init_first(size_t t, vsmc_particle particle)
{
    pf_normal(particle, 2, 1);
}

static inline void pf_init_each(size_t t, vsmc_particle_index idx)
{
    idx.state[PosX] = pf_pos_x[idx.id];
    idx.state[PosY] = pf_pos_y[idx.id];
    idx.state[VelX] = pf_vel_x[idx.id];
    idx.state[VelY] = pf_vel_y[idx.id];
}

static inline void pf_move_first(size_t t, vsmc_particle particle)
{
    pf_normal(particle, sqrt(0.02), sqrt(0.001));
}

static inline void pf_move_each(size_t t, vsmc_particle_index idx)
{
    idx.state[PosX] += pf_pos_x[idx.id] + 0.1 * idx.state[VelX];
    idx.state[PosY] += pf_pos_y[idx.id] + 0.1 * idx.state[VelY];
    idx.state[VelX] += pf_vel_x[idx.id];
    idx.state[VelY] += pf_vel_y[idx.id];
}

static inline void pf_weight_each(size_t t, vsmc_particle_index idx)
{
    pf_inc_w[idx.id] = pf_log_likelihood(t, idx);
}

static inline void pf_weight_last(size_t t, vsmc_particle particle)
{
    vsmc_weight_add_log(vsmc_particle_weight(particle), pf_inc_w, 1);
}

static inline void pf_eval_each(
    size_t t, size_t dim, vsmc_particle_index idx, double *r)
{
    r[0] = idx.state[PosX];
    r[1] = idx.state[PosY];
}

int main(int argc, char **argv)
{
    size_t n = 10000;
    if (argc > 1)
        n = (size_t) atoi(argv[1]);

    pf_malloc(n);

    vsmc_sampler_eval_smp_type pf_init = {pf_init_each, pf_init_first, NULL};
    vsmc_sampler_eval_smp_type pf_move = {pf_move_each, pf_move_first, NULL};
    vsmc_sampler_eval_smp_type pf_weight = {
        pf_weight_each, NULL, pf_weight_last};
    vsmc_monitor_eval_smp_type pf_eval = {pf_eval_each, NULL, NULL};

    vsmc_sampler sampler = vsmc_sampler_new(n, 4);
    vsmc_sampler_resample_scheme(sampler, vSMCStratified, 0.5);
    vsmc_sampler_eval_smp(
        vSMCBackendTBB, sampler, pf_init, vSMCSamplerInit, 1);
    vsmc_sampler_eval_smp(
        vSMCBackendTBB, sampler, pf_move, vSMCSamplerMove, 1);
    vsmc_sampler_eval_smp(vSMCBackendTBB, sampler, pf_weight,
        vSMCSamplerInit | vSMCSamplerMove, 1);
    vsmc_monitor pf_pos =
        vsmc_monitor_new_smp(vSMCBackendTBB, 2, pf_eval, 0, vSMCMonitorMCMC);
    vsmc_sampler_set_monitor(sampler, "pos", pf_pos);
    vsmc_monitor_delete(&pf_pos);
    vsmc_sampler_initialize(sampler);
    vsmc_sampler_iterate(sampler, pf_obs_size - 1);
    vsmc_sampler_print_f(sampler, "pf.out", '\t');
    vsmc_sampler_delete(&sampler);

    pf_free();

    return 0;
}
