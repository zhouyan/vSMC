#include "pf.h"

static inline void pf_init_pre(size_t t, vsmc_particle particle)
{
    pf_normal(particle, 2, 1);
}

static inline size_t pf_init_sp(size_t t, vsmc_single_particle sp)
{
    sp.state[PosX] = pf_pos_x[sp.id];
    sp.state[PosY] = pf_pos_y[sp.id];
    sp.state[VelX] = pf_vel_x[sp.id];
    sp.state[VelY] = pf_vel_y[sp.id];

    return 0;
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

    return 0;
}

static inline size_t pf_weight_sp(size_t t, vsmc_single_particle sp)
{
    pf_inc_w[sp.id] = pf_log_likelihood(t, sp);

    return 0;
}

static inline void pf_weight_post(size_t t, vsmc_particle particle)
{
    vsmc_weight_add_log(vsmc_particle_weight(particle), pf_inc_w, 1);
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

    pf_malloc(n);

    vsmc_sampler_eval_smp_type pf_init = {pf_init_sp, pf_init_pre, NULL};
    vsmc_sampler_eval_smp_type pf_move = {pf_move_sp, pf_move_pre, NULL};
    vsmc_sampler_eval_smp_type pf_weight = {
        pf_weight_sp, pf_weight_post, NULL};
    vsmc_monitor_eval_smp_type pf_eval = {pf_eval_sp, NULL, NULL};

    vsmc_sampler sampler = vsmc_sampler_new(n, 4);
    vsmc_sampler_resample_scheme(sampler, vSMCMultinomial, 0.5);
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
