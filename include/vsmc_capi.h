#ifndef VSMC_CAPI_H
#define VSMC_CAPI_H

#include <stddef.h>

/* Resample scheme */

#define VSMC_RESAMPLE_MULTINOMIAL           101
#define VSMC_RESAMPLE_RESIDUAL              102
#define VSMC_RESAMPLE_STRATIFIED            103
#define VSMC_RESAMPLE_SYSTEMATIC            104
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED   105
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC   106

/* Base class types */

#define VSMC_BASE_SEQUENTIAL 201
#define VSMC_BASE_CILK       202
#define VSMC_BASE_OMP        203
#define VSMC_BASE_TBB        204
#define VSMC_BASE_THREAD     205

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct
{
    void *sampler_ptr;
    int base_type;
} vsmcSamplerInfo;

typedef struct
{
    void *particle_ptr;
    int base_type;
} vsmcParticleInfo;

typedef struct
{
    void *monitor_ptr;
    int base_type;
} vsmcMonitorInfo;

typedef struct
{
    void *monitor_ptr;
    int base_type;
} vsmcPathInfo;

/* vsmc::Sampler */
vsmcSamplerInfo  vsmc_sampler_new (size_t, unsigned, int, double, int);
vsmcParticleInfo vsmc_sampler_particle (vsmcSamplerInfo);
void     vsmc_sampler_delete                 (vsmcSamplerInfo);
size_t   vsmc_sampler_size                   (vsmcSamplerInfo);
unsigned vsmc_sampler_iter_size              (vsmcSamplerInfo);
void     vsmc_sampler_resample_scheme        (vsmcSamplerInfo, int);
double   vsmc_sampler_resample_threshold     (vsmcSamplerInfo, double);
void     vsmc_sampler_read_ess_history       (vsmcSamplerInfo, double *);
void     vsmc_sampler_read_resampled_history (vsmcSamplerInfo, int *);
void     vsmc_sampler_initialize             (vsmcSamplerInfo, void *);
void     vsmc_sampler_iterate                (vsmcSamplerInfo, unsigned);
void     vsmc_sampler_show_progress          (vsmcSamplerInfo, int);

/* vsmc::Particle */
size_t vsmc_particle_size             (vsmcParticleInfo)
void   vsmc_particle_resample         (vsmcParticleInfo, double)
int    vsmc_particle_resampled        (vsmcParticleInfo)
void   vsmc_particle_resample_scheme  (vsmcParticleInfo, int)
void   vsmc_particle_read_weight      (vsmcParticleInfo, double *)
void   vsmc_particle_read_log_weight  (vsmcParticleInfo, double *)
double vsmc_particle_weight           (vsmcParticleInfo, size_t)
double vsmc_particle_log_weight       (vsmcParticleInfo, size_t)
void   vsmc_particle_set_equal_weight (vsmcParticleInfo)
void   vsmc_particle_set_weight       (vsmcParticleInfo, const double *)
void   vsmc_particle_mul_weight       (vsmcParticleInfo, const double *)
void   vsmc_particle_set_log_weight   (vsmcParticleInfo, const double *)
void   vsmc_particle_add_log_weight   (vsmcParticleInfo, const double *)
double vsmc_particle_ess              (vsmcParticleInfo info)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // VSMC_CAPI_H
