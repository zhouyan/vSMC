#ifndef VSMC_CAPI_H
#define VSMC_CAPI_H

#include <stddef.h>

// Resample scheme
#define VSMC_RESAMPLE_MULTINOMIAL          101
#define VSMC_RESAMPLE_RESIDUAL             102
#define VSMC_RESAMPLE_STRATIFIED           103
#define VSMC_RESAMPLE_SYSTEMATIC           104
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED  105
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC  106

// Base class types
#define VSMC_BASE_SEQ  201
#define VSMC_BASE_CILK 202
#define VSMC_BASE_OMP  203
#define VSMC_BASE_STD  204
#define VSMC_BASE_TBB  205

// Matrix order
#define VSMC_COLUMN_MAJOR 301
#define VSMC_ROW_MAJOR    302

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct
{
    void *value_ptr;
    int base_type;
} vsmcValue;

typedef struct
{
    void *sampler_ptr;
    int base_type;
} vsmcSampler;

typedef struct
{
    void *particle_ptr;
    int base_type;
} vsmcParticle;

typedef struct
{
    void *monitor_ptr;
    int base_type;
} vsmcMonitor;

typedef struct
{
    void *path_ptr;
    int base_type;
} vsmcPath;

typedef struct
{
    void *initialize_ptr;
    int base_type;
} vsmcInitialize;

typedef struct
{
    void *monitor_eval_ptr;
    int base_type;
} vsmcMonitorEval;

typedef struct
{
    void *path_eval_ptr;
    int base_type;
} vsmcPathEval;

typedef struct
{
    void *init_ptr;
    int base_type;
} vsmcInit;

typedef struct
{
    void *move_ptr;
    int base_type;
} vsmcMove;

// vsmc::Sampler
vsmcSampler  vsmc_sampler_new (size_t, unsigned, int, double, int);
void vsmc_sampler_delete (vsmcSampler);
vsmcParticle vsmc_sampler_particle (vsmcSampler);
size_t   vsmc_sampler_size                   (vsmcSampler);
unsigned vsmc_sampler_iter_size              (vsmcSampler);
void     vsmc_sampler_resample_scheme        (vsmcSampler, int);
double   vsmc_sampler_resample_threshold     (vsmcSampler, double);
void     vsmc_sampler_read_ess_history       (vsmcSampler, double *);
void     vsmc_sampler_read_resampled_history (vsmcSampler, int *);
void     vsmc_sampler_initialize             (vsmcSampler, void *);
void     vsmc_sampler_iterate                (vsmcSampler, unsigned);
void     vsmc_sampler_show_progress          (vsmcSampler, int);
void     vsmc_sampler_init                   (vsmcSampler, vsmcInit);
void     vsmc_sampler_move                   (vsmcSampler, vsmcMove);
void     vsmc_sampler_move_queue_clear       (vsmcSampler);
void     vsmc_sampler_move_queue_push_back   (vsmcSampler, vsmcMove);
void     vsmc_sampler_mcmc                   (vsmcSampler, vsmcMove);
void     vsmc_sampler_mcmc_queue_clear       (vsmcSampler);
void     vsmc_sampler_mcmc_queue_push_back   (vsmcSampler, vsmcMove);
void     vsmc_sampler_print                  (vsmcSampler, const char *);

// vsmc::Particle
vsmcValue vsmc_particle_value (vsmcParticle);
size_t vsmc_particle_size             (vsmcParticle);
void   vsmc_particle_resample         (vsmcParticle, double);
int    vsmc_particle_resampled        (vsmcParticle);
void   vsmc_particle_resample_scheme  (vsmcParticle, int);
void   vsmc_particle_read_weight      (vsmcParticle, double *);
void   vsmc_particle_read_log_weight  (vsmcParticle, double *);
double vsmc_particle_weight           (vsmcParticle, size_t);
double vsmc_particle_log_weight       (vsmcParticle, size_t);
void   vsmc_particle_set_equal_weight (vsmcParticle);
void   vsmc_particle_set_weight       (vsmcParticle, const double *, int);
void   vsmc_particle_mul_weight       (vsmcParticle, const double *, int);
void   vsmc_particle_set_log_weight   (vsmcParticle, const double *, int);
void   vsmc_particle_add_log_weight   (vsmcParticle, const double *, int);
double vsmc_particle_ess              (vsmcParticle);

// vsmc::Monitor
vsmcMonitor vsmc_monitor_new (vsmcSampler, unsigned, vsmcMonitorEval);
void     vsmc_monitor_delete             (vsmcMonitor);
unsigned vsmc_monitor_dim                (vsmcMonitor);
unsigned vsmc_monitor_iter_size          (vsmcMonitor);
void     vsmc_monitor_read_index         (vsmcMonitor, unsigned *);
double  *vsmc_monitor_read_record        (vsmcMonitor, unsigned, double *);
double  *vsmc_monitor_read_record_matrix (vsmcMonitor, int, double *);

// vsmc::Path
vsmcPath vsmc_path_new (vsmcSampler, vsmcPathEval);
void     vsmc_path_delete         (vsmcPath);
unsigned vsmc_path_iter_size      (vsmcPath);
void     vsmc_path_read_index     (vsmcPath, unsigned *);
void     vsmc_path_read_integrand (vsmcPath, double *);
void     vsmc_path_read_width     (vsmcPath, double *);
void     vsmc_path_read_grid      (vsmcPath, double *);

// vsmc::Samler::value_type
unsigned vsmc_value_dim              (vsmcValue);
void     vsmc_value_rezie_dim        (vsmcValue, unsigned);
size_t   vsmc_value_size             (vsmcValue);
double   vsmc_value_state            (vsmcValue, size_t, unsigned);
double  *vsmc_value_ptr              (vsmcValue, size_t);
double  *vsmc_value_read_state       (vsmcValue, unsigned, double *);
double  *vsmc_value_read_state_matrix(vsmcValue, int, double *);

// vsmc::Sampler::init_type
vsmcInit vsmc_init_new (vsmcSampler,
        unsigned (*) (unsigned, double *),
        void (*) (vsmcParticle, void *),
        void (*) (vsmcParticle),
        void (*) (vsmcParticle));
void vsmc_init_delete (vsmcInit);

// vsmc::Sampler::move_type
vsmcMove vsmc_move_new (vsmcSampler,
        unsigned (*) (unsigned, unsigned, double *),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_move_delete (vsmcMove);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // VSMC_CAPI_H
