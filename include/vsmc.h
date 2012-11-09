#ifndef VSMC_H
#define VSMC_H

#include <stddef.h>

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#ifndef VSMC_USE_RANDOM123
#define VSMC_USE_RANDOM123 1
#endif

#if VSMC_USE_RANDOM123
#define R123_USE_U01_DOUBLE 1
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#endif // VSMC_USE_RANDOM123

/* Resample scheme */
#define VSMC_RESAMPLE_MULTINOMIAL          101
#define VSMC_RESAMPLE_RESIDUAL             102
#define VSMC_RESAMPLE_STRATIFIED           103
#define VSMC_RESAMPLE_SYSTEMATIC           104
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED  105
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC  106

/* Base class types */
#define VSMC_BASE_SEQ  201
#define VSMC_BASE_CILK 202
#define VSMC_BASE_OMP  203
#define VSMC_BASE_STD  204
#define VSMC_BASE_TBB  205

/* Matrix order */
#define VSMC_COLUMN_MAJOR 301
#define VSMC_ROW_MAJOR    302

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* C type of vSMC templates */
typedef struct {
    void *value_ptr;
    int base_type;
} vsmcValue;

typedef struct {
    void *sampler_ptr;
    int base_type;
} vsmcSampler;

typedef struct {
    void *particle_ptr;
    int base_type;
} vsmcParticle;

typedef struct {
    void *monitor_ptr;
    int base_type;
} vsmcMonitor;

typedef struct {
    void *path_ptr;
    int base_type;
} vsmcPath;

typedef struct {
    void *initialize_ptr;
    int base_type;
} vsmcInitialize;

typedef struct {
    void *move_ptr;
    int base_type;
} vsmcMove;

typedef struct {
    void *monitor_eval_ptr;
    int base_type;
} vsmcMonitorEval;

typedef struct {
    void *path_eval_ptr;
    int base_type;
} vsmcPathEval;

/* vsmc::Seed */
unsigned vsmc_seed_get  (void);
void     vsmc_seed_set  (unsigned);
void     vsmc_seed_skip (unsigned);

/* vsmc::Samler::value_type */
unsigned vsmc_value_dim              (vsmcValue);
void     vsmc_value_rezie_dim        (vsmcValue, unsigned);
size_t   vsmc_value_size             (vsmcValue);
double   vsmc_value_state            (vsmcValue, size_t, unsigned);
double  *vsmc_value_state_ptr        (vsmcValue, size_t);
double  *vsmc_value_read_state       (vsmcValue, unsigned, double *);
double  *vsmc_value_read_state_matrix(vsmcValue, int, double *);

/* vsmc::Sampler */
vsmcSampler  vsmc_sampler_new (size_t, unsigned, int, double, int);
void         vsmc_sampler_delete                 (vsmcSampler);
size_t       vsmc_sampler_size                   (vsmcSampler);
unsigned     vsmc_sampler_iter_size              (vsmcSampler);
void         vsmc_sampler_resample_scheme        (vsmcSampler, int);
double       vsmc_sampler_resample_threshold     (vsmcSampler, double);
double       vsmc_sampler_ess_history            (vsmcSampler, unsigned);
void         vsmc_sampler_read_ess_history       (vsmcSampler, double *);
int          vsmc_sampler_resampled_history      (vsmcSampler, unsigned);
void         vsmc_sampler_read_resampled_history (vsmcSampler, int *);
unsigned     vsmc_sampler_move_num               (vsmcSampler, unsigned);
unsigned     vsmc_sampler_accept_history         (vsmcSampler, unsigned,
                                                  unsigned);
vsmcParticle vsmc_sampler_particle               (vsmcSampler);
void         vsmc_sampler_init                   (vsmcSampler, vsmcInitialize);
void         vsmc_sampler_move                   (vsmcSampler, vsmcMove);
void         vsmc_sampler_move_queue_clear       (vsmcSampler);
void         vsmc_sampler_move_queue_push_back   (vsmcSampler, vsmcMove);
void         vsmc_sampler_mcmc                   (vsmcSampler, vsmcMove);
void         vsmc_sampler_mcmc_queue_clear       (vsmcSampler);
void         vsmc_sampler_mcmc_queue_push_back   (vsmcSampler, vsmcMove);
void         vsmc_sampler_initialize             (vsmcSampler, void *);
void         vsmc_sampler_iterate                (vsmcSampler, unsigned);
vsmcMonitor  vsmc_sampler_monitor                (vsmcSampler, const char *,
                                                  unsigned, vsmcMonitorEval);
void         vsmc_sampler_clear_monitor          (vsmcSampler);
vsmcPath     vsmc_sampler_path                   (vsmcSampler);
double       vsmc_sampler_path_sampling          (vsmcSampler, vsmcPathEval);
void         vsmc_sampler_print                  (vsmcSampler, const char *,
                                                  unsigned);
void         vsmc_sampler_show_progress          (vsmcSampler, int);

/* vsmc::Particle */
size_t    vsmc_particle_size             (vsmcParticle);
vsmcValue vsmc_particle_value            (vsmcParticle);
void      vsmc_particle_resample         (vsmcParticle, double);
int       vsmc_particle_resampled        (vsmcParticle);
void      vsmc_particle_resample_scheme  (vsmcParticle, int);
void      vsmc_particle_read_weight      (vsmcParticle, double *);
void      vsmc_particle_read_log_weight  (vsmcParticle, double *);
double    vsmc_particle_weight           (vsmcParticle, size_t);
double    vsmc_particle_log_weight       (vsmcParticle, size_t);
void      vsmc_particle_set_equal_weight (vsmcParticle);
void      vsmc_particle_set_weight       (vsmcParticle, const double *, int);
void      vsmc_particle_mul_weight       (vsmcParticle, const double *, int);
void      vsmc_particle_set_log_weight   (vsmcParticle, const double *, int);
void      vsmc_particle_add_log_weight   (vsmcParticle, const double *, int);
double    vsmc_particle_ess              (vsmcParticle);

/* vsmc::Monitor */
vsmcMonitor vsmc_monitor_new (vsmcSampler, unsigned, vsmcMonitorEval);
void      vsmc_monitor_delete             (vsmcMonitor);
unsigned  vsmc_monitor_dim                (vsmcMonitor);
unsigned  vsmc_monitor_iter_size          (vsmcMonitor);
unsigned  vsmc_monitor_index              (vsmcMonitor, unsigned);
double    vsmc_monitor_record             (vsmcMonitor, unsigned, unsigned);
unsigned *vsmc_monitor_read_index         (vsmcMonitor, unsigned *);
double   *vsmc_monitor_read_record        (vsmcMonitor, unsigned, double *);
double   *vsmc_monitor_read_record_matrix (vsmcMonitor, int, double *);
void      vsmc_monitor_clear              (vsmcMonitor);

/* vsmc::Path */
vsmcPath vsmc_path_new (vsmcSampler, vsmcPathEval);
void      vsmc_path_delete         (vsmcPath);
unsigned  vsmc_path_iter_size      (vsmcPath);
unsigned  vsmc_path_index          (vsmcPath, unsigned);
double    vsmc_path_integrand      (vsmcPath, unsigned);
double    vsmc_path_width          (vsmcPath, unsigned);
double    vsmc_path_grid           (vsmcPath, unsigned);
unsigned *vsmc_path_read_index     (vsmcPath, unsigned *);
double   *vsmc_path_read_integrand (vsmcPath, double *);
double   *vsmc_path_read_width     (vsmcPath, double *);
double   *vsmc_path_read_grid      (vsmcPath, double *);
void      vsmc_path_clear          (vsmcPath);

/* vsmc::Sampler::init_type */
vsmcInitialize vsmc_init_new (vsmcSampler,
        unsigned (*) (size_t, size_t, unsigned, double *),
        void (*) (vsmcParticle, void *),
        void (*) (vsmcParticle),
        void (*) (vsmcParticle));
void vsmc_init_delete (vsmcInitialize);

/* vsmc::Sampler::move_type */
vsmcMove vsmc_move_new (vsmcSampler,
        unsigned (*) (size_t, size_t, unsigned, unsigned, double *),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_move_delete (vsmcMove);

/* vsmc::Monitor::eval_type */
vsmcMonitorEval vsmc_monitor_eval_new (vsmcSampler,
        void (*) (size_t, size_t, unsigned, unsigned, unsigned,
            const double *, double *),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_monitor_eval_delete (vsmcMonitorEval eval);

/* vsmc::Path::eval_type */
vsmcPathEval vsmc_path_eval_new (vsmcSampler,
        double (*) (size_t, size_t, unsigned, unsigned, const double *),
        double (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_path_eval_delete (vsmcPathEval);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* VSMC_H */
