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

/**
 * \defgroup CAPI C API
 * \brief C API to a subset of vSMC functionalities
 *
 * \details
 * The C API can be used to access vSMC when the value collection is of a class
 * of particular types, subclasses of `vsmc::StateBase<vsmc::Dynamic, double>`,
 * namely `vsmc::StateSEQ<vsmc::Dynamic, double>` etc. For example if one use
 * the C API to construct a sampler of type VSMC_BASE_SEQ, then it will be a
 * sampler for `vsmc::StateSEQ<vsmc::Dynamic, double>` value collection.  Only
 * those features avaiable when compiling the C library are accessible. The
 * avaible base types can be tested through vsmc_has_base_type(int).
 * */

/**
 * \defgroup CResample Resample scheme
 * \ingroup CAPI
 * \brief Macro definitions of resample scheme
 * */

/**
 * \defgroup CBase Base class types
 * \ingroup CAPI
 * \brief Macro definitions of base class types
 * */

/**
 * \defgroup MatrixOrder Matrix order
 * \ingroup CAPI
 * \brief Macro definitions of matrix orders used by some output functions
 * */

/**
 * \defgroup COBJ C types of vSMC classes
 * \ingroup CAPI
 * \brief C types representing vSMC class templates objects
 *
 * \details
 * Each type has two memebers, a pointer of type `void *` and an integer
 * `base_type`. The pointer points to a vSMC class object. For example,
 * `sampler_ptr` of vsmcSampler pionts to an `vsmc::Sampler<T>` object, where
 * `T`'s type is determined by the `base_type`. For instance, if `base_type` is
 * `VSMC_BASE_SEQ`, then `T` will be `vsmc::StateSEQ<vsmc::Dynamic, double>`
 *
 * In the documents of this module, when we refer to a class templates defined
 * in vSMC C++ headers, by `T` we refer to such a class specialization whose
 * actual type depends on the `base_type` member of the C type object involved.
 * By `Impl` we refer to a class template in the vSMC helper module that
 * implement the functionality. For example, when documting vsmcInitialize, we
 * say that it representing `vsmc::InitializeAdapter<T, Impl>`, then if
 * `base_type` is VSMC_BASE_SEQ, then `T` is `vsmc::StateSEQ<vsmc::Dynamic,
 * double>` and `Impl` is `vsmc::InitializeSEQ`.
 * */

/**
 * \defgroup CValue C API to vsmc::StateBase<vsmc::Dynamic, double>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::StateBase<vsmc::Dynamic, double>
 * object
 * */

/**
 * \defgroup CSampler C API to vsmc::Sampler<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::Sampler<T> object
 * */

/**
 * \defgroup CParticle C API to vsmc::Particle<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::Particle<T> object
 * */

/**
 * \defgroup CMonitor C API to vsmc::Monitor<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::Monitor<T> object
 * */

/**
 * \defgroup CPath C API to vsmc::Path<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::Path<T> object
 * */

/**
 * \defgroup CInitializeAdapter C API to vsmc::InitializeAdapter<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::InitializeAdapter<T, Impl> object
 * */

/**
 * \defgroup CMoveAdapter C API to vsmc::MoveAdapter<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::MoveAdapter<T, Impl> object
 * */

/**
 * \defgroup CMonitorEvalAdapter C API to vsmc::MonitorEvalAdapter<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::MonitorEvalAdapter<T, Impl>
 * object
 * */

/**
 * \defgroup CPathEvalAdapter C API to vsmc::PathEvalAdapter<T>
 * \ingroup CAPI
 * \brief C functions that manipulate a vsmc::PathEvalAdapter<T, Impl> object
 * */

/****************************************************************************/

/**
 * \brief Multinomial resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_MULTINOMIAL 101

/**
 * \brief Residual resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_RESIDUAL 102

/**
 * \brief Stratified resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_STRATIFIED 103
/**
 * \brief Systematic resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_SYSTEMATIC 104

/**
 * \brief Residual stratified resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED 105

/**
 * \brief Residual systematic resampling
 * \ingroup CResample
 * */
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC 106

/****************************************************************************/

/**
 * \brief Sequential implementation
 * \ingroup CBase
 * */
#define VSMC_BASE_SEQ 201
/**
 * \brief Parallel implementation using Intel Cilk Plus
 * \ingroup CBase
 * */
#define VSMC_BASE_CILK 202
/**
 * \brief Parallel implementation using OpenMP
 * \ingroup CBase
 * */
#define VSMC_BASE_OMP 203
/**
 * \brief Parallel implementation using C++11 threads
 * \ingroup CBase
 * */
#define VSMC_BASE_STD 204
/**
 * \brief Parallel implementation using Intel Threading Building Blocks
 * \ingroup CBase
 * */
#define VSMC_BASE_TBB 205

/****************************************************************************/

/**
 * \brief Column major matrix
 * \ingroup MatrixOrder
 * */
#define VSMC_COLUMN_MAJOR 301
/**
 * \brief Row major matrix
 * \ingroup MatrixOrder
 * */
#define VSMC_ROW_MAJOR 302

/****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************/

/**
 * \brief Given an integer value, determine if it equals to one of the valid
 * base type class macros
 * */
int vsmc_has_base_type (int);

/**
 * \brief A vsmc::StateBase<vsmc::Dynamic, double> subclass
 * \ingroup COBJ
 * */
typedef struct {
    void *value_ptr;
    int base_type;
} vsmcValue;

/**
 * \brief vsmc::Sampler<T>
 * \ingroup COBJ
 * */
typedef struct {
    void *sampler_ptr;
    int base_type;
} vsmcSampler;

/**
 * \brief vsmc::Particle<T>
 * \ingroup COBJ
 * */
typedef struct {
    void *particle_ptr;
    int base_type;
} vsmcParticle;

/**
 * \brief vsmc::Monitor<T>
 * \ingroup COBJ
 * */
typedef struct {
    void *monitor_ptr;
    int base_type;
} vsmcMonitor;

/**
 * \brief vsmc::Path<T>
 * \ingroup COBJ
 * */
typedef struct {
    void *path_ptr;
    int base_type;
} vsmcPath;

/**
 * \brief vsmc::InitializeAdapter<T, Impl>
 * \ingroup COBJ
 * */
typedef struct {
    void *initialize_ptr;
    int base_type;
} vsmcInitialize;

/**
 * \brief vsmc::MoveAdapter<T, Impl>
 * \ingroup COBJ
 * */
typedef struct {
    void *move_ptr;
    int base_type;
} vsmcMove;

/**
 * \brief vsmc::MonitorEvalAdapter<T, Impl>
 * \ingroup COBJ
 * */
typedef struct {
    void *monitor_eval_ptr;
    int base_type;
} vsmcMonitorEval;

/**
 * \brief vsmc::PathEvalAdapter<T, Impl>
 * \ingroup COBJ
 * */
typedef struct {
    void *path_eval_ptr;
    int base_type;
} vsmcPathEval;

/****************************************************************************/

/**
 * \brief Get a seed for a RNG
 * \ingroup CAPI
 * */
unsigned vsmc_seed_get (void);

/**
 * \brief Set the seed for a program
 * \ingroup CAPI
 * */
void vsmc_seed_set (unsigned seed);

/**
 * \brief Skip a given steps of seeds
 * \ingroup CAPI
 * */
void vsmc_seed_skip (unsigned steps);

/****************************************************************************/

/**
  * \brief value.value_ptr->dim()
  * \ingroup CValue
  * */
unsigned vsmc_value_dim (vsmcValue value);

/**
  * \brief value.value_ptr->resize_dim(dim)
  * \ingroup CValue
  * */
void vsmc_value_rezie_dim (vsmcValue value, unsigned dim);

/**
  * \brief value.value_ptr->size()
  * \ingroup CValue
  * */
size_t vsmc_value_size (vsmcValue value);

/**
  * \brief value.value_ptr->state(id, pos)
  * \ingroup CValue
  * */
double vsmc_value_state (vsmcValue value, size_t id, unsigned pos);

/**
  * \brief value.value_ptr->state(id)
  * \ingroup CValue
  * */
double *vsmc_value_state_ptr (vsmcValue value, size_t id);

/**
  * \brief value.value_ptr->read_state(id, first)
  * \ingroup CValue
  * */
double *vsmc_value_read_state (vsmcValue value, unsigned id, double *first);

/**
  * \brief value.value_ptr->read_state_matrix(order, first)
  * \ingroup CValue
  *
  * \param value A vsmcValue object
  * \param order Either VSMC_COLUMN_MAJOR or VSMC_ROW_MAJOR. It will be
  * `vsmc::ColumnMajor` or `vsmc::RowMajor` passed to the underlying C++ call,
  * respectively
  * \param first The output pointer
  * */
double *vsmc_value_read_state_matrix(vsmcValue value, int order,
        double *first);

/****************************************************************************/

/**
  * \brief new vsmc::Sampler<T>(N, scheme, threshold)
  * \ingroup CSampler
  *
  * \param N Number of particles
  * \param Dim The initial dimension of the value collection. Since within the
  * CAPI, the value collection type is a subclass of
  * `vsmc::StateBase<vsmc::Dynamic, double>`, the value collection's dimension
  * can be resized.
  * \param scheme VSMC_RESAMPLE_MULTINOMIAL etc. It will be corresponding C++
  * name, `vsmc::Multinomial` etc. passed to the constructor of the C++ class.
  * \param threshold The initial threshold of ESS/N below which resampling will
  * be performed
  * \param base_type The value collection type of the sampler
  *
  * \return An vsmcSampler object, with `sampler.sampler_ptr` be the address of
  * the newly created `vsmc::Sampler<T>` object if the constructing is sucessful,
  * or zero otherwise. `sampler.base_type` is the `base_type` passed to the
  * function.
  * */
vsmcSampler vsmc_sampler_new (size_t N, unsigned Dim,
        int scheme, double threshold, int base_type);

/**
  * \brief delete sampler.sampler_ptr
  * \ingroup CSampler
  * */
void vsmc_sampler_delete (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->size()
  * \ingroup CSampler
  * */
size_t vsmc_sampler_size (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->iter_size()
  * \ingroup CSampler
  * */
unsigned vsmc_sampler_iter_size (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->resample_scheme(scheme)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param scheme VSMC_RESAMPLE_MULTINOMIAL etc. It will be corresponding C++
  * name, `vsmc::Multinomial` etc. passed to the C++ call.
  *
  * \return Non-zero value if the `scheme` is valid and the changing of
  * resampling scheme is actually changed, otherwise zero.
  * */
int vsmc_sampler_resample_scheme (vsmcSampler sampler, int scheme);

/**
  * \brief Call vsmc::Sampler::threshold on the sampler
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param threshold The threshold of ESS/N. If threshold < 0, call
  * `sampler.sampler_ptr->threshold()`, otherwise call
  * `sampler.sampler_ptr->threshold(threshold)`
  *
  * \return The threshold after the (possible) change
  * */
double vsmc_sampler_resample_threshold (vsmcSampler sampler, double threshold);

/**
  * \brief sampler.sampler_ptr->ess_history(iter)
  * \ingroup CSampler
  * */
double vsmc_sampler_ess_history (vsmcSampler sampler, unsigned iter);

/**
  * \brief sampler.sampler_ptr->read_ess_history(first)
  * \ingroup CSampler
  * */
void vsmc_sampler_read_ess_history (vsmcSampler sampler, double *first);

/**
  * \brief sampler.sampler_ptr->resampled_history(iter)
  * \ingroup CSampler
  * */
int vsmc_sampler_resampled_history (vsmcSampler sampler, unsigned iter);

/**
  * \brief sampler.sampler_ptr->read_resampled_history(first)
  * \ingroup CSampler
  * */
void vsmc_sampler_read_resampled_history (vsmcSampler sampler, int *first);

/**
  * \brief sampler.sampler_ptr->move_num(iter)
  * \ingroup CSampler
  * */
unsigned vsmc_sampler_move_num (vsmcSampler sampler, unsigned iter);

/**
  * \brief sampler.sampler_ptr->accept_history(id, iter)
  * \ingroup CSampler
  * */
unsigned vsmc_sampler_accept_history (vsmcSampler sampler,
        unsigned id, unsigned iter);

/**
  * \brief sampler.sampler_ptr->particle()
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  *
  * \return A vsmcParticle object, say `particle`. `particle.particle_ptr`
  * points to the `vsmc::Particle<T>` object returned by the C++ call.
  * `particle.base_type` is `sampler.base_type`.
  * */
vsmcParticle vsmc_sampler_particle (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->init(new_init)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param new_init A vsmcInitialize object created by vsmc_initialize_new. It
  * will be C++ object pointed by `new_init.initialize_ptr` passed to the
  * underlying C++ call. After the function call, it is safe to delete the
  * vsmcInitialize object by vsmc_initialize_delete(new_init).
  * */
void vsmc_sampler_init (vsmcSampler sampler, vsmcInitialize new_init);

/**
  * \brief sampler.sampler_ptr->move(new_move)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param new_move A vsmcMove object created by vsmc_move_new. It will be C++
  * object pointed by `new_move.move_ptr` passed to the underlying C++ call.
  * After the function call, it is safe to delete the vsmcMove object by
  * vsmc_move_delete(new_move).
  * */
void vsmc_sampler_move (vsmcSampler sampler, vsmcMove new_move);

/**
  * \brief sampler.sampler_ptr->move_queue().clear()
  * \ingroup CSampler
  * */
void vsmc_sampler_move_queue_clear (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->move_queue().push_back(move)
  * \ingroup CSampler
  * */
void vsmc_sampler_move_queue_push_back (vsmcSampler sampler, vsmcMove move);

/**
  * \brief sampler.sampler_ptr->mcmc(new_mcmc)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param new_mcmc A vsmcMove object created by vsmc_move_new. It will be C++
  * object pointed by `new_move.move_ptr` passed to the underlying C++ call.
  * After the function call, it is safe to delete the vsmcMove object by
  * vsmc_move_delete(new_mcmc).
  * */
void vsmc_sampler_mcmc (vsmcSampler sampler, vsmcMove new_mcmc);

/**
  * \brief sampler.sampler_ptr->mcmc_queue().clear()
  * \ingroup CSampler
  * */
void vsmc_sampler_mcmc_queue_clear (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->mcmc_queue().push_back(mcmc)
  * \ingroup CSampler
  * */
void vsmc_sampler_mcmc_queue_push_back (vsmcSampler sampler, vsmcMove mcmc);

/**
 * \brief sampler.sampler_ptr->initialize(param)
 * \ingroup CSampler
 * */
void vsmc_sampler_initialize (vsmcSampler sampler, void *param);

/**
  * \brief sampler.sampler_ptr->iterate(num)
  * \ingroup CSampler
  * */
void vsmc_sampler_iterate (vsmcSampler sampler, unsigned num);

/**
  * \brief Call vsmc::Sampler::monitor on the sampler
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param name The name of the monitor
  * \param dim The dimension the monitor
  * \param eval A vsmcMonitorEval object, created by vsmc_monitor_eval_new or
  * otherwise. If `eval.monitor_eval_ptr` is not a NULL pointer, then, call
  * `sampler.sampler_ptr->monitor(name, dim, *(eval.monitor_eval_ptr))`,
  * otherwise call `vsampler.sampler_ptr->monitor(name)->second`.
  *
  * \return A vsmcMonitor object of the (possiblely newly created) monitor
  * given by the name.
  *
  * \note It is possible that a runtime whill happen during this function call
  * if `name` does not name an existing montior, nor does
  * `eval.monitor_eval_ptr` point to a valid `vsmc::Monitor::eval_type`
  * object
  * */
vsmcMonitor vsmc_sampler_monitor (vsmcSampler sampler, const char *name,
        unsigned dim, vsmcMonitorEval eval);

/**
 * \brief Call vsmc::Sampler::clear_monitor on the sampler
 * \ingroup CSampler
 *
 * \param sampler A vsmcSampler object
 * \param name The name of the monitor to be cleared. If `name` is a NULL
 * pointer, then call `sampler.sampler_ptr->clear_monitor()`, otherwise call
 * `sampler.sampler_ptr->clear_monitor(name)`
 * */
void vsmc_sampler_clear_monitor (vsmcSampler sampler, const char *name);

/**
  * \brief sampler.sampler_ptr->path()
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  *
  * \return A vsmcPath object, say `path`. `path.path_ptr` points to the
  * `vsmc::Path<T>` object returned by the C++ call. `path.base_type` is
  * `sampler.base_type`.
  * */
vsmcPath vsmc_sampler_path (vsmcSampler sampler);

/**
  * \brief sampler.sampler_ptr->path_sampling(eval)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param eval A vsmcPathEval object, created by vsmc_path_eval_new or
  * otherwise. If `eval.path_eval_ptr` is not a NULL pointer, then, call
  * `sampler.sampler_ptr->path_sampling(*(eval.path_eval_ptr))`, otherwise
  * call `vsampler.sampler_ptr->path_sampling()`.
  *
  * \return The (possible zero) path sampling estimate
  * */
double vsmc_sampler_path_sampling (vsmcSampler sampler, vsmcPathEval eval);

/**
  * \brief sampler.sampler_ptr->print(file_stream)
  * \ingroup CSampler
  *
  * \param sampler A vsmcSampler object
  * \param file_name The name of the output file. A `std::ofstream` is opened
  * with this name and passed to the underlying C++ call.
  * \param precision The precision of the ouptut, set by call
  * `file_stream.precison(precision)`.
  * */
void vsmc_sampler_print (vsmcSampler sampler, const char *file_name,
        unsigned precision);

/**
  * \brief sampler.sampler_ptr->show_progress(show)
  * \ingroup CSampler
  * */
void vsmc_sampler_show_progress (vsmcSampler sampler, int show);

/****************************************************************************/

size_t    vsmc_particle_size             (vsmcParticle);
vsmcValue vsmc_particle_value            (vsmcParticle);
void      vsmc_particle_resample         (vsmcParticle, double);
int       vsmc_particle_resampled        (vsmcParticle);
int       vsmc_particle_resample_scheme  (vsmcParticle, int);
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

/****************************************************************************/

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

/****************************************************************************/

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

/****************************************************************************/

vsmcInitialize vsmc_init_new (vsmcSampler,
        unsigned (*) (size_t, size_t, unsigned, double *),
        void (*) (vsmcParticle, void *),
        void (*) (vsmcParticle),
        void (*) (vsmcParticle));
void vsmc_init_delete (vsmcInitialize);

/****************************************************************************/

vsmcMove vsmc_move_new (vsmcSampler,
        unsigned (*) (size_t, size_t, unsigned, unsigned, double *),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_move_delete (vsmcMove);

/****************************************************************************/

vsmcMonitorEval vsmc_monitor_eval_new (vsmcSampler,
        void (*) (size_t, size_t, unsigned, unsigned, unsigned,
            const double *, double *),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_monitor_eval_delete (vsmcMonitorEval eval);

/****************************************************************************/

vsmcPathEval vsmc_path_eval_new (vsmcSampler,
        double (*) (size_t, size_t, unsigned, unsigned, const double *),
        double (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle),
        void (*) (unsigned, vsmcParticle));
void vsmc_path_eval_delete (vsmcPathEval);

/****************************************************************************/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* VSMC_H */
