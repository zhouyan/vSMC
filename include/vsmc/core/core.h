//============================================================================
// vSMC/include/vsmc/core/core.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CORE_CORE_H
#define VSMC_CORE_CORE_H

#include <vsmc/internal/common.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_Core_StateMatrix
/// @{

/// \brief `vsmc::StateMatrix::StateMatrix`
vsmc_state_matrix vsmc_state_matrix_new(int n, int dim);

/// \brief `vsmc::StateMatrix::~StateMatrix`
void vsmc_state_matrix_delete(vsmc_state_matrix *state_matrix_ptr);

/// \brief `vsmc::StateMatrix::operator=`
void vsm_state_matrix_assign(
    vsmc_state_matrix state_matrix, vsmc_state_matrix other);

/// \brief `vsmc::StateMatrix::dim`
int vsmc_state_matrix_dim(vsmc_state_matrix state_matrix);

/// \brief `vsmc::StateMatrix::resize`
void vsmc_state_matrix_resize(vsmc_state_matrix state_matrix, int n, int dim);

/// \brief `vsmc::StateMatrix::resize_dim`
void vsmc_state_matrix_resize_dim(vsmc_state_matrix state_matrix, int n);

/// \brief `vsmc::StateMatrix::size`
int vsmc_state_matrix_size(vsmc_state_matrix state_matrix);

/// \brief `vsmc::StateMatrix::reserve`
void vsmc_state_matrix_reserve(vsmc_state_matrix state_matrix, int N, int dim);

/// \brief `vsmc::StateMatrix::shrink_to_fit`
void vsmc_state_matrix_shrink_to_fit(vsmc_state_matrix state_matrix);

/// \brief `vsmc::StateMatrix::state`
double vsmc_state_matrix_get(vsmc_state_matrix state_matrix, int id, int pos);

/// \brief `vsmc::StateMatrix::state`
void vsmc_state_matrix_set(
    vsmc_state_matrix state_matrix, int id, int pos, double s);

/// \brief `vsmc::StateMatrix::data`
double *vsmc_state_matrix_data(vsmc_state_matrix state_matrix);

/// \brief `vsmc::StateMatrix::row_data`
double *vsmc_state_matrix_row_data(vsmc_state_matrix state_matrix, int id);

/// \brief `vsmc::StateMatrix::read_state`
double *vsmc_state_matrix_read_state(
    vsmc_state_matrix state_matrix, int pos, double *first);

/// \brief `vsmc::StateMatrix::read_state_matrix`
double *vsmc_state_matrix_read_state_matrix(
    vsmc_state_matrix state_matrix, vSMCMatrixLayout layout, double *first);

/// \brief `vsmc::StateMatrix::copy`
void vsmc_state_matrix_copy(
    vsmc_state_matrix state_matrix, int N, const int *index);

/// \brief `vsmc::StateMatrix::copy_particle`
void vsmc_state_matrix_copy_particle(
    vsmc_state_matrix state_matrix, int src, int dst);

/// @} C_API_Core_StateMatrix

/// \addtogroup C_API_Core_Weight
/// @{

/// \brief `vsmc::Weight::Weight`
vsmc_weight vsmc_weight_new(int n);

/// \brief `vsmc::Weight::~Weight`
void vsmc_weight_delete(vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::operator=`
void vsmc_weight_assign(vsmc_weight weight, vsmc_weight other);

/// \brief `vsmc::Weight::size`
int vsmc_weight_size(vsmc_weight weight);

/// \brief `vsmc::Weight::resize`
void vsmc_weight_resize(vsmc_weight weight, int N);

/// \brief `vsmc::Weight::reserve`
void vsmc_weight_reserve(vsmc_weight weight, int N);

/// \brief `vsmc::Weight::shrink_to_fit`
void vsmc_weight_shrink_to_fit(vsmc_weight weight);

/// \brief `vsmc::Weight::ess`
double vsmc_weight_ess(vsmc_weight weight);

/// \brief `vsmc::Weight::data`
const double *vsmc_weight_data(vsmc_weight weight);

/// \brief `vsmc::Weight::equal`
void vsmc_weight_set_equal(vsmc_weight weight);

/// \brief `vsmc::Weight::set`
void vsmc_weight_set(vsmc_weight weight, const double *first, int stride);

/// \brief `vsmc::Weight::mul`
void vsmc_weight_mul(vsmc_weight weight, const double *first, int stride);

/// \brief `vsmc::Weight::set_log`
void vsmc_weight_set_log(vsmc_weight weight, const double *first, int stride);

/// \brief `vsmc::Weight::log`
void vsmc_weight_add_log(vsmc_weight weight, const double *first, int stride);

/// \brief `vsmc::Weight::draw`
int vsmc_weight_draw(vsmc_weight weight, vsmc_rng rng);

/// @} C_API_Core_Weight

/// \addtogroup C_API_Core_Particle
/// @{

/// \brief `vsmc::Particle::Particle`
vsmc_particle vsmc_particle_new(int n, int dim);

/// \brief `vsmc::Particle::~Particle`
void vsmc_particle_delete(vsmc_particle *particle_ptr);

/// \brief `vsmc::Particle::operator=`
void vsmc_particle_assign(vsmc_particle particle, vsmc_particle other);

/// \brief `vsmc::Particle::clone`
void vsmc_particle_clone(
    vsmc_particle particle, vsmc_particle other, int retain_rng);

/// \brief `vsmc::Particle::size`
int vsmc_particle_size(vsmc_particle particle);

/// \brief `vsmc::Particle::resize_by_index`
void vsmc_particle_resize_by_index(vsmc_particle particle, int N, int *index);

/// \brief `vsmc::Particle::resize_by_mask`
void vsmc_particle_resize_by_mask(vsmc_particle particle, int N, int *mask);

/// \brief `vsmc::Particle::resize_by_resample`
void vsmc_particle_resize_by_resample(
    vsmc_particle particle, int N, vsmc_resample_type op);

/// \brief `vsmc::Particle::resize_by_uniform`
void vsmc_particle_resize_by_uniform(vsmc_particle particle, int N);

/// \brief `vsmc::Particle::resize_by_range`
void vsmc_particle_resize_by_ragne(
    vsmc_particle particle, int N, int first, int last);

/// \brief `vsmc:Particle::value`
vsmc_state_matrix vsmc_particle_value(vsmc_particle particle);

/// \brief `vsmc:Particle::weight`
vsmc_weight vsmc_particle_weight(vsmc_particle particle);

/// \brief `vsmc::Particle::rng`
vsmc_rng vsmc_particle_rng(vsmc_particle particle, int id);

/// \brief `vsmc::Particle::sp`
vsmc_single_particle vsmc_particle_sp(vsmc_particle particle, int id);

/// @} C_API_Core_Particle

/// \addtogroup C_API_Core_Monitor
/// @{

/// \brief `vsmc::Monitor::Monitor`
vsmc_monitor vsmc_monitor_new(int dim, vsmc_monitor_eval_type eval,
    int record_only, vSMCMonitorStage stage);

/// \brief `vsmc::Monitor::~Monitor`
void vsmc_monitor_delete(vsmc_monitor *monitor_ptr);

/// \brief `vsmc::Monitor::operator=`
void vsmc_monitor_assign(vsmc_monitor monitor, vsmc_monitor other);

/// \brief `vsmc::Monitor::dim`
int vsmc_monitor_dim(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::record_only`
int vsmc_monitor_record_only(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::stage`
vSMCMonitorStage vsmc_monitor_stage(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::iter_size`
int vsmc_monitor_iter_size(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::reserve`
void vsmc_monitor_reserve(vsmc_monitor monitor, int num);

/// \brief `vsmc::Monitor::empty`
int vsmc_monitor_empty(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::set_name`
void vsmc_monitor_set_name(vsmc_monitor monitor, int id, const char *name);

/// \brief `vsmc::Monitor::get_name`
int vsmc_monitor_get_name(vsmc_monitor monitor, int id, char *name);

/// \brief `vsmc::Monitor::index`
int vsmc_monitor_index(vsmc_monitor monitor, int iter);

/// \brief `vsmc::Monitor::read_index`
int *vsmc_monitor_read_index(vsmc_monitor monitor, int *fist);

/// \brief `vsmc::Monitor::record`
double vsmc_monitor_record(vsmc_monitor monitor, int id, int iter);

/// \brief `vsmc::Monitor::read_record`
double *vsmc_monitor_read_record(vsmc_monitor monitor, int id, double *first);

/// \brief `vsmc::Monitor::read_record_matrix`
double *vsmc_monitor_read_record_matrix(
    vsmc_monitor monitor, vSMCMatrixLayout layout, double *first);

/// \brief `vsmc::Monitor::set_eval`
void vsmc_monitor_set_eval(
    vsmc_monitor monitor, vsmc_monitor_eval_type new_eval);

/// \brief `vsmc::Monitor::eval`
void vsmc_monitor_eval(vsmc_monitor monitor, int iter, vsmc_particle particle,
    vSMCMonitorStage stage);

/// \brief `vsmc::Monitor::clear`
void vsmc_monitor_clear(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::recording`
int vsmc_monitor_recording(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::turn_on`
void vsmc_monitor_turn_on(vsmc_monitor monitor);

/// \brief `vsmc::Monitor::turn_off`
void vsmc_monitor_turn_off(vsmc_monitor monitor);

/// @} C_API_Core_Monitor

/// \addtogroup C_API_Core_Sampler
/// @{

/// \brief `vsmc::Sampler::Sampler`
vsmc_sampler vsmc_sampler_new(
    int n, int dim, vSMCResampleScheme scheme, double threshold);

/// \brief `vsmc::Sampler::~Sampler`
void vsmc_sampler_delete(vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::operator=`
void vsmc_sampler_assign(vsmc_sampler sampler, vsmc_sampler other);

/// \brief `vsmc::Sampler::clone`
void vsmc_sampler_clone(
    vsmc_sampler sampler, vsmc_sampler other, int retain_rng);

/// \brief `vsmc::Sampler::size`
int vsmc_sampler_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::reserve`
void vsmc_sampler_reserve(vsmc_sampler sampler, int num);

/// \brief `vsmc::Sampler::iter_size`
int vsmc_sampler_iter_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::iter_num`
int vsmc_sampler_iter_num(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::status_size`
int vsmc_sampler_status_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::resample`
void vsmc_sampler_resample(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::resample_scheme`
void vsmc_sampler_resample_scheme(
    vsmc_sampler sampler, vSMCResampleScheme scheme);

/// \brief `vsmc::Sampler::resample_method`
void vsmc_sampler_resample_algorithm(
    vsmc_sampler sampler, vsmc_resample_type res_alg);

/// \brief `vsmc::Sampler::resample_method`
void vsmc_sampler_resample_move(
    vsmc_sampler sampler, vsmc_sampler_move_type res_move);

/// \brief `vsmc::Sampler::threshold`
double vsmc_sampler_get_threshold(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::threshold`
void vsmc_sampler_set_threshold(vsmc_sampler sampler, double threshold);

/// \brief `vsmc::Sampler::threshold_never`
double vsmc_sampler_resample_threshold_never(void);

/// \brief `vsmc::Sampler::threshold_always`
double vsmc_sampler_resample_threshold_always(void);

/// \brief `vsmc::Sampler::size_history`
int vsmc_sampler_size_history(vsmc_sampler sampler, int iter);

/// \brief `vsmc::Sampler::read_size_history`
int *vsmc_sampler_read_size_history(vsmc_sampler sampler, int *first);

/// \brief `vsmc::Sampler::ess_history`
double vsmc_sampler_ess_history(vsmc_sampler sampler, int iter);

/// \brief `vsmc::Sampler::read_ess_history`
double *vsmc_sampler_read_ess_history(vsmc_sampler sampler, double *first);

/// \brief `vsmc::Sampler::resampled_history`
int vsmc_sampler_resampled_history(vsmc_sampler sampler, int iter);

/// \brief `vsmc::Sampler::read_resampled_history`
int *vsmc_sampler_read_resampled_history(vsmc_sampler sampler, int *first);

/// \brief `vsmc::Sampler::status_history`
int vsmc_sampler_status_history(vsmc_sampler sampler, int id, int iter);

/// \brief `vsmc::Sampler::read_status_history`
int *vsmc_sampler_read_status_history(vsmc_sampler, int id, int *first);

/// \brief `vsmc::Sampler::read_status_history_matrix`
int *vsmc_sampler_read_status_history_matrix(
    vsmc_sampler sampler, vSMCMatrixLayout layout, int *first);

/// \brief `vsmc::Sampler::particle`
vsmc_particle vsmc_sampler_particle(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::init_by_iter`
void vsmc_sampler_init_by_iter(
    vsmc_sampler sampler, int initialize_by_iterate);

/// \brief `vsmc::Sampler::init_queue_clear`
void vsmc_sampler_init_queue_clear(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::init_queue_empty`
int vsmc_sampler_init_queue_empty(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::init_queue_size`
int vsmc_sampler_init_queue_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::init`
void vsmc_sampler_init(
    vsmc_sampler sampler, vsmc_sampler_init_type new_init, int append);

/// \brief `vsmc::Sampler::move_queue_clear`
void vsmc_sampler_move_queue_clear(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::move_queue_empty`
int vsmc_sampler_move_queue_empty(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::move_queue_size`
int vsmc_sampler_move_queue_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::move`
void vsmc_sampler_move(
    vsmc_sampler sampler, vsmc_sampler_move_type new_move, int append);

/// \brief `vsmc::Sampler::mcmc_queue_clear`
void vsmc_sampler_mcmc_queue_clear(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::mcmc_queue_empty`
int vsmc_sampler_mcmc_queue_empty(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::mcmc_queue_size`
int vsmc_sampler_mcmc_queue_size(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::mcmc`
void vsmc_sampler_mcmc(
    vsmc_sampler sampler, vsmc_sampler_move_type new_mcmc, int append);

/// \brief `vsmc::Sampler::initialize`
void vsmc_sampler_initialize(vsmc_sampler sampler, void *param);

/// \brief `vsmc::Sampler::iterate`
void vsmc_sampler_iterate(vsmc_sampler sampler, int num);

/// \brief `vsmc::Sampler::monitor`
void vsmc_sampler_set_monitor_direct(
    vsmc_sampler sampler, const char *name, vsmc_monitor mon);

/// \brief `vsmc::Sampler::monitor`
void vsmc_sampler_set_monitor(vsmc_sampler sampler, const char *name, int dim,
    vsmc_monitor_eval_type eval, int record_only, vSMCMonitorStage stage);

/// \brief `vsmc::Sampler::monitor`
vsmc_monitor vsmc_sampler_get_monitor(vsmc_sampler sampler, const char *name);

/// \brief `vsmc::Sampler::clear_monitor`
void vsmc_sampler_clear_monitor(vsmc_sampler sampler, const char *name);

/// \brief `vsmc::Sampler::clear_monitor`
void vsmc_sampler_clear_monitor_all(vsmc_sampler sampler);

/// \brief `vsmc::Sampler::print`
size_t vsmc_sampler_print(vsmc_sampler sampler, char *buf, char sepchar);

/// \brief `vsmc::Sampler::print` directly to an external file
void vsmc_sampler_print_f(
    vsmc_sampler sampler, const char *filename, char sepchar);

/// @} C_API_Core_Sampler

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_CORE_CORE_H
