/*============================================================================
 * vSMC/include/vsmc/vsmc.h
 *----------------------------------------------------------------------------
 *                         vSMC: Scalable Monte Carlo
 *----------------------------------------------------------------------------
 * Copyright (c) 2013-2016, Yan Zhou
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *==========================================================================*/

#ifndef VSMC_VSMC_H
#define VSMC_VSMC_H

#include <vsmc/rngc/rngc.h>
#include <stdalign.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup C_API C API
/// @{

/// \defgroup C_API_Definitions Enumerators, placeholders and macros
/// @{

/// \brief `vsmc::MatrixLayout`
typedef enum { vSMCRowMajor = 101, vSMCColMajor = 102 } vSMCMatrixLayout;

/// \brief `vsmc::ResampleScheme`
typedef enum {
    vSMCMultinomial,
    vSMCStratified,
    vSMCSystematic,
    vSMCResidual,
    vSMCResidualStratified,
    vSMCResidualSystematic
} vSMCResampleScheme;

/// \brief `vsmc::MonitorStage`
typedef enum {
    vSMCMonitorMove,
    vSMCMonitorResample,
    vSMCMonitorMCMC
} vSMCMonitorStage;

/// @}

/// \defgroup C_API_RNG Random number generating
/// @{

/// \defgroup C_API_RNG_Seed `vsmc::Seed`
/// @{

/// \brief `vsmc::Seed::get`
int vsmc_seed_get();

/// \brief `vsmc::Seed::set`
void vsmc_seed_set(int seed);

/// \brief `vsmc::Seed::modulo`
void vsmc_seed_modulo(int div, int rem);

/// \brief `vsmc::Seed::skip`
void vsmc_seed_skip(int steps);

/// \brief `vsmc::Seed::operator<<`
///
/// \details
/// If `mem == nullptr`, return the number of bytes required in `mem`.
/// Otherwise, store `vsmc::Seed::instacne()` in `mem`
int vsmc_seed_save(void *mem);

/// \brief `vsmc::Seed::operator>>`
///
/// \details
/// `mem` points to memory previously written by `vsmc_seed_save`
void vsmc_seed_load(const void *mem);

/// \brief `vsmc::Seed::operator<<` direct to external file
void vsmc_seed_save_f(const char *filename);

/// \brief `vsmc::Seed::operator>>` direct from external file
void vsmc_seed_load_f(const char *filename);

/// @} C_API_RNG_Seed

/// \defgroup C_API_RNG_RNG vsmc::RNG
/// @{

/// \brief `vsmc::RNG`
typedef struct {
    void *ptr;
} vsmc_rng;

/// \brief `vsmc::RNG::RNG`
void vsmc_rng_malloc(vsmc_rng *rng_ptr, int seed);

/// \brief `vsmc::RNG::~RNG`
void vsmc_rng_free(vsmc_rng *rng_ptr);

/// \brief `vsmc::RNG::seed`
void vsmc_rng_seed(vsmc_rng *rng_ptr, int seed);

/// \brief `vsmc::RNG::seed`
void vsmc_rng_seed(vsmc_rng *rng_ptr, int seed);

/// \brief `vsmc::RNG::key`
void vsmc_rng_get_key(const vsmc_rng *rng_ptr, int n, int *key);

/// \brief `vsmc::RNG::key`
void vsmc_rng_set_key(vsmc_rng *rng_ptr, int n, const int *key);

/// \brief `vsmc::RNG::ctr`
void vsmc_rng_get_ctr(const vsmc_rng *rng_ptr, int n, int *ctr);

/// \brief `vsmc::RNG::ctr`
void vsmc_rng_set_ctr(vsmc_rng *rng_ptr, int n, const int *ctr);

/// \brief `vsmc::rng_rand`
void vsmc_rng_rand(vsmc_rng *rng_ptr, int n, int *r);

/// @} C_API_RNG_RNG

/// \defgroup C_API_RNG_U01Sequence U01 Sequence
/// @{

/// \brief `vsmc::u01_sorted`
void vsmc_u01_sorted(int n, const double *u01, double *u01seq);

/// \brief `vsmc::u01_stratifed`
void vsmc_u01_stratified(int n, const double *u01, double *u01seq);

/// \brief `vsmc::u01_systematic`
void vsmc_u01_systematic(int n, double u01, double *u01seq);

/// @} C_API_RNG_U01Sequence

/// \defgroup C_API_RNG_DISTRIBUITON Distributions
/// @{

/// \brief `vsmc::DiscreteDistribution<int>`
void vsmc_discrete_distribution(vsmc_rng *rng_ptr, int n, int *r, int m,
    const double *weight, int normalized);

/// \brief `std::uniform_int_distribution<int>`
void vsmc_uniform_int_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int a, int b);

/// \brief `std::bernoulli_distribution`
void vsmc_bernoulli_distribution(vsmc_rng *rng_ptr, int n, int *r, double p);

/// \brief `std::binomial_distribution<int>`
void vsmc_binomial_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int t, double p);

/// \brief `std::negative_binomial_distribution<int>`
void vsmc_negative_binomial_distribution(
    vsmc_rng *rng_ptr, int n, int *r, int k, double p);

/// \brief `std::geometri_distribution<int>`
void vsmc_geometric_distribution(vsmc_rng *rng_ptr, int n, int *r, double p);

/// \brief `std::poisson_distribution<int>`
void vsmc_poisson_distribution(vsmc_rng *rng_ptr, int n, int *r, double mean);

/// \brief `vsmc::BetaDistribution<double>`
void vsmc_beta_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double alpha, double beta);

/// \brief `vsmc::CachyDistribution<double>`
void vsmc_cauchy_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::ChiSquaredDistribution<double>`
void vsmc_chi_squared_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double df);

/// \brief `vsmc::ExponentialDistribution<double>`
void vsmc_exponential_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double lambda);

/// \brief `vsmc::ExtremeValueDistribution<double>`
void vsmc_extreme_value_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::FisherFDistribution<double>`
void vsmc_fisher_f_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double df1, double df2);

/// \brief `vsmc::GammaDistribution<double>`
void vsmc_gamma_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double alpha, double beta);

/// \brief `vsmc::LaplaceDistribution<double>`
void vsmc_laplace_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::LevyDistribution<double>`
void vsmc_levy_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::LogisticDistribution<double>`
void vsmc_logistic_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::LognormalDistribution<double>`
void vsmc_lognormal_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double m, double s);

/// \brief `vsmc::NormalDistribution<double>`
void vsmc_normal_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double mean, double stddev);

/// \brief `vsmc::NormalMVDistribution<double, vsmc::Dynamic>`
void vsmc_normal_mv_distribution(vsmc_rng *rng_ptr, int n, double *r, int dim,
    const double *mean, const double *chol);

/// \brief `vsmc::ParetoDistribution<double>`
void vsmc_pareto_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::RayleighDistribution<double>`
void vsmc_rayleigh_distribution(vsmc_rng *rng_ptr, int n, double *r, double b);

/// \brief `vsmc::StudentTDistribution<double>`
void vsmc_student_t_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double df);

/// \brief `vsmc::U01Distribution<double>`
void vsmc_u01_distribution(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01DistributionCC<double>`
void vsmc_u01_cc_distribution(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01DistributionCO<double>`
void vsmc_u01_co_distribution(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01DistributionOC<double>`
void vsmc_u01_oc_distribution(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01DistributionOO<double>`
void vsmc_u01_oc_distribution(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::UniformRealDistribution<double>`
void vsmc_uniform_real_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::WeibullDistribution<double>`
void vsmc_weibull_distribution(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// @} C_API_RNG_DISTRIBUITON

/// \defgroup C_API_RNG_MLK_BRNG Register MKL BRNG (`vsmc::mkl_brng`)
/// @{

/// \brief `vsmc::mkl_brng<std::mt19937>`
int vsmc_mkl_brng_mt19937(void);

/// \brief `vsmc::mkl_brng<std::mt19937_64>`
int vsmc_mkl_brng_mt19937_64(void);

/// \brief `vsmc::mkl_brng<std::minstd_rand0>`
int vsmc_mkl_brng_minstd_rand0(void);

/// \brief `vsmc::mkl_brng<std::minstd_rand>`
int vsmc_mkl_brng_minstd_rand(void);

/// \brief `vsmc::mkl_brng<std::ranlux24_base>`
int vsmc_mkl_brng_ranlux24_base(void);

/// \brief `vsmc::mkl_brng<std::ranlux48_base>`
int vsmc_mkl_brng_ranlux48_base(void);

/// \brief `vsmc::mkl_brng<std::ranlux24>`
int vsmc_mkl_brng_ranlux24(void);

/// \brief `vsmc::mkl_brng<std::ranlux48>`
int vsmc_mkl_brng_ranlux48(void);

/// \brief `vsmc::mkl_brng<std::knuth_b>`
int vsmc_mkl_brng_knuth_b(void);

/// \brief `vsmc::mkl_brng<vsmc::Philox2x32>`
int vsmc_mkl_brng_philox2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::Philox4x32>`
int vsmc_mkl_brng_philox4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::Philox2x64>`
int vsmc_mkl_brng_philox2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::Philox4x64>`
int vsmc_mkl_brng_philox4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x32>`
int vsmc_mkl_brng_threefry2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x32>`
int vsmc_mkl_brng_threefry4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x64>`
int vsmc_mkl_brng_threefry2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x64>`
int vsmc_mkl_brng_threefry4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x32SSE2>`
int vsmc_mkl_brng_threefry2x32sse2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x32SSE2>`
int vsmc_mkl_brng_threefry4x32sse2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x64SSE2>`
int vsmc_mkl_brng_threefry2x64sse2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x64SSE2>`
int vsmc_mkl_brng_threefry4x64sse2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x32AVX2>`
int vsmc_mkl_brng_threefry2x32avx2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x32AVX2>`
int vsmc_mkl_brng_threefry4x32avx2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry2x64AVX2>`
int vsmc_mkl_brng_threefry2x64avx2(void);

/// \brief `vsmc::mkl_brng<vsmc::Threefry4x64AVX2>`
int vsmc_mkl_brng_threefry4x64avx2(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_1x32>`
int vsmc_mkl_brng_aes128_1x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_2x32>`
int vsmc_mkl_brng_aes128_2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_4x32>`
int vsmc_mkl_brng_aes128_4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_8x32>`
int vsmc_mkl_brng_aes128_8x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_1x64>`
int vsmc_mkl_brng_aes128_1x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_2x64>`
int vsmc_mkl_brng_aes128_2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_4x64>`
int vsmc_mkl_brng_aes128_4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES128_8x64>`
int vsmc_mkl_brng_aes128_8x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_1x32>`
int vsmc_mkl_brng_aes192_1x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_2x32>`
int vsmc_mkl_brng_aes192_2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_4x32>`
int vsmc_mkl_brng_aes192_4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_8x32>`
int vsmc_mkl_brng_aes192_8x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_1x64>`
int vsmc_mkl_brng_aes192_1x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_2x64>`
int vsmc_mkl_brng_aes192_2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_4x64>`
int vsmc_mkl_brng_aes192_4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES192_8x64>`
int vsmc_mkl_brng_aes192_8x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_1x32>`
int vsmc_mkl_brng_aes256_1x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_2x32>`
int vsmc_mkl_brng_aes256_2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_4x32>`
int vsmc_mkl_brng_aes256_4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_8x32>`
int vsmc_mkl_brng_aes256_8x32(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_1x64>`
int vsmc_mkl_brng_aes256_1x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_2x64>`
int vsmc_mkl_brng_aes256_2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_4x64>`
int vsmc_mkl_brng_aes256_4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::AES256_8x64>`
int vsmc_mkl_brng_aes256_8x64(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_1x32>`
int vsmc_mkl_brng_ars_1x32(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_2x32>`
int vsmc_mkl_brng_ars_2x32(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_4x32>`
int vsmc_mkl_brng_ars_4x32(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_8x32>`
int vsmc_mkl_brng_ars_8x32(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_1x64>`
int vsmc_mkl_brng_ars_1x64(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_2x64>`
int vsmc_mkl_brng_ars_2x64(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_4x64>`
int vsmc_mkl_brng_ars_4x64(void);

/// \brief `vsmc::mkl_brng<vsmc::ARS_8x64>`
int vsmc_mkl_brng_ars_8x64(void);

/// \brief `vsmc::mkl_brng<vsmc::RDRAND16>`
int vsmc_mkl_brng_rdrand16(void);

/// \brief `vsmc::mkl_brng<vsmc::RDRAND32>`
int vsmc_mkl_brng_rdrand32(void);

/// \brief `vsmc::mkl_brng<vsmc::RDRAND64>`
int vsmc_mkl_brng_rdrand64(void);

/// @} C_API_RNG_MLK_BRNG

/// \defgroup C_API_RNG_RandomWalk Random walk
/// @{

/// \brief `vsmc::RandomWalk<double, vsmc::Dynamic>`
int vsmc_random_walk(vsmc_rng *rng_ptr, int dim, double *x, double *ltx,
    double (*log_target)(int, const double *),
    double (*proposal)(vsmc_rng *, int, const double *, double *));

/// \brief `vsmc::RandomWalkG<double, vsmc::Dynamic, vsmc::Dynamic>`
int vsmc_random_walk_g(vsmc_rng *rng_ptr, int dim_x, int dim_g, double *x,
    double *ltx, double *g,
    double (*log_target)(int, int, const double *, double *),
    double (*proposal)(vsmc_rng *, int, const double *, double *));

/// \brief `vsmc::NormalProposal<double>`
double vsmc_normal_proposal(vsmc_rng *rng_ptr, int, const double *x, double *y,
    double stddev, double a, double b);

/// \brief `vsmc::NormalMVProposal<double, vsmc::Dynamic>`
double vsmc_normal_mv_proposal(vsmc_rng *rng_ptr, int dim, const double *x,
    double *y, const double *chol, const double *a, const double *b);

/// @} C_API_RNG_RandomWalk

/// @} C_API_RNG

/// \defgroup C_API_Core Core
/// @{

/// \defgroup C_API_Core_StateMatrix  `vsmc::StateMatrix`
/// @{

/// \brief `vsmc::StateMatrix<vsmc::RowMajor, vsmc::Dynamic, double>`
typedef struct {
    void *ptr;
} vsmc_state_matrix;

/// \brief `vsmc::StateMatrix::StateMatrix`
void vsmc_state_matrix_malloc(
    vsmc_state_matrix *state_matrix_ptr, int n, int dim);

/// \brief `vsmc::StateMatrix::~StateMatrix`
void vsmc_state_matrix_free(vsmc_state_matrix *state_matrix_ptr);

/// \brief `vsmc::StateMatrix::operator=`
void vsm_state_matrix_assign(
    vsmc_state_matrix *dst, const vsmc_state_matrix *src);

/// \brief `vsmc::StateMatrix::dim`
int vsmc_state_matrix_dim(const vsmc_state_matrix *state_matrix_ptr);

/// \brief `vsmc::StateMatrix::resize_dim`
void vsmc_state_matrix_resize_dim(vsmc_state_matrix *state_matrix_ptr, int n);

/// \brief `vsmc::StateMatrix::size`
int vsmc_state_matrix_size(const vsmc_state_matrix *state_matrix_ptr);

/// \brief `vsmc::StateMatrix::state`
double vsmc_state_matrix_get(
    const vsmc_state_matrix *state_matrix_ptr, int id, int pos);

/// \brief `vsmc::StateMatrix::state`
void vsmc_state_matrix_set(
    vsmc_state_matrix *state_matrix_ptr, int id, int pos, double s);

/// \brief `vsmc::StateMatrix::data`
double *vsmc_state_matrix_data(vsmc_state_matrix *state_matrix_ptr);

/// \brief `vsmc::StateMatrix::row_data`
double *vsmc_state_matrix_row_data(
    vsmc_state_matrix *state_matrix_ptr, int id);

/// \brief `vsmc::StateMatrix::copy`
void vsmc_state_matrix_copy(
    vsmc_state_matrix *state_matrix_ptr, int N, const int *index);

/// \brief `vsmc::StateMatrix::copy_particle`
void vsmc_state_matrix_copy_particle(
    vsmc_state_matrix *state_matrix_ptr, int src, int dst);

/// @} C_API_Core_StateMatrix

/// \defgroup C_API_Core_Weight  `vsmc::Weight`
/// @{

/// \brief `vsmc::Weight`
typedef struct {
    void *ptr;
} vsmc_weight;

/// \brief `vsmc::Weight::Weight`
void vsmc_weight_malloc(vsmc_weight *weight_ptr, int n);

/// \brief `vsmc::Weight::~Weight`
void vsmc_weight_free(vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::operator=`
void vsmc_weight_assign(vsmc_weight *dst, const vsmc_weight *src);

/// \brief `vsmc::Weight::size`
int vsmc_weight_size(const vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::ess`
double vsmc_weight_ess(const vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::data`
const double *vsmc_weight_data(const vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::equal`
void vsmc_weight_set_equal(vsmc_weight *weight_ptr);

/// \brief `vsmc::Weight::set`
void vsmc_weight_set(vsmc_weight *weight_ptr, const double *first, int stride);

/// \brief `vsmc::Weight::mul`
void vsmc_weight_mul(vsmc_weight *weight_ptr, const double *first, int stride);

/// \brief `vsmc::Weight::set_log`
void vsmc_weight_set_log(
    vsmc_weight *weight_ptr, const double *first, int stride);

/// \brief `vsmc::Weight::log`
void vsmc_weight_add_log(
    vsmc_weight *weight_ptr, const double *first, int stride);

/// \brief `vsmc::Weight::draw`
int vsmc_weight_draw(vsmc_weight *weight_ptr, vsmc_rng *rng_ptr);

/// @} C_API_Core_Weight

/// \defgroup C_API_Core_SingleParticle
/// @{

/// \brief `vsmc::SingleParticle`
typedef struct {
    double *state;
    int id;
} vsmc_single_particle;

/// @} C_API_Core_SingleParticle

/// \defgroup C_API_Core_Particle  `vsmc::Particle`
/// @{

/// \brief `vsmc::Particle`
typedef struct {
    void *ptr;
    vsmc_state_matrix value;
    vsmc_weight weight;
} vsmc_particle;

/// \brief `vsmc::Particle::resample_type`
typedef void (*vsmc_particle_resample_type)(
    int, int, vsmc_rng *, const double *, int *);

/// \brief `vsmc::Particle::Particle`
void vsmc_particle_malloc(vsmc_particle *particle_ptr, int n, int dim);

/// \brief `vsmc::Particle::~Particle`
void vsmc_particle_free(vsmc_particle *particle_ptr);

/// \brief `vsmc::Particle::operator=`
void vsmc_particle_assign(vsmc_particle *dst, const vsmc_particle *src);

/// \brief `vsmc::Particle::clone`
void vsmc_particle_clone(
    vsmc_particle *particle_ptr, const vsmc_particle *other, int retain_rng);

/// \brief `vsmc::Particle::size`
int vsmc_particle_size(const vsmc_particle *particle_ptr);

/// \brief `vsmc::Particle::rng`
vsmc_rng vsmc_particle_rng(vsmc_particle *particle_ptr, int id);

/// \brief `vsmc::Particle::sp`
vsmc_single_particle vsmc_particle_sp(vsmc_particle *particle_ptr, int id);

/// \brief `vsmc::Particle::resample`
void vsmc_particle_resample(vsmc_particle *particle_ptr,
    vsmc_particle_resample_type op, double threshold);

/// @} C_API_Core_Particle

/// \defgroup C_API_Core_Monitor  `vsmc::Monitor`
/// @{

/// \brief `vsmc::Monitor`
typedef struct {
    void *ptr;
} vsmc_monitor;

/// \brief `vsmc::Monitor::eval_type`
typedef void (*vsmc_monitor_eval_type)(int, int, vsmc_particle *, double *);

/// @} C_API_Core_Monitor

/// \defgroup C_API_Core_Sampler  `vsmc::Sampler`
/// @{

/// \brief `vsmc::Sampler`
typedef struct {
    void *ptr;
    vsmc_particle particle;
} vsmc_sampler;

/// \brief `vsmc::Sampler::init_type`
typedef int (*vsmc_sampler_init_type)(vsmc_particle *, void *);

/// \brief `vsmc::Sampler::move_type`
typedef int (*vsmc_sampler_move_type)(int, vsmc_particle *);

/// \brief `vsmc::Sampler::mcmc_type`
typedef int (*vsmc_sampler_mcmc_type)(int, vsmc_particle *);

/// \brief `vsmc::Sampler::Sampler`
void vsmc_sampler_malloc(vsmc_sampler *sampler_ptr, int n, int dim,
    vSMCResampleScheme scheme, double threshold);

/// \brief `vsmc::Sampler::~Sampler`
void vsmc_sampler_free(vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::operator=`
void vsmc_sampler_assign(vsmc_sampler *dst, const vsmc_sampler *src);

/// \brief `vsmc::Sampler::clone`
void vsmc_sampler_clone(
    vsmc_sampler *sampler_ptr, const vsmc_sampler *other, int retain_rng);

/// \brief `vsmc::Sampler::size`
int vsmc_sampler_size(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::reserve`
void vsmc_sampler_reserve(vsmc_sampler *sampler_ptr, int num);

/// \brief `vsmc::Sampler::iter_size`
int vsmc_sampler_iter_size(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::iter_num`
int vsmc_sampler_iter_num(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::resample`
void vsmc_sampler_resample(vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::resample_scheme`
void vsmc_sampler_resample_op(
    vsmc_sampler *sampler_ptr, vsmc_particle_resample_type op);

/// \brief `vsmc::Sampler::resample_scheme`
void vsmc_sampler_resample_scheme(
    vsmc_sampler *sampler_ptr, vSMCResampleScheme scheme);

/// \brief `vsmc::Sampler::threshold`
double vsmc_sampler_get_threshold(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::threshold`
void vsmc_sampler_set_threshold(vsmc_sampler *sampler_ptr, double threshold);

/// \brief `vsmc::Sampler::threshold_never`
double vsmc_sampler_resample_threshold_never();

/// \brief `vsmc::Sampler::threshold_always`
double vsmc_sampler_resample_threshold_always();

/// \brief `vsmc::Sampler::init`
void vsmc_sampler_init(
    vsmc_sampler *sampler_ptr, vsmc_sampler_init_type new_init);

/// \brief `vsmc::Sampler::init_by_iter`
void vsmc_sampler_init_by_iter(
    vsmc_sampler *sampler_ptr, int initialize_by_iterate);

/// \brief `vsmc::Sampler::init_by_move`
void vsmc_sampler_init_by_move(
    vsmc_sampler *sampler_ptr, vsmc_sampler_move_type new_init);

/// \brief `vsmc::Sampler::move_queue_clear`
void vsmc_sampler_move_queue_clear(vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::move_queue_empty`
int vsmc_sampler_move_queue_empty(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::move`
void vsmc_smapler_move(
    vsmc_sampler *sampler_ptr, vsmc_sampler_move_type new_move, int append);

/// \brief `vsmc::Sampler::mcmc_queue_clear`
void vsmc_sampler_mcmc_queue_clear(vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::mcmc_queue_empty`
int vsmc_sampler_mcmc_queue_empty(const vsmc_sampler *sampler_ptr);

/// \brief `vsmc::Sampler::mcmc`
void vsmc_smapler_mcmc(
    vsmc_sampler *sampler_ptr, vsmc_sampler_mcmc_type new_mcmc, int append);

/// \brief `vsmc::Sampler::initialize`
void vsmc_sampler_initialize(vsmc_sampler *sampler_ptr, void *param);

/// \brief `vsmc::Sampler::iterate`
void vsmc_sampler_iterate(vsmc_sampler *sampler_ptr, int num);

/// \brief `vsmc::Sampler::monitor`
void vsmc_sampler_set_monitor_direct(
    vsmc_sampler *sampler_ptr, const char *name, const vsmc_monitor *mon);

/// \brief `vsmc::Sampler::monitor`
void vsmc_sampler_set_monitor(vsmc_sampler *sampler_ptr, const char *name,
    int dim, vsmc_monitor_eval_type eval, int record_only,
    vSMCMonitorStage stage);

/// \brief `vsmc::Sampler::monitor`
vsmc_monitor vsmc_sampler_get_monitor(
    vsmc_sampler *sampler_ptr, const char *name);

/// \brief `vsmc::Sampler::clear_monitor`
void vsmc_sampler_clear_monitor(vsmc_sampler *sampler_ptr, const char *name);

/// \brief `vsmc::Sampler::clear_monitor`
void vsmc_sampler_clear_monitor_all(vsmc_sampler *sampler_ptr);

/// \brief `operator<<(vsmc::Sampler)`
int vsmc_sampler_save(const vsmc_sampler *sampler_ptr, char *mem);

/// \brief `operator<<(vsmc::Sampler::Sampler)` direct to file
void vsmc_sampler_save_f(
    const vsmc_sampler *sampler_ptr, const char *filename);

/// @} C_API_Core_Sampler

/// @} C_API_Core

/// \defgroup C_API_Utility Utility
/// @{

/// \defgroup C_API_Utility_AlignedMemory Aligned memory allocation
/// @{

/// \brief `vsmc::AlignedMemory::aligned_malloc`
void *vsmc_malloc(size_t n, int alignment);

/// \brief `vsmc::AlignedMemory::aligned_free`
void vsmc_free(void *ptr);

/// @} C_API_Utility_AlignedMemory

/// @} C_API_Utility

/// \defgroup C_API_Resample Resample algorithms
/// @{

/// \brief `vsmc::resample_trans_u01_rep`
void vsmc_resample_trans_u01_rep(
    int m, int n, const double *weight, const double *u01, int *replication);

/// \brief `vsmc::resample_trans_u01_index`
void vsmc_resample_trans_u01_index(
    int m, int n, const double *weight, const double *u01, int *index);

/// \brief `vsmc::resample_trans_rep_index`
void vsmc_resample_trans_rep_index(
    int m, int n, const int *replication, int *index);

/// \brief `vsmc::resample_trans_index_rep`
void vsmc_resample_trans_index_rep(
    int m, int n, const int *index, int *replication);

/// \brief `vsmc::resample_trans_residual`
int vsmc_resample_trans_residual(
    int m, int n, const double *weight, double *resid, int *integ);

/// \brief `vsmc::ResampleMultinomial`
void vsmc_resample_multinomial(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// \brief `vsmc::ResampleStratified`
void vsmc_resample_stratified(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// \brief `vsmc::ResampleSystematic`
void vsmc_resample_systematic(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// \brief `vsmc::ResampleResidual`
void vsmc_resample_residual(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// \brief `vsmc::ResampleResidualStratified`
void vsmc_resample_residual_stratified(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// \brief `vsmc::ResampleSystematic`
void vsmc_resample_residual_systematic(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);

/// @} C_API_Resample

/// @} C_API

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_VSMC_H
