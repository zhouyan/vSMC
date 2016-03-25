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

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup C_API C API
 * \brief Interfacing with C and other languages
 *
 * \defgroup C_API_Definitions Enumerators, placeholders and macros
 * \ingroup C_API
 *
 * \defgroup C_API_Core Core
 * \ingroup C_API
 *
 * \defgroup C_API_Core_StateMatrix vsmc::StateMatrix
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Core_Weight vsmc::Weight
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Core_Particle vsmc::Particle
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Core_SingleParticle vsmc::SingleParticle
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Core_Monitor vsmc::Monitor
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Core_Sampler vsmc::Sampler
 * \ingroup C_API_Core
 *
 * \defgroup C_API_Resample Resample algorithms
 * \ingroup C_API
 *
 * \defgroup C_API_RNG Random number generating
 * \ingroup C_API
 *
 * \defgroup C_API_RNG_RNG vsmc::RNG
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_Seed vsmc::Seed
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_RNG vsmc::RNG
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_U01Sequence U01 sequence
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_DISTRIBUITON Distribution
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_MLK_BRNG Register MKL BRNG
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_RNG_RandomWalk Random walk
 * \ingroup C_API_RNG
 *
 * \defgroup C_API_SMP Symmetric multiprocessing
 * \ingroup C_API
 *
 * \defgroup C_API_SMP_SEQ Sequential
 * \ingroup C_API_SMP
 *
 * \defgroup C_API_SMP_OMP OpenMP
 * \ingroup C_API_SMP
 *
 * \defgroup C_API_SMP_TBB Intel Threading Building Blocks
 * \ingroup C_API_SMP
 *
 * \defgroup C_API_Utility Utility
 * \ingroup C_API
 *
 * \defgroup C_API_Utility_AlignedMemory Aligned memory allocation
 * \ingroup C_API_Utility
 *
 * \defgroup C_API_Utility_Covariance Covariance
 * \ingroup C_API_Utility
 *
 * \defgroup C_API_Utility_HDF5IO HDF5 objects IO
 * \ingroup C_API_Utility
 *
 * \defgroup C_API_Utility_ProgramOption Program option
 * \ingroup C_API_Utility
 *
 * \defgroup C_API_Utility_Progress Progress
 * \ingroup C_API_Utility
 *
 * \defgroup C_API_Utility_StopWatch Stop watch
 * \ingroup C_API_Utility
 *
 */

/** \addtogroup C_API_Definitions */
/** @{ */

/** \brief `vsmc::MatrixLayout` */
typedef enum { vSMCRowMajor = 101, vSMCColMajor = 102 } vSMCMatrixLayout;

/** \brief `vsmc::ResampleScheme` */
typedef enum {
    vSMCMultinomial,
    vSMCStratified,
    vSMCSystematic,
    vSMCResidual,
    vSMCResidualStratified,
    vSMCResidualSystematic
} vSMCResampleScheme;

/** \brief `vsmc::MonitorStage` */
typedef enum {
    vSMCMonitorMove,
    vSMCMonitorResample,
    vSMCMonitorMCMC
} vSMCMonitorStage;

/** @} */ /* C_API_Definitions */

/** \addtogroup C_API_RNG_Seed */
/** @{ */

/** \brief `vsmc::Seed::get` */
int vsmc_seed_get(void);

/** \brief `vsmc::Seed::set` */
void vsmc_seed_set(int seed);

/** \brief `vsmc::Seed::modulo` */
void vsmc_seed_modulo(int div, int rem);

/** \brief `vsmc::Seed::skip` */
void vsmc_seed_skip(int steps);

/** \brief `vsmc::Seed::operator<<`
 *
 * \details
 * If `mem == nullptr`, return the number of bytes required in `mem`.
 * Otherwise, store `vsmc::Seed::instacne()` in `mem` */
int vsmc_seed_save(void *mem);

/** \brief `vsmc::Seed::operator>>`
 *
 * \details
 * `mem` points to memory previously written by `vsmc_seed_save` */
void vsmc_seed_load(const void *mem);

/** \brief `vsmc::Seed::operator<<` direct to external file */
void vsmc_seed_save_f(const char *filename);

/** \brief `vsmc::Seed::operator>>` direct from external file */
void vsmc_seed_load_f(const char *filename);

/** @} */ /* C_API_RNG_Seed */

/** \addtogroup C_API_RNG_RNG */
/** @{ */

/** \brief `vsmc::RNG` */
typedef struct {
    void *ptr;
} vsmc_rng;

/** \brief `vsmc::RNG::RNG` */
vsmc_rng vsmc_rng_new(int seed);

/** \brief `vsmc::RNG::~RNG` */
void vsmc_rng_delete(vsmc_rng *rng_ptr);

/** \brief `vsmc::RNG::operator=` */
void vsmc_rng_assign(vsmc_rng rng, vsmc_rng other);

/** \brief `vsmc::RNG::seed` */
void vsmc_rng_seed(vsmc_rng rng, int seed);

/** \brief `vsmc::RNG::key` */
void vsmc_rng_get_key(vsmc_rng rng, int n, int *key);

/** \brief `vsmc::RNG::key` */
void vsmc_rng_set_key(vsmc_rng rng, int n, const int *key);

/** \brief `vsmc::RNG::ctr` */
void vsmc_rng_get_ctr(vsmc_rng rng, int n, int *ctr);

/** \brief `vsmc::RNG::ctr` */
void vsmc_rng_set_ctr(vsmc_rng rng, int n, const int *ctr);

/** \brief `vsmc::rng_rand` */
void vsmc_rng_rand(vsmc_rng rng, int n, int *r);

/** @} */ /* C_API_RNG_RNG */

/** \addtogroup C_API_RNG_U01Sequence */
/** @{ */

/** \brief `vsmc::u01_sorted` */
void vsmc_u01_sorted(int n, const double *u01, double *u01seq);

/** \brief `vsmc::u01_stratifed` */
void vsmc_u01_stratified(int n, const double *u01, double *u01seq);

/** \brief `vsmc::u01_systematic` */
void vsmc_u01_systematic(int n, double u01, double *u01seq);

/** @} */ /* C_API_RNG_U01Sequence */

/** \addtogroup C_API_RNG_DISTRIBUITON */
/** @{ */

/** \brief `vsmc::DiscreteDistribution<int>` */
void vsmc_discrete_distribution(
    vsmc_rng rng, int n, int *r, int m, const double *weight, int normalized);

/** \brief `std::uniform_int_distribution<int>` */
void vsmc_uniform_int_distribution(vsmc_rng rng, int n, int *r, int a, int b);

/** \brief `std::bernoulli_distribution` */
void vsmc_bernoulli_distribution(vsmc_rng rng, int n, int *r, double p);

/** \brief `std::binomial_distribution<int>` */
void vsmc_binomial_distribution(vsmc_rng rng, int n, int *r, int t, double p);

/** \brief `std::negative_binomial_distribution<int>` */
void vsmc_negative_binomial_distribution(
    vsmc_rng rng, int n, int *r, int k, double p);

/** \brief `std::geometric_distribution<int>` */
void vsmc_geometric_distribution(vsmc_rng rng, int n, int *r, double p);

/** \brief `std::poisson_distribution<int>` */
void vsmc_poisson_distribution(vsmc_rng rng, int n, int *r, double mean);

/** \brief `vsmc::beta_distribution<double>` */
void vsmc_beta_distribution(
    vsmc_rng rng, int n, double *r, double alpha, double beta);

/** \brief `vsmc::cauchy_distribution<double>` */
void vsmc_cauchy_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::chi_squared_distribution<double>` */
void vsmc_chi_squared_distribution(vsmc_rng rng, int n, double *r, double df);

/** \brief `vsmc::exponential_distribution<double>` */
void vsmc_exponential_distribution(
    vsmc_rng rng, int n, double *r, double lambda);

/** \brief `vsmc::extreme_value_distribution<double>` */
void vsmc_extreme_value_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::fisher_f_distribution<double>` */
void vsmc_fisher_f_distribution(
    vsmc_rng rng, int n, double *r, double df1, double df2);

/** \brief `vsmc::gamma_distribution<double>` */
void vsmc_gamma_distribution(
    vsmc_rng rng, int n, double *r, double alpha, double beta);

/** \brief `vsmc::laplace_distribution<double>` */
void vsmc_laplace_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::levy_distribution<double>` */
void vsmc_levy_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::logistic_distribution<double>` */
void vsmc_logistic_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::lognormal_distribution<double>` */
void vsmc_lognormal_distribution(
    vsmc_rng rng, int n, double *r, double m, double s);

/** \brief `vsmc::normal_distribution<double>` */
void vsmc_normal_distribution(
    vsmc_rng rng, int n, double *r, double mean, double stddev);

/** \brief `vsmc::normal_mv_distribution<double>` */
void vsmc_normal_mv_distribution(vsmc_rng rng, int n, double *r, int dim,
    const double *mean, const double *chol);

/** \brief `vsmc::pareto_distribution<double>` */
void vsmc_pareto_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::rayleigh_distribution<double>` */
void vsmc_rayleigh_distribution(vsmc_rng rng, int n, double *r, double b);

/** \brief `vsmc::student_t_distribution<double>` */
void vsmc_student_t_distribution(vsmc_rng rng, int n, double *r, double df);

/** \brief `vsmc::u01_distribution<double>` */
void vsmc_u01_distribution(vsmc_rng rng, int n, double *r);

/** \brief `vsmc::u01_cc_distribution<double>` */
void vsmc_u01_cc_distribution(vsmc_rng rng, int n, double *r);

/** \brief `vsmc::u01_co_distribution<double>` */
void vsmc_u01_co_distribution(vsmc_rng rng, int n, double *r);

/** \brief `vsmc::u01_oc_distribution<double>` */
void vsmc_u01_oc_distribution(vsmc_rng rng, int n, double *r);

/** \brief `vsmc::u01_oo_distribution<double>` */
void vsmc_u01_oo_distribution(vsmc_rng rng, int n, double *r);

/** \brief `vsmc::uniform_real_distribution<double>` */
void vsmc_uniform_real_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** \brief `vsmc::weibull_distribution<double>` */
void vsmc_weibull_distribution(
    vsmc_rng rng, int n, double *r, double a, double b);

/** @} */ /* C_API_RNG_DISTRIBUITON */

/** \addtogroup C_API_RNG_MLK_BRNG */
/** @{ */

/** \brief `vsmc::mkl_brng<std::mt19937>` */
int vsmc_mkl_brng_mt19937(void);

/** \brief `vsmc::mkl_brng<std::mt19937_64>` */
int vsmc_mkl_brng_mt19937_64(void);

/** \brief `vsmc::mkl_brng<std::minstd_rand0>` */
int vsmc_mkl_brng_minstd_rand0(void);

/** \brief `vsmc::mkl_brng<std::minstd_rand>` */
int vsmc_mkl_brng_minstd_rand(void);

/** \brief `vsmc::mkl_brng<std::ranlux24_base>` */
int vsmc_mkl_brng_ranlux24_base(void);

/** \brief `vsmc::mkl_brng<std::ranlux48_base>` */
int vsmc_mkl_brng_ranlux48_base(void);

/** \brief `vsmc::mkl_brng<std::ranlux24>` */
int vsmc_mkl_brng_ranlux24(void);

/** \brief `vsmc::mkl_brng<std::ranlux48>` */
int vsmc_mkl_brng_ranlux48(void);

/** \brief `vsmc::mkl_brng<std::knuth_b>` */
int vsmc_mkl_brng_knuth_b(void);

/** \brief `vsmc::mkl_brng<vsmc::Philox2x32>` */
int vsmc_mkl_brng_philox2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::Philox4x32>` */
int vsmc_mkl_brng_philox4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::Philox2x64>` */
int vsmc_mkl_brng_philox2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::Philox4x64>` */
int vsmc_mkl_brng_philox4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x32>` */
int vsmc_mkl_brng_threefry2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x32>` */
int vsmc_mkl_brng_threefry4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x64>` */
int vsmc_mkl_brng_threefry2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x64>` */
int vsmc_mkl_brng_threefry4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x32SSE2>` */
int vsmc_mkl_brng_threefry2x32sse2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x32SSE2>` */
int vsmc_mkl_brng_threefry4x32sse2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x64SSE2>` */
int vsmc_mkl_brng_threefry2x64sse2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x64SSE2>` */
int vsmc_mkl_brng_threefry4x64sse2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x32AVX2>` */
int vsmc_mkl_brng_threefry2x32avx2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x32AVX2>` */
int vsmc_mkl_brng_threefry4x32avx2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry2x64AVX2>` */
int vsmc_mkl_brng_threefry2x64avx2(void);

/** \brief `vsmc::mkl_brng<vsmc::Threefry4x64AVX2>` */
int vsmc_mkl_brng_threefry4x64avx2(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_1x32>` */
int vsmc_mkl_brng_aes128_1x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_2x32>` */
int vsmc_mkl_brng_aes128_2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_4x32>` */
int vsmc_mkl_brng_aes128_4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_8x32>` */
int vsmc_mkl_brng_aes128_8x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_1x64>` */
int vsmc_mkl_brng_aes128_1x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_2x64>` */
int vsmc_mkl_brng_aes128_2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_4x64>` */
int vsmc_mkl_brng_aes128_4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES128_8x64>` */
int vsmc_mkl_brng_aes128_8x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_1x32>` */
int vsmc_mkl_brng_aes192_1x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_2x32>` */
int vsmc_mkl_brng_aes192_2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_4x32>` */
int vsmc_mkl_brng_aes192_4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_8x32>` */
int vsmc_mkl_brng_aes192_8x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_1x64>` */
int vsmc_mkl_brng_aes192_1x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_2x64>` */
int vsmc_mkl_brng_aes192_2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_4x64>` */
int vsmc_mkl_brng_aes192_4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES192_8x64>` */
int vsmc_mkl_brng_aes192_8x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_1x32>` */
int vsmc_mkl_brng_aes256_1x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_2x32>` */
int vsmc_mkl_brng_aes256_2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_4x32>` */
int vsmc_mkl_brng_aes256_4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_8x32>` */
int vsmc_mkl_brng_aes256_8x32(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_1x64>` */
int vsmc_mkl_brng_aes256_1x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_2x64>` */
int vsmc_mkl_brng_aes256_2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_4x64>` */
int vsmc_mkl_brng_aes256_4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::AES256_8x64>` */
int vsmc_mkl_brng_aes256_8x64(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_1x32>` */
int vsmc_mkl_brng_ars_1x32(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_2x32>` */
int vsmc_mkl_brng_ars_2x32(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_4x32>` */
int vsmc_mkl_brng_ars_4x32(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_8x32>` */
int vsmc_mkl_brng_ars_8x32(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_1x64>` */
int vsmc_mkl_brng_ars_1x64(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_2x64>` */
int vsmc_mkl_brng_ars_2x64(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_4x64>` */
int vsmc_mkl_brng_ars_4x64(void);

/** \brief `vsmc::mkl_brng<vsmc::ARS_8x64>` */
int vsmc_mkl_brng_ars_8x64(void);

/** \brief `vsmc::mkl_brng<vsmc::RDRAND16>` */
int vsmc_mkl_brng_rdrand16(void);

/** \brief `vsmc::mkl_brng<vsmc::RDRAND32>` */
int vsmc_mkl_brng_rdrand32(void);

/** \brief `vsmc::mkl_brng<vsmc::RDRAND64>` */
int vsmc_mkl_brng_rdrand64(void);

/** @} */ /* C_API_RNG_MLK_BRNG */

/** \addtogroup C_API_RNG_RandomWalk */
/** @{ */

/** \brief `vsmc::RandomWalk<double, vsmc::Dynamic>` */
int vsmc_random_walk(vsmc_rng rng, int dim, double *x, double *ltx,
    double (*log_target)(int, const double *),
    double (*proposal)(vsmc_rng, int, const double *, double *));

/** \brief `vsmc::RandomWalkG<double, vsmc::Dynamic, vsmc::Dynamic>` */
int vsmc_random_walk_g(vsmc_rng rng, int dim_x, int dim_g, double *x,
    double *ltx, double *g,
    double (*log_target)(int, int, const double *, double *),
    double (*proposal)(vsmc_rng, int, const double *, double *));

/** \brief `vsmc::NormalProposal<double>` */
double vsmc_normal_proposal(vsmc_rng rng, int, const double *x, double *y,
    double stddev, double a, double b);

/** \brief `vsmc::NormalMVProposal<double, vsmc::Dynamic>` */
double vsmc_normal_mv_proposal(vsmc_rng rng, int dim, const double *x,
    double *y, const double *chol, const double *a, const double *b);

/** @} */ /* C_API_RNG_RandomWalk */

/** \addtogroup C_API_Resample */
/** @{ */

/** \brief `vsmc::Sampler::resample_type` */
typedef void (*vsmc_resample_type)(int, int, vsmc_rng, const double *, int *);

/** \brief `vsmc::resample_trans_u01_rep` */
void vsmc_resample_trans_u01_rep(
    int m, int n, const double *weight, const double *u01, int *replication);

/** \brief `vsmc::resample_trans_u01_index` */
void vsmc_resample_trans_u01_index(
    int m, int n, const double *weight, const double *u01, int *index);

/** \brief `vsmc::resample_trans_rep_index` */
void vsmc_resample_trans_rep_index(
    int m, int n, const int *replication, int *index);

/** \brief `vsmc::resample_trans_index_rep` */
void vsmc_resample_trans_index_rep(
    int m, int n, const int *index, int *replication);

/** \brief `vsmc::resample_trans_residual` */
int vsmc_resample_trans_residual(
    int m, int n, const double *weight, double *resid, int *integ);

/** \brief `vsmc::ResampleMultinomial` */
void vsmc_resample_multinomial(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** \brief `vsmc::ResampleStratified` */
void vsmc_resample_stratified(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** \brief `vsmc::ResampleSystematic` */
void vsmc_resample_systematic(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** \brief `vsmc::ResampleResidual` */
void vsmc_resample_residual(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** \brief `vsmc::ResampleResidualStratified` */
void vsmc_resample_residual_stratified(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** \brief `vsmc::ResampleSystematic` */
void vsmc_resample_residual_systematic(
    int m, int n, vsmc_rng rng, const double *weight, int *replication);

/** @} */ /* C_API_Resample */

/** \addtogroup C_API_Core_StateMatrix */
/** @{ */

/** \brief `vsmc::StateMatrix<vsmc::RowMajor, vsmc::Dynamic, double>` */
typedef struct {
    void *ptr;
} vsmc_state_matrix;

/** \brief `vsmc::StateMatrix::StateMatrix` */
vsmc_state_matrix vsmc_state_matrix_new(int n, int dim);

/** \brief `vsmc::StateMatrix::~StateMatrix` */
void vsmc_state_matrix_delete(vsmc_state_matrix *state_matrix_ptr);

/** \brief `vsmc::StateMatrix::operator=` */
void vsm_state_matrix_assign(
    vsmc_state_matrix state_matrix, vsmc_state_matrix other);

/** \brief `vsmc::StateMatrix::dim` */
int vsmc_state_matrix_dim(vsmc_state_matrix state_matrix);

/** \brief `vsmc::StateMatrix::resize_dim` */
void vsmc_state_matrix_resize_dim(vsmc_state_matrix state_matrix, int n);

/** \brief `vsmc::StateMatrix::size` */
int vsmc_state_matrix_size(vsmc_state_matrix state_matrix);

/** \brief `vsmc::StateMatrix::state` */
double vsmc_state_matrix_get(vsmc_state_matrix state_matrix, int id, int pos);

/** \brief `vsmc::StateMatrix::state` */
void vsmc_state_matrix_set(
    vsmc_state_matrix state_matrix, int id, int pos, double s);

/** \brief `vsmc::StateMatrix::data` */
double *vsmc_state_matrix_data(vsmc_state_matrix state_matrix);

/** \brief `vsmc::StateMatrix::row_data` */
double *vsmc_state_matrix_row_data(vsmc_state_matrix state_matrix, int id);

/** \brief `vsmc::StateMatrix::read_state` */
double *vsmc_state_matrix_read_state(
    vsmc_state_matrix state_matrix, int pos, double *first);

/** \brief `vsmc::StateMatrix::read_state_list` */
double *const *vsmc_state_matrix_read_state_list(
    vsmc_state_matrix state_matrix, double *const *first);

/** \brief `vsmc::StateMatrix::read_state_matrix` */
double *vsmc_state_matrix_read_state_matrix(
    vsmc_state_matrix state_matrix, vSMCMatrixLayout layout, double *first);

/** \brief `vsmc::StateMatrix::copy` */
void vsmc_state_matrix_copy(
    vsmc_state_matrix state_matrix, int N, const int *index);

/** \brief `vsmc::StateMatrix::copy_particle` */
void vsmc_state_matrix_copy_particle(
    vsmc_state_matrix state_matrix, int src, int dst);

/** @} */ /* C_API_Core_StateMatrix */

/** \addtogroup C_API_Core_Weight */
/** @{ */

/** \brief `vsmc::Weight` */
typedef struct {
    void *ptr;
} vsmc_weight;

/** \brief `vsmc::Weight::Weight` */
vsmc_weight vsmc_weight_new(int n);

/** \brief `vsmc::Weight::~Weight` */
void vsmc_weight_delete(vsmc_weight *weight_ptr);

/** \brief `vsmc::Weight::operator=` */
void vsmc_weight_assign(vsmc_weight weight, vsmc_weight other);

/** \brief `vsmc::Weight::size` */
int vsmc_weight_size(vsmc_weight weight);

/** \brief `vsmc::Weight::ess` */
double vsmc_weight_ess(vsmc_weight weight);

/** \brief `vsmc::Weight::data` */
const double *vsmc_weight_data(vsmc_weight weight);

/** \brief `vsmc::Weight::equal` */
void vsmc_weight_set_equal(vsmc_weight weight);

/** \brief `vsmc::Weight::set` */
void vsmc_weight_set(vsmc_weight weight, const double *first, int stride);

/** \brief `vsmc::Weight::mul` */
void vsmc_weight_mul(vsmc_weight weight, const double *first, int stride);

/** \brief `vsmc::Weight::set_log` */
void vsmc_weight_set_log(vsmc_weight weight, const double *first, int stride);

/** \brief `vsmc::Weight::log` */
void vsmc_weight_add_log(vsmc_weight weight, const double *first, int stride);

/** \brief `vsmc::Weight::draw` */
int vsmc_weight_draw(vsmc_weight weight, vsmc_rng rng);

/** @} */ /* C_API_Core_Weight */

/** \addtogroup C_API_Core_SingleParticle */
/** @{ */

/** \brief `vsmc::SingleParticle` */
typedef struct {
    double *state;
    int id;
} vsmc_single_particle;

/** @} C_API_Core_SingleParticle */

/** \addtogroup C_API_Core_Particle */
/** @{ */

/** \brief `vsmc::Particle` */
typedef struct {
    void *ptr;
} vsmc_particle;

/** \brief `vsmc::Particle::Particle` */
vsmc_particle vsmc_particle_new(int n, int dim);

/** \brief `vsmc::Particle::~Particle` */
void vsmc_particle_delete(vsmc_particle *particle_ptr);

/** \brief `vsmc::Particle::operator=` */
void vsmc_particle_assign(vsmc_particle particle, vsmc_particle other);

/** \brief `vsmc::Particle::clone` */
void vsmc_particle_clone(
    vsmc_particle particle, vsmc_particle other, int retain_rng);

/** \brief `vsmc::Particle::size` */
int vsmc_particle_size(vsmc_particle particle);

/** \brief `vsmc:Particle::value` */
vsmc_state_matrix vsmc_particle_value(vsmc_particle particle);

/** \brief `vsmc:Particle::weight` */
vsmc_weight vsmc_particle_weight(vsmc_particle particle);

/** \brief `vsmc::Particle::rng` */
vsmc_rng vsmc_particle_rng(vsmc_particle particle, int id);

/** \brief `vsmc::Particle::sp` */
vsmc_single_particle vsmc_particle_sp(vsmc_particle particle, int id);

/** \brief `vsmc::Particle::resample` */
void vsmc_particle_resample(
    vsmc_particle particle, vsmc_resample_type op, double threshold);

/** @} */ /* C_API_Core_Particle */

/** \addtogroup C_API_Core_Monitor */
/** @{ */

/** \brief `vsmc::Monitor` */
typedef struct {
    void *ptr;
} vsmc_monitor;

/** \brief `vsmc::Monitor::eval_type` */
typedef void (*vsmc_monitor_eval_type)(int, int, vsmc_particle, double *);

/** \brief `vsmc::Monitor::Monitor` */
vsmc_monitor vsmc_monitor_new(int dim, vsmc_monitor_eval_type eval,
    int record_only, vSMCMonitorStage stage);

/** \brief `vsmc::Monitor::~Monitor` */
void vsmc_monitor_delete(vsmc_monitor *monitor_ptr);

/** \brief `vsmc::Monitor::operator=` */
void vsmc_monitor_assign(vsmc_monitor monitor, vsmc_monitor other);

/** \brief `vsmc::Monitor::dim` */
int vsmc_monitor_dim(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::record_only` */
int vsmc_monitor_record_only(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::stage` */
vSMCMonitorStage vsmc_monitor_stage(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::iter_size` */
int vsmc_monitor_iter_size(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::reserve` */
void vsmc_monitor_reserve(vsmc_monitor monitor, int num);

/** \brief `vsmc::Monitor::empty` */
int vsmc_monitor_empty(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::set_name` */
void vsmc_monitor_set_name(vsmc_monitor monitor, int id, const char *name);

/** \brief `vsmc::Monitor::get_name` */
int vsmc_monitor_get_name(vsmc_monitor monitor, int id, char *name);

/** \brief `vsmc::Monitor::index`
 *
 * \details
 * If `iter < 0`, call `monitor.index()`, otherwise call `monitor.index(iter)`
 * */
int vsmc_monitor_index(vsmc_monitor monitor, int iter);

/** \brief `vsmc::Monitor::read_index` */
int *vsmc_monitor_read_index(vsmc_monitor monitor, int *fist);

/** \brief `vsmc::Monitor::record`
 *
 * \details
 * If `iter < 0`, then call `monitor.record(id)`, otherwise call
 * `monitor.record(id, iter)`; */
double vsmc_monitor_record(vsmc_monitor monitor, int id, int iter);

/** \brief `vsmc::Monitor::read_record` */
double *vsmc_monitor_read_record(vsmc_monitor monitor, int id, double *first);

/** \brief `vsmc::Monitor::read_record_list` */
double *const *vsmc_monitor_read_record_list(
    vsmc_monitor monitor, double *const *first);

/** \brief `vsmc::Monitor::read_record_matrix` */
double *vsmc_monitor_read_record_matrix(
    vsmc_monitor monitor, vSMCMatrixLayout layout, double *first);

/** \brief `vsmc::Monitor::set_eval` */
void vsmc_monitor_set_eval(
    vsmc_monitor monitor, vsmc_monitor_eval_type new_eval);

/** \brief `vsmc::Monitor::eval` */
void vsmc_monitor_eval(vsmc_monitor monitor, int iter, vsmc_particle particle,
    vSMCMonitorStage stage);

/** \brief `vsmc::Monitor::clear` */
void vsmc_monitor_clear(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::recording` */
int vsmc_monitor_recording(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::turn_on` */
void vsmc_monitor_turn_on(vsmc_monitor monitor);

/** \brief `vsmc::Monitor::turn_off` */
void vsmc_monitor_turn_off(vsmc_monitor monitor);

/** @} */ /* C_API_Core_Monitor */

/** \addtogroup C_API_Core_Sampler */
/** @{ */

/** \brief `vsmc::Sampler` */
typedef struct {
    void *ptr;
} vsmc_sampler;

/** \brief `vsmc::Sampler::init_type` */
typedef int (*vsmc_sampler_init_type)(vsmc_particle, void *);

/** \brief `vsmc::Sampler::move_type` */
typedef int (*vsmc_sampler_move_type)(int, vsmc_particle);

/** \brief `vsmc::Sampler::mcmc_type` */
typedef int (*vsmc_sampler_mcmc_type)(int, vsmc_particle);

/** \brief `vsmc::Sampler::Sampler` */
vsmc_sampler vsmc_sampler_new(
    int n, int dim, vSMCResampleScheme scheme, double threshold);

/** \brief `vsmc::Sampler::~Sampler` */
void vsmc_sampler_delete(vsmc_sampler *sampler_ptr);

/** \brief `vsmc::Sampler::operator=` */
void vsmc_sampler_assign(vsmc_sampler sampler, vsmc_sampler other);

/** \brief `vsmc::Sampler::clone` */
void vsmc_sampler_clone(
    vsmc_sampler sampler, vsmc_sampler other, int retain_rng);

/** \brief `vsmc::Sampler::size` */
int vsmc_sampler_size(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::reserve` */
void vsmc_sampler_reserve(vsmc_sampler sampler, int num);

/** \brief `vsmc::Sampler::iter_size` */
int vsmc_sampler_iter_size(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::iter_num` */
int vsmc_sampler_iter_num(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::accept_size` */
int vsmc_sampler_accept_size(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::resample` */
void vsmc_sampler_resample(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::resample_scheme` */
void vsmc_sampler_resample_scheme_f(
    vsmc_sampler sampler, vsmc_resample_type op);

/** \brief `vsmc::Sampler::resample_scheme` */
void vsmc_sampler_resample_scheme(
    vsmc_sampler sampler, vSMCResampleScheme scheme);

/** \brief `vsmc::Sampler::threshold` */
double vsmc_sampler_get_threshold(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::threshold` */
void vsmc_sampler_set_threshold(vsmc_sampler sampler, double threshold);

/** \brief `vsmc::Sampler::threshold_never` */
double vsmc_sampler_resample_threshold_never(void);

/** \brief `vsmc::Sampler::threshold_always` */
double vsmc_sampler_resample_threshold_always(void);

/** \brief `vsmc::Sampler::size_history` */
int vsmc_sampler_size_history(vsmc_sampler sampler, int iter);

/** \brief `vsmc::Sampler::read_size_history` */
int *vsmc_sampler_read_size_history(vsmc_sampler sampler, int *first);

/** \brief `vsmc::Sampler::ess_history` */
double vsmc_sampler_ess_history(vsmc_sampler sampler, int iter);

/** \brief `vsmc::Sampler::read_ess_history` */
double *vsmc_sampler_read_ess_history(vsmc_sampler sampler, double *first);

/** \brief `vsmc::Sampler::resampled_history` */
int vsmc_sampler_resampled_history(vsmc_sampler sampler, int iter);

/** \brief `vsmc::Sampler::read_resampled_history` */
int *vsmc_sampler_read_resampled_history(vsmc_sampler sampler, int *first);

/** \brief `vsmc::Sampler::accept_history` */
int vsmc_sampler_accept_history(vsmc_sampler sampler, int id, int iter);

/** \brief `vsmc::Sampler::read_accept_history` */
int *vsmc_sampler_read_accept_history(vsmc_sampler, int id, int *first);

/** \brief `vsmc::Sampler::read_accept_history_list` */
int *const *vsmc_sampler_read_accept_history_list(
    vsmc_sampler sampler, int *const *first);

/** \brief `vsmc::Sampler::read_accept_history_matrix` */
int *vsmc_sampler_read_accept_history_matrix(
    vsmc_sampler sampler, vSMCMatrixLayout layout, int *first);

/** \brief `vsmc::Sampler::particle` */
vsmc_particle vsmc_sampler_particle(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::init` */
void vsmc_sampler_init(vsmc_sampler sampler, vsmc_sampler_init_type new_init);

/** \brief `vsmc::Sampler::init_by_iter` */
void vsmc_sampler_init_by_iter(
    vsmc_sampler sampler, int initialize_by_iterate);

/** \brief `vsmc::Sampler::init_by_move` */
void vsmc_sampler_init_by_move(
    vsmc_sampler sampler, vsmc_sampler_move_type new_init);

/** \brief `vsmc::Sampler::move_queue_clear` */
void vsmc_sampler_move_queue_clear(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::move_queue_empty` */
int vsmc_sampler_move_queue_empty(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::move_queue_size` */
int vsmc_sampler_move_queue_size(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::move` */
void vsmc_sampler_move(
    vsmc_sampler sampler, vsmc_sampler_move_type new_move, int append);

/** \brief `vsmc::Sampler::mcmc_queue_clear` */
void vsmc_sampler_mcmc_queue_clear(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::mcmc_queue_empty` */
int vsmc_sampler_mcmc_queue_empty(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::mcmc_queue_size` */
int vsmc_sampler_mcmc_queue_size(vsmc_sampler sampler);

/** \brief `vsmc::Sampler::mcmc` */
void vsmc_sampler_mcmc(
    vsmc_sampler sampler, vsmc_sampler_mcmc_type new_mcmc, int append);

/** \brief `vsmc::Sampler::initialize` */
void vsmc_sampler_initialize(vsmc_sampler sampler, void *param);

/** \brief `vsmc::Sampler::iterate` */
void vsmc_sampler_iterate(vsmc_sampler sampler, int num);

/** \brief `vsmc::Sampler::monitor` */
void vsmc_sampler_set_monitor_direct(
    vsmc_sampler sampler, const char *name, vsmc_monitor mon);

/** \brief `vsmc::Sampler::monitor` */
void vsmc_sampler_set_monitor(vsmc_sampler sampler, const char *name, int dim,
    vsmc_monitor_eval_type eval, int record_only, vSMCMonitorStage stage);

/** \brief `vsmc::Sampler::monitor` */
vsmc_monitor vsmc_sampler_get_monitor(vsmc_sampler sampler, const char *name);

/** \brief `vsmc::Sampler::clear_monitor` */
void vsmc_sampler_clear_monitor(vsmc_sampler sampler, const char *name);

/** \brief `vsmc::Sampler::clear_monitor` */
void vsmc_sampler_clear_monitor_all(vsmc_sampler sampler);

/** \brief `operator<<(vsmc::Sampler)` */
int vsmc_sampler_save(vsmc_sampler sampler, char *mem);

/** \brief `operator<<(vsmc::Sampler::Sampler)` direct to file */
void vsmc_sampler_save_f(vsmc_sampler sampler, const char *filename);

/** @} */ /* C_API_Core_Sampler */

/** \addtogroup C_API_SMP */
/** @{ */

/** \brief `vsmc::InitializeBase::eval_sp` */
typedef int (*vsmc_initialize_eval_sp_type)(vsmc_single_particle);

/** \brief `vsmc::InitializeBase::eval_param` */
typedef void (*vsmc_initialize_eval_param_type)(vsmc_particle, void *);

/** \brief `vsmc::InitializeBase::eval_pre` */
typedef void (*vsmc_initialize_eval_pre_type)(vsmc_particle);

/** \brief `vsmc::InitializeBase::eval_post` */
typedef void (*vsmc_initialize_eval_post_type)(vsmc_particle);

/** \brief `vsmc::MoveBase::eval_sp` */
typedef int (*vsmc_move_eval_sp_type)(int, vsmc_single_particle);

/** \brief `vsmc::MoveBase::eval_pre` */
typedef void (*vsmc_move_eval_pre_type)(int, vsmc_particle);

/** \brief `vsmc::MoveBase::eval_post` */
typedef void (*vsmc_move_eval_post_type)(int, vsmc_particle);

/** \brief `vsmc::MonitorEvalBase::eval_sp` */
typedef void (*vsmc_monitor_eval_sp_type)(
    int, int, vsmc_single_particle, double *);

/** \brief `vsmc::MonitorEvalBase::eval_pre` */
typedef void (*vsmc_monitor_eval_pre_type)(int, vsmc_particle);

/** \brief `vsmc::MonitorEvalBase::eval_post` */
typedef void (*vsmc_monitor_eval_post_type)(int, vsmc_particle);

/** @} */ /* C_API_SMP */

/** \addtogroup C_API_SMP_SEQ */
/** @{ */

/** \brief `vsmc::Sampler::init` with `vsmc::InitializeSEQ` as input */
void vsmc_sampler_init_seq(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/** \brief `vsmc::Sampler::init_by_move` with `vsmc::MoveSEQ` as input */
void vsmc_sampler_init_by_move_seq(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post);

/** \brief `vsmc::Sampler::move` with `vsmc::MoveSEQ` as input */
void vsmc_sampler_move_seq(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::mcmc` with `vsmc::MoveSEQ` as input */
void vsmc_sampler_mcmc_seq(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalSEQ` as input */
void vsmc_sampler_set_monitor_seq(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/** @} */ /* C_API_SMP_SEQ */

/** \addtogroup C_API_SMP_OMP */
/** @{ */

/** \brief `vsmc::Sampler::init` with `vsmc::InitializeOMP` as input */
void vsmc_sampler_init_omp(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/** \brief `vsmc::Sampler::init_by_move` with `vsmc::MoveOMP` as input */
void vsmc_sampler_init_by_move_omp(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post);

/** \brief `vsmc::Sampler::move` with `vsmc::MoveOMP` as input */
void vsmc_sampler_move_omp(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::mcmc` with `vsmc::MoveOMP` as input */
void vsmc_sampler_mcmc_omp(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalOMP` as input */
void vsmc_sampler_set_monitor_omp(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/** @} */ /* C_API_SMP_OMP */

/** \addtogroup C_API_SMP_TBB */
/** @{ */

/** \brief `vsmc::Sampler::init` with `vsmc::InitializeTBB` as input */
void vsmc_sampler_init_tbb(vsmc_sampler sampler,
    vsmc_initialize_eval_sp_type eval_sp,
    vsmc_initialize_eval_param_type eval_param,
    vsmc_initialize_eval_pre_type eval_pre,
    vsmc_initialize_eval_post_type eval_post);

/** \brief `vsmc::Sampler::init_by_move` with `vsmc::MoveTBB` as input */
void vsmc_sampler_init_by_move_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post);

/** \brief `vsmc::Sampler::move` with `vsmc::MoveTBB` as input */
void vsmc_sampler_move_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::mcmc` with `vsmc::MoveTBB` as input */
void vsmc_sampler_mcmc_tbb(vsmc_sampler sampler,
    vsmc_move_eval_sp_type eval_sp, vsmc_move_eval_pre_type eval_pre,
    vsmc_move_eval_post_type eval_post, int append);

/** \brief `vsmc::Sampler::monitor` with `vsmc::MonitorEvalTBB` as input */
void vsmc_sampler_set_monitor_tbb(vsmc_sampler sampler, const char *name,
    int dim, vsmc_monitor_eval_sp_type eval_sp,
    vsmc_monitor_eval_pre_type eval_pre, vsmc_monitor_eval_post_type eval_post,
    int record_only, vSMCMonitorStage stage);

/** @} */ /* C_API_SMP_TBB */

/** \addtogroup C_API_Utility_AlignedMemory */
/** @{ */

/** \brief `vsmc::AlignedMemory::aligned_malloc` */
void *vsmc_malloc(size_t n, int alignment);

/** \brief `vsmc::AlignedMemory::aligned_free` */
void vsmc_free(void *ptr);

/** \brief Simple vector type for `double` */
typedef struct {
    double *data;
    int size;
} vsmc_vector;

/** \brief Simple vector type for `int` */
typedef struct {
    int *data;
    int size;
} vsmc_vector_int;

/** \brief Simple vector type for `unsigned char` */
typedef struct {
    unsigned char *data;
    int size;
} vsmc_vector_raw;

/** \brief Create a new vector given its size  */
vsmc_vector vsmc_vector_new(int size);

/** \brief Delete a vector */
void vsmc_vector_delete(vsmc_vector *vector_ptr);

/** \brief Resize a vector */
void vsmc_vector_resize(vsmc_vector *vector_ptr, int size);

/** \brief Create a new vector given its size  */
vsmc_vector_int vsmc_vector_int_new(int size);

/** \brief Delete a vector */
void vsmc_vector_int_delete(vsmc_vector_int *vector_int_ptr);

/** \brief Resize a vector */
void vsmc_vector_int_resize(vsmc_vector_int *vector_int_ptr, int size);

/** \brief Create a new vector given its size  */
vsmc_vector_raw vsmc_vector_raw_new(int size);

/** \brief Delete a vector */
void vsmc_vector_raw_delete(vsmc_vector_raw *vector_raw_ptr);

/** \brief Resize a vector */
void vsmc_vector_raw_resize(vsmc_vector_raw *vector_raw_ptr, int size);

/** @} */ /* C_API_Utility_AlignedMemory */

/** \addtogroup C_API_Utility_Covariance */
/** @{ */

typedef struct {
    void *ptr;
} vsmc_covariance;

/** \brief `vsmc::cov_chol` */
int vsmc_cov_chol(int dim, const double *cov, double *chol,
    vSMCMatrixLayout layout, int upper, int packed);

/** \brief `vsmc::Covariance::Covariance` */
vsmc_covariance vsmc_covariance_new(void);

/** \brief `vsmc::Covariance::~Covariance` */
void vsmc_covariance_delete(vsmc_covariance *covariance_ptr);

/** \brief `vsmc::Covariance::operator=` */
void vsmc_covariance_assign(vsmc_covariance covariance, vsmc_covariance other);

/** \brief `vsmc::Covariance::operator()` */
void vsmc_covariance_compute(vsmc_covariance covariance,
    vSMCMatrixLayout layout, int n, int dim, const double *x, const double *w,
    double *mean, double *cov, vSMCMatrixLayout cov_layout, int cov_upper,
    int cov_packed);

/** @} */ /* C_API_Utility_Covariance */

/** \addtogroup C_API_Utility_HDF5IO */
/** @{ */

/** \brief `vsmc::hdf5size` */
int vsmc_hdf5size(const char *file_name, const char *data_name);

/** \brief `vsmc::hdf5load<double>` */
double *vsmc_hdf5load(
    const char *file_name, const char *data_name, double *first);

/** \brief `vsmc::hdf5load<int>` */
int *vsmc_hdf5load_int(
    const char *file_name, const char *data_name, int *first);

/** \brief `vsmc::hdf5load<unsigned char>` */
unsigned char *vsmc_hdf5load_raw(
    const char *file_name, const char *data_name, unsigned char *first);

/** \brief `vsmc::hdf5store_new` */
void vsmc_hdf5store_new(const char *file_name);

/** \brief `vsmc::hdf5store_matrix<double>` */
void vsmc_hdf5store_matrix(vSMCMatrixLayout layout, int nrow, int ncol,
    const char *file_name, const char *data_name, const double *first,
    int append);

/** \brief `vsmc::hdf5store_matrix<int>` */
void vsmc_hdf5store_matrix_int(vSMCMatrixLayout layout, int nrow, int ncol,
    const char *file_name, const char *data_name, const int *first,
    int append);

/** \brief `vsmc::hdf5store_matrix<unsigned char>` */
void vsmc_hdf5store_matrix_raw(vSMCMatrixLayout layout, int nrow, int ncol,
    const char *file_name, const char *data_name, const unsigned char *first,
    int append);

/** \brief `vsmc::hdf5store_list_empty` */
void vsmc_hdf5store_list_empty(
    const char *file_name, const char *data_name, int append);

/** \brief `vsmc::hdf5store_list_insert<double>` */
void vsmc_hdf5store_list_insert(int N, const char *file_name,
    const char *data_name, const double *first, const char *vname);

/** \brief `vsmc::hdf5store_list_insert<int>` */
void vsmc_hdf5store_list_insert_int(int N, const char *file_name,
    const char *data_name, const int *first, const char *vname);

/** \brief `vsmc::hdf5store_list_insert<unsigned char>` */
void vsmc_hdf5store_list_insert_raw(int N, const char *file_name,
    const char *data_name, const unsigned char *first, const char *vname);

/** \brief `vsmc::hdf5store` for `vsmc::StateMatrix` */
void vsmc_hdf5store_state_matrix(vsmc_state_matrix state_matrix,
    const char *file_name, const char *data_name, int append);

/** \brief `vsmc::hdf5store` for `vsmc::Particle` */
void vsmc_hdf5store_particle(vsmc_particle particle, const char *file_name,
    const char *data_name, int append);

/** \brief `vsmc::hdf5store` for `vsmc::Monitor` */
void vsmc_hdf5store_monitor(vsmc_monitor monitor, const char *file_name,
    const char *data_name, int append);

/** \brief `vsmc::hdf5store` for `vsmc::Sampler` */
void vsmc_hdf5store_sampler(vsmc_sampler sampler, const char *file_name,
    const char *data_name, int append);

/** @} */ /* C_API_Utility_HDF5IO */

/** \addtogroup C_API_Utility_ProgramOption */
/** @{ */

typedef struct {
    void *ptr;
} vsmc_program_option_map;

/** \brief `vsmc::ProgramOptionMap::ProgramOptionMap` */
vsmc_program_option_map vsmc_program_option_map_new(int silent);

/** \brief `vsmc::ProgramOptionMap::~ProgramOptionMap` */
void vsmc_program_option_map_delete(
    vsmc_program_option_map *program_option_map_ptr);

/** \brief `vsmc::ProgramOptionMap::operator=` */
void vsmc_program_option_map_assign(
    vsmc_program_option_map program_option_map, vsmc_program_option_map other);

/** \brief `vsmc::ProgramOptionMap::add<double>` */
void vsmc_program_option_map_add(vsmc_program_option_map program_option_map,
    const char *name, const char *desc, double *ptr);

/** \brief `vsmc::ProgramOptionMap::add<double, double>` */
void vsmc_program_option_map_add_val(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, double *ptr, double val);

/** \brief `vsmc::ProgramOptionMap::add<int>` */
void vsmc_program_option_map_add_int(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, int *ptr);

/** \brief `vsmc::ProgramOptionMap::add<int, int>` */
void vsmc_program_option_map_add_val_int(
    vsmc_program_option_map program_option_map, const char *name,
    const char *desc, int *ptr, int val);

/** \brief `vsmc::ProgramOptionMap::remove` */
void vsmc_program_option_map_remove(
    vsmc_program_option_map program_option_map, const char *name);

/** \brief `vsmc::ProgramOptionMap::process` */
void vsmc_program_option_map_process(
    vsmc_program_option_map program_option_map, int argc, char *const *argv);

/** \brief `vsmc::ProgramOptionMap::print_help` */
void vsmc_program_option_map_print_help(
    vsmc_program_option_map program_option_map);

/** \brief `vsmc::ProgramOptionMap::count` */
void vsmc_program_option_map_count(
    vsmc_program_option_map program_option_map, const char *name);

/** \brief `vsmc::ProgramOptionMap::help` */
int vsmc_program_option_map_help(vsmc_program_option_map program_option_map);

/** \brief `vsmc::ProgramOptionMap::silent` */
void vsmc_program_option_map_silent(
    vsmc_program_option_map program_option_map, int flag);

/** @} */ /* C_API_Utility_ProgramOption */

/** \addtogroup C_API_Utility_Progress */
/** @{ */

typedef struct {
    void *ptr;
} vsmc_progress;

/** \brief `vsmc:Progress::Progress` */
vsmc_progress vsmc_progress_new(void);

/** \brief `vsmc:Progress::~Progress` */
void vsmc_progress_delete(vsmc_progress *progress_ptr);

/** \brief `vsmc:Progress::start` */
void vsmc_progress_start(vsmc_progress progress, int total, const char *msg,
    int length, int show_iter, double interval_s);

/** \brief `vsmc:Progress::stop` */
void vsmc_progress_stop(vsmc_progress progress, int finished);

/** \brief `vsmc:Progress::increment` */
void vsmc_progress_increment(vsmc_progress progress, int step);

/** \brief `vsmc:Progress::message` */
void vsmc_progress_message(vsmc_progress progress, const char *msg);

/** @} */ /* C_API_Utility_Progress */

/** \addtogroup C_API_Utility_StopWatch */
/** @{ */

/** \brief `vsmc::StopWatch` */
typedef struct {
    void *ptr;
} vsmc_stop_watch;

/** \brief `vsmc::StopWatch::StopWatch` */
vsmc_stop_watch vsmc_stop_watch_new(void);

/** \brief `vsmc::StopWatch::~StopWatch` */
void vsmc_stop_watch_delete(vsmc_stop_watch *stop_watch_ptr);

/** \brief `vsmc::StopWatch::operator=` */
void vsmc_stop_watch_assign(vsmc_stop_watch stop_watch, vsmc_stop_watch other);

/** \brief `vsmc::StopWatch::running` */
int vsmc_stop_watch_running(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::start` */
int vsmc_stop_watch_start(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::stop` */
int vsmc_stop_watch_stop(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::reset` */
void vsmc_stop_watch_reset(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::nanoseconds` */
double vsmc_stop_watch_nanoseconds(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::microseconds` */
double vsmc_stop_watch_microseconds(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::milliseconds` */
double vsmc_stop_watch_milliseconds(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::seconds` */
double vsmc_stop_watch_seconds(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::minutes` */
double vsmc_stop_watch_minutes(vsmc_stop_watch stop_watch);

/** \brief `vsmc::StopWatch::hours` */
double vsmc_stop_watch_hours(vsmc_stop_watch stop_watch);

/** @} */ /* C_API_Utility_StopWatch */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* VSMC_VSMC_H */
