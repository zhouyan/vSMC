/*============================================================================
 * vSMC/include/vsmc/vsmc.h
 *----------------------------------------------------------------------------
 *                         vSMC: Scalable Monte Carlo
 *----------------------------------------------------------------------------
 * Copyright (c) 2013-2015, Yan Zhou
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

/// \defgroup C_API C API
/// @{

/// \defgroup C_API_RNG Random number generating
/// @{

/// \brief `vsmc::RNG`
typedef struct {
    int state[64];
} vsmc_rng;

/// \brief `sizeof(vsmc::RNG) + 4`
int vsmc_rng_size();

/// \brief `vsmc::RNG` constructor
void vsmc_rng_init(vsmc_rng *rng_ptr, int seed);

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

/// \brief `vsmc::RNG::operator()`
void vsmc_rng_rand(vsmc_rng *rng_ptr, int n, int *r);

/// \brief `vsmc::u01_sorted`
void vsmc_rng_u01_sorted(int n, const double *u01, double *u01seq);

/// \brief `vsmc::u01_stratifed`
void vsmc_rng_u01_stratified(int n, const double *u01, double *u01seq);

/// \brief `vsmc::u01_systematic`
void vsmc_rng_u01_systematic(int n, double u01, double *u01seq);

/// \brief `vsmc::DiscreteDistribution<int>`
void vsmc_rng_discrete(vsmc_rng *rng_ptr, int n, int *r, int m,
    const double *weight, int normalized);

/// \brief `std::uniform_int_distribution<int>`
void vsmc_rng_uniform_int(vsmc_rng *rng_ptr, int n, int *r, int a, int b);

/// \brief `std::bernoulli_distribution`
void vsmc_rng_bernoulli(vsmc_rng *rng_ptr, int n, int *r, double p);

/// \brief `std::binomial_distribution<int>`
void vsmc_rng_binomial(vsmc_rng *rng_ptr, int n, int *r, int t, double p);

/// \brief `std::negative_binomial_distribution<int>`
void vsmc_rng_negative_binomial(
    vsmc_rng *rng_ptr, int n, int *r, int k, double p);

/// \brief `std::geometri_distribution<int>`
void vsmc_rng_geometric(vsmc_rng *rng_ptr, int n, int *r, double p);

/// \brief `std::poisson_distribution<int>`
void vsmc_rng_poisson(vsmc_rng *rng_ptr, int n, int *r, double mean);

/// \brief `vsmc::BetaDistribution<double>`
void vsmc_rng_beta(
    vsmc_rng *rng_ptr, int n, double *r, double alpha, double beta);

/// \brief `vsmc::CachyDistribution<double>`
void vsmc_rng_cauchy(vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::ChiSquaredDistribution<double>`
void vsmc_rng_chi_squared(vsmc_rng *rng_ptr, int n, double *r, double df);

/// \brief `vsmc::ExponentialDistribution<double>`
void vsmc_rng_exponential(vsmc_rng *rng_ptr, int n, double *r, double lambda);

/// \brief `vsmc::ExtremeValueDistribution<double>`
void vsmc_rng_extreme_value(
    vsmc_rng *rng_ptr, int n, double *r, double location, double scale);

/// \brief `vsmc::FisherFDistribution<double>`
void vsmc_rng_fisher_f(
    vsmc_rng *rng_ptr, int n, double *r, double df1, double df2);

/// \brief `vsmc::GammaDistribution<double>`
void vsmc_rng_gamma(
    vsmc_rng *rng_ptr, int n, double *r, double alpha, double beta);

/// \brief `vsmc::LaplaceDistribution<double>`
void vsmc_rng_laplace(vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::LognormalDistribution<double>`
void vsmc_rng_lognormal(
    vsmc_rng *rng_ptr, int n, double *r, double m, double s);

/// \brief `vsmc::NormalDistribution<double>`
void vsmc_rng_normal(
    vsmc_rng *rng_ptr, int n, double *r, double mean, double stddev);

/// \brief `vsmc::RayleighDistribution<double>`
void vsmc_rng_rayleigh(vsmc_rng *rng_ptr, int n, double *r, double b);

/// \brief `vsmc::StudentTDistribution<double>`
void vsmc_rng_student_t(vsmc_rng *rng_ptr, int n, double *r, double df);

/// \brief `vsmc::U01Distribution<double>`
void vsmc_rng_u01(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01CCDistribution<double>`
void vsmc_rng_u01_cc(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01CODistribution<double>`
void vsmc_rng_u01_co(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01OCDistribution<double>`
void vsmc_rng_u01_oc(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::U01OODistribution<double>`
void vsmc_rng_u01_oo(vsmc_rng *rng_ptr, int n, double *r);

/// \brief `vsmc::UniformRealDistribution<double>`
void vsmc_rng_uniform_real(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::UniformRealCCDistribution<double>`
void vsmc_rng_uniform_real_cc(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::UniformRealCODistribution<double>`
void vsmc_rng_uniform_real_co(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::UniformRealOCDistribution<double>`
void vsmc_rng_uniform_real_oc(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::UniformRealOODistribution<double>`
void vsmc_rng_uniform_real_oo(
    vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// \brief `vsmc::WeibullDistribution<double>`
void vsmc_rng_weibull(vsmc_rng *rng_ptr, int n, double *r, double a, double b);

/// @}

/// \defgroup C_API_MLK_BRNG Register MKL BRNG (`vsmc::mkl_brng`)
/// @{

int vsmc_mkl_brng_mt19937(void);
int vsmc_mkl_brng_minstd_rand0(void);
int vsmc_mkl_brng_minstd_rand(void);
int vsmc_mkl_brng_mt19937_64(void);
int vsmc_mkl_brng_ranlux24_base(void);
int vsmc_mkl_brng_ranlux48_base(void);
int vsmc_mkl_brng_ranlux24(void);
int vsmc_mkl_brng_ranlux48(void);
int vsmc_mkl_brng_knuth_b(void);
int vsmc_mkl_brng_philox2x32(void);
int vsmc_mkl_brng_philox4x32(void);
int vsmc_mkl_brng_philox2x64(void);
int vsmc_mkl_brng_philox4x64(void);
int vsmc_mkl_brng_threefry2x32(void);
int vsmc_mkl_brng_threefry4x32(void);
int vsmc_mkl_brng_threefry2x64(void);
int vsmc_mkl_brng_threefry4x64(void);
int vsmc_mkl_brng_threefry2x32avx2(void);
int vsmc_mkl_brng_threefry4x32avx2(void);
int vsmc_mkl_brng_threefry2x64avx2(void);
int vsmc_mkl_brng_threefry4x64avx2(void);
int vsmc_mkl_brng_threefry2x32sse2(void);
int vsmc_mkl_brng_threefry4x32sse2(void);
int vsmc_mkl_brng_threefry2x64sse2(void);
int vsmc_mkl_brng_threefry4x64sse2(void);
int vsmc_mkl_brng_aes128_1x32(void);
int vsmc_mkl_brng_aes128_2x32(void);
int vsmc_mkl_brng_aes128_4x32(void);
int vsmc_mkl_brng_aes128_8x32(void);
int vsmc_mkl_brng_aes128_1x64(void);
int vsmc_mkl_brng_aes128_2x64(void);
int vsmc_mkl_brng_aes128_4x64(void);
int vsmc_mkl_brng_aes128_8x64(void);
int vsmc_mkl_brng_aes192_1x32(void);
int vsmc_mkl_brng_aes192_2x32(void);
int vsmc_mkl_brng_aes192_4x32(void);
int vsmc_mkl_brng_aes192_8x32(void);
int vsmc_mkl_brng_aes192_1x64(void);
int vsmc_mkl_brng_aes192_2x64(void);
int vsmc_mkl_brng_aes192_4x64(void);
int vsmc_mkl_brng_aes192_8x64(void);
int vsmc_mkl_brng_aes256_1x32(void);
int vsmc_mkl_brng_aes256_2x32(void);
int vsmc_mkl_brng_aes256_4x32(void);
int vsmc_mkl_brng_aes256_8x32(void);
int vsmc_mkl_brng_aes256_1x64(void);
int vsmc_mkl_brng_aes256_2x64(void);
int vsmc_mkl_brng_aes256_4x64(void);
int vsmc_mkl_brng_aes256_8x64(void);
int vsmc_mkl_brng_ars_1x32(void);
int vsmc_mkl_brng_ars_2x32(void);
int vsmc_mkl_brng_ars_4x32(void);
int vsmc_mkl_brng_ars_8x32(void);
int vsmc_mkl_brng_ars_1x64(void);
int vsmc_mkl_brng_ars_2x64(void);
int vsmc_mkl_brng_ars_4x64(void);
int vsmc_mkl_brng_ars_8x64(void);
int vsmc_mkl_brng_rdrand16(void);
int vsmc_mkl_brng_rdrand32(void);
int vsmc_mkl_brng_rdrand64(void);

/// @}

/// \defgroup C_API_Memory Memory allocation
/// @{

void *vsmc_malloc(size_t n, int alignment);
void vsmc_free(void *ptr);

/// @}

/// \defgroup C_API_Resample Resample algorithms
/// @{

typedef enum {
    VSMC_RESAMPLE_MULTINOMIAL,
    VSMC_RESAMPLE_STRATIFIED,
    VSMC_RESAMPLE_SYSTEMATIC,
    VSMC_RESAMPLE_RESIDUAL,
    VSMC_RESAMPLE_RESIDUAL_STRATIFIED,
    VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC
} vsmc_resample_scheme;

void vsmc_resample_trans_u01_rep(
    int m, int n, const double *weight, const double *u01, int *replication);
void vsmc_resample_trans_u01_index(
    int m, int n, const double *weight, const double *u01, int *src_idx);
void vsmc_resample_trans_rep_index(
    int m, int n, const int *replication, int *src_idx);
void vsmc_resample_trans_index_rep(
    int m, int n, const int *src_idx, int *replication);
int vsmc_resample_trans_residual(
    int m, int n, const double *weight, double *resid, int *integ);

void vsmc_resample_multinomial(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample_stratified(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample_systematic(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample_residual(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample_residual_stratified(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample_residual_systematic(
    int m, int n, vsmc_rng *rng_ptr, const double *weight, int *replication);
void vsmc_resample(int m, int n, vsmc_rng *rng_ptr, const double *weight,
    int *replication, vsmc_resample_scheme scheme);

/// @}

/// @}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_VSMC_H
