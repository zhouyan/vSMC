//============================================================================
// vSMC/include/vsmc/rng/rng.h
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

#ifndef VSMC_RNG_RNG_H
#define VSMC_RNG_RNG_H

#include <vsmc/internal/common.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \addtogroup C_API_RNG_RNG
/// @{

/// \brief `vsmc::RNG::RNG`
vsmc_rng vsmc_rng_new(unsigned seed, vSMCRNGType type);

/// \brief `vsmc::RNG::~RNG`
void vsmc_rng_delete(vsmc_rng *rng_ptr);

/// \brief `vsmc::RNG::operator=`
void vsmc_rng_assign(vsmc_rng rng, vsmc_rng other);

/// \brief `vsmc::RNG::seed`
void vsmc_rng_seed(vsmc_rng rng, unsigned seed);

/// \brief `vsmc::uniform_bits_distribution`
void vsmc_rng_rand(vsmc_rng rng, size_t n, unsigned *r);

/// \brief `vsmc::RNG::discard`
void vsmc_rng_discard(vsmc_rng rng, unsigned nskip);

/// \brief `vsmc::RNG::operator==`
int vsmc_rng_is_eq(vsmc_rng rng1, vsmc_rng rng2);

/// \brief `vsmc::RNG::operator!=`
int vsmc_rng_is_neq(vsmc_rng rng1, vsmc_rng rng2);

/// \brief `vsmc::RNG::operator<<`
size_t vsmc_rng_save(vsmc_rng rng, void *mem);

/// \brief `vsmc::RNG::operator>>`
void vsmc_rng_load(vsmc_rng rng, void *mem);

/// \brief `vsmc::RNG::operator<<` directly to an external file
void vsmc_rng_save_f(vsmc_rng rng, const char *filename);

/// \brief `vsmc::RNG::operator>>` directly from an external file*/
void vsmc_rng_load_f(vsmc_rng rng, const char *filename);

/// @} C_API_RNG_RNG

/// \addtogroup C_API_RNG_Seed
/// @{

/// \brief `vsmc::Seed::operator()`
void vsmc_seed(vsmc_rng rng);

/// \brief `vsmc::Seed::get`
unsigned vsmc_seed_get(void);

/// \brief `vsmc::Seed::set`
void vsmc_seed_set(unsigned seed);

/// \brief `vsmc::Seed::modulo`
void vsmc_seed_modulo(unsigned div, unsigned rem);

/// \brief `vsmc::Seed::operator<<`
size_t vsmc_seed_save(void *mem);

/// \brief `vsmc::Seed::operator>>`
void vsmc_seed_load(const void *mem);

/// \brief `vsmc::Seed::operator<<` directly to an external file
void vsmc_seed_save_f(const char *filename);

/// \brief `vsmc::Seed::operator>>` directly from an external file
void vsmc_seed_load_f(const char *filename);

/// @} C_API_RNG_Seed

/// \addtogroup C_API_RNG_U01Sequence
/// @{

/// \brief `vsmc::u01_trans_sorted`
void vsmc_u01_trans_sorted(size_t n, const double *u01, double *r);

/// \brief `vsmc::u01_trans_stratifed`
void vsmc_u01_trans_stratified(size_t n, const double *u01, double *r);

/// \brief `vsmc::u01_trans_systematic`
void vsmc_u01_trans_systematic(size_t n, const double *u01, double *r);

/// \brief `vsmc::u01_rand_sorted`
void vsmc_u01_rand_sorted(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_rand_stratifed`
void vsmc_u01_rand_stratified(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_rand_systematic`
void vsmc_u01_rand_systematic(vsmc_rng rng, size_t n, double *r);

/// @} C_API_RNG_U01Sequence

/// \addtogroup C_API_RNG_DISTRIBUITON
/// @{

/// \brief `vsmc::DiscreteDistribution<int>`
void vsmc_discrete_rand(vsmc_rng rng, size_t n, int *r, size_t m,
    const double *weight, int normalized);

/// \brief `std::uniform_int_distribution<int>`
void vsmc_uniform_int_rand(vsmc_rng rng, size_t n, int *r, int a, int b);

/// \brief `std::bernoulli_distribution`
void vsmc_bernoulli_rand(vsmc_rng rng, size_t n, int *r, double p);

/// \brief `std::binomial_distribution<int>`
void vsmc_binomial_rand(vsmc_rng rng, size_t n, int *r, int t, double p);

/// \brief `std::negative_binomial_distribution<int>`
void vsmc_negative_binomial_rand(
    vsmc_rng rng, size_t n, int *r, int k, double p);

/// \brief `std::geometric_distribution<int>`
void vsmc_geometric_rand(vsmc_rng rng, size_t n, int *r, double p);

/// \brief `std::poisson_distribution<int>`
void vsmc_poisson_rand(vsmc_rng rng, size_t n, int *r, double mean);

/// \brief `vsmc::beta_distribution<double>`
void vsmc_beta_rand(
    vsmc_rng rng, size_t n, double *r, double alpha, double beta);

/// \brief `vsmc::cauchy_distribution<double>`
void vsmc_cauchy_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::chi_squared_distribution<double>`
void vsmc_chi_squared_rand(vsmc_rng rng, size_t n, double *r, double df);

/// \brief `vsmc::exponential_distribution<double>`
void vsmc_exponential_rand(vsmc_rng rng, size_t n, double *r, double lambda);

/// \brief `vsmc::extreme_value_distribution<double>`
void vsmc_extreme_value_rand(
    vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::fisher_f_distribution<double>`
void vsmc_fisher_f_rand(
    vsmc_rng rng, size_t n, double *r, double df1, double df2);

/// \brief `vsmc::gamma_distribution<double>`
void vsmc_gamma_rand(
    vsmc_rng rng, size_t n, double *r, double alpha, double beta);

/// \brief `vsmc::laplace_distribution<double>`
void vsmc_laplace_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::levy_distribution<double>`
void vsmc_levy_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::logistic_distribution<double>`
void vsmc_logistic_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::lognormal_distribution<double>`
void vsmc_lognormal_rand(
    vsmc_rng rng, size_t n, double *r, double m, double s);

/// \brief `vsmc::normal_distribution<double>`
void vsmc_normal_rand(
    vsmc_rng rng, size_t n, double *r, double mean, double stddev);

/// \brief `vsmc::normal_mv_distribution<double>`
void vsmc_normal_mv_rand(vsmc_rng rng, size_t n, double *r, size_t dim,
    const double *mean, const double *chol);

/// \brief `vsmc::pareto_distribution<double>`
void vsmc_pareto_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::rayleigh_distribution<double>`
void vsmc_rayleigh_rand(vsmc_rng rng, size_t n, double *r, double b);

/// \brief `vsmc::student_t_distribution<double>`
void vsmc_student_t_rand(vsmc_rng rng, size_t n, double *r, double df);

/// \brief `vsmc::u01_distribution<double>`
void vsmc_u01_rand(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_cc_distribution<double>`
void vsmc_u01_cc_rand(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_co_distribution<double>`
void vsmc_u01_co_rand(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_oc_distribution<double>`
void vsmc_u01_oc_rand(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::u01_oo_distribution<double>`
void vsmc_u01_oo_rand(vsmc_rng rng, size_t n, double *r);

/// \brief `vsmc::uniform_real_distribution<double>`
void vsmc_uniform_real_rand(
    vsmc_rng rng, size_t n, double *r, double a, double b);

/// \brief `vsmc::weibull_distribution<double>`
void vsmc_weibull_rand(vsmc_rng rng, size_t n, double *r, double a, double b);

/// @} C_API_RNG_DISTRIBUITON

/// \addtogroup C_API_RNG_MKLRNG
/// @{

/// \brief `vsmc::mkl_brng`
int vsmc_mkl_brng(vSMCRNGType type);

/// @} C_API_RNG_MKLRNG

#ifdef __cplusplus
} // extern "C"
#endif

#endif // VSMC_RNG_RNG_H