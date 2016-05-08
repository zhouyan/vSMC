//============================================================================
// vSMC/lib/src/rng/rng.cpp
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

#include "rand.cpp"
#include "rand_64.cpp"
#include "rand_bernoulli.cpp"
#include "rand_binomial.cpp"
#include "rand_binomial_64.cpp"
#include "rand_discrete.cpp"
#include "rand_discrete_64.cpp"
#include "rand_geometric.cpp"
#include "rand_geometric_64.cpp"
#include "rand_negative_binomial.cpp"
#include "rand_negative_binomial_64.cpp"
#include "rand_poisson.cpp"
#include "rand_poisson_64.cpp"
#include "rand_uniform_int.cpp"
#include "rand_uniform_int_64.cpp"

#include "rand_arcsine.cpp"
#include "rand_beta.cpp"
#include "rand_cauchy.cpp"
#include "rand_chi_squared.cpp"
#include "rand_exponential.cpp"
#include "rand_extreme_value.cpp"
#include "rand_fisher_f.cpp"
#include "rand_gamma.cpp"
#include "rand_laplace.cpp"
#include "rand_levy.cpp"
#include "rand_logistic.cpp"
#include "rand_lognormal.cpp"
#include "rand_normal.cpp"
#include "rand_normal_mv.cpp"
#include "rand_pareto.cpp"
#include "rand_rayleigh.cpp"
#include "rand_student_t.cpp"
#include "rand_u01.cpp"
#include "rand_u01_cc.cpp"
#include "rand_u01_co.cpp"
#include "rand_u01_oc.cpp"
#include "rand_u01_oo.cpp"
#include "rand_uniform_real.cpp"
#include "rand_weibull.cpp"

#include "rng_assign.cpp"
#include "rng_delete.cpp"
#include "rng_discard.cpp"
#include "rng_is_eq.cpp"
#include "rng_is_neq.cpp"
#include "rng_load.cpp"
#include "rng_load_f.cpp"
#include "rng_new.cpp"
#include "rng_save.cpp"
#include "rng_save_f.cpp"
#include "rng_seed.cpp"
#include "rng_type.cpp"

#include "seed.cpp"

#include "u01_rand_sorted.cpp"
#include "u01_rand_stratified.cpp"
#include "u01_rand_systematic.cpp"
#include "u01_trans.cpp"

#if VSMC_HAS_MKL
#include "mkl_brng.cpp"
#endif
