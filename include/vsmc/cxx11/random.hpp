//============================================================================
// vSMC/include/vsmc/cxx11/random.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_CXX11_RANDOM_HPP
#define VSMC_CXX11_RANDOM_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_RANDOM

#include <random>

namespace vsmc { namespace cxx11 {

using std::linear_congruential_engine;
using std::mersenne_twister_engine;
using std::subtract_with_carry_engine;
using std::discard_block_engine;
using std::independent_bits_engine;
using std::shuffle_order_engine;
using std::minstd_rand0;
using std::minstd_rand;
using std::mt19937;
using std::mt19937_64;
using std::ranlux24_base;
using std::ranlux48_base;
using std::ranlux24;
using std::ranlux48;
using std::knuth_b;
using std::default_random_engine;
// using std::random_device;
using std::seed_seq;
using std::generate_canonical;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::bernoulli_distribution;
using std::binomial_distribution;
using std::geometric_distribution;
using std::negative_binomial_distribution;
using std::poisson_distribution;
using std::exponential_distribution;
using std::gamma_distribution;
using std::weibull_distribution;
using std::extreme_value_distribution;
using std::normal_distribution;
using std::lognormal_distribution;
using std::chi_squared_distribution;
using std::cauchy_distribution;
using std::fisher_f_distribution;
using std::student_t_distribution;
using std::discrete_distribution;
using std::piecewise_constant_distribution;
using std::piecewise_linear_distribution;

} } // namespace vsmc::cxx11

#else // VSMC_HAS_CXX11LIB_RANDOM

#include <boost/random.hpp>
#include <boost/cstdint.hpp>

namespace vsmc { namespace cxx11 {

using ::boost::random::linear_congruential_engine;
using ::boost::random::mersenne_twister_engine;
using ::boost::random::subtract_with_carry_engine;
using ::boost::random::discard_block_engine;
using ::boost::random::independent_bits_engine;
using ::boost::random::shuffle_order_engine;
using ::boost::random::minstd_rand0;
using ::boost::random::minstd_rand;
using ::boost::random::mt19937;
using ::boost::random::mt19937_64;
// using ::boost::random::ranlux24_base;
typedef ::boost::random::subtract_with_carry_engine<
    ::boost::uint32_t, 24, 10, 24> ranlux24_base;
// using ::boost::random::ranlux48_base;
typedef ::boost::random::subtract_with_carry_engine<
    ::boost::uint64_t, 48, 5, 12> ranlux48_base;
using ::boost::random::ranlux24;
using ::boost::random::ranlux48;
using ::boost::random::knuth_b;
// using ::boost::random::default_random_engine;
typedef ::boost::random::mt19937 default_random_engine;
// using ::boost::random::random_device;
using ::boost::random::seed_seq;
using ::boost::random::generate_canonical;
using ::boost::random::uniform_int_distribution;
using ::boost::random::uniform_real_distribution;
// using ::boost::random::bernoulli_distribution;
typedef ::boost::random::bernoulli_distribution<double> bernoulli_distribution;
using ::boost::random::binomial_distribution;
using ::boost::random::geometric_distribution;
using ::boost::random::negative_binomial_distribution;
using ::boost::random::poisson_distribution;
using ::boost::random::exponential_distribution;
using ::boost::random::gamma_distribution;
using ::boost::random::weibull_distribution;
using ::boost::random::extreme_value_distribution;
using ::boost::random::normal_distribution;
using ::boost::random::lognormal_distribution;
using ::boost::random::chi_squared_distribution;
using ::boost::random::cauchy_distribution;
using ::boost::random::fisher_f_distribution;
using ::boost::random::student_t_distribution;
using ::boost::random::discrete_distribution;
using ::boost::random::piecewise_constant_distribution;
using ::boost::random::piecewise_linear_distribution;

} } //namespace vsmc::cxx11

#endif // VSMC_HAS_CXX11LIB_RANDOM

#endif // VSMC_CXX11_RANDOM_HPP
