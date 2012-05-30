#ifndef V_SMC_INTERNAL_RANDOM_HPP
#define V_SMC_INTERNAL_RANDOM_HPP

// #include <Random123/aes.h>
// #include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER

#include <Random123/conventional/Engine.hpp>

#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

#include <vSMC/internal/config.hpp>

/// The parallel RNG (based on Random123) seed, unsigned
#ifndef V_SMC_CRNG_SEED
#define V_SMC_CRNG_SEED 0xdeadbeefU
#endif // V_SMC_CRNG_SEED

/// The parallel RNG (based on Random123) type, philox or threefry
#ifndef V_SMC_CRNG_TYPE
#define V_SMC_CRNG_TYPE r123::Threefry4x64
#endif // V_SMC_CRNG_TYPE

#ifdef V_SMC_USE_STD_RANDOM
#include <random>
#else // V_SMC_USE_STD_RANDOM
#include <boost/random.hpp>
#endif // V_SMC_USE_STD_RANDOM

namespace vSMC { namespace rng {

#ifdef V_SMC_USE_STD_RANDOM

using std::uniform_int_distribution;
using std::uniform_real_distribution;

using std::bernoulli_distribution;
using std::binomial_distribution;
using std::extreme_value_distribution;
using std::geometric_distribution;
using std::negative_binomial_distribution;

using std::exponential_distribution;
using std::gamma_distribution;
using std::poisson_distribution;
using std::weibull_distribution;

using std::cauchy_distribution;
using std::chi_squared_distribution;
using std::fisher_f_distribution;
using std::lognormal_distribution;
using std::normal_distribution;
using std::student_t_distribution;

using std::discrete_distribution;
using std::piecewise_constant_distribution;
using std::piecewise_linear_distribution;

#else // V_SMC_USE_STD_RANDOM

using boost::random::uniform_int_distribution;
using boost::random::uniform_real_distribution;

typedef boost::random::bernoulli_distribution<> bernoulli_distribution;
using boost::random::binomial_distribution;
using boost::random::extreme_value_distribution;
using boost::random::geometric_distribution;
using boost::random::negative_binomial_distribution;

using boost::random::exponential_distribution;
using boost::random::gamma_distribution;
using boost::random::poisson_distribution;
using boost::random::weibull_distribution;

using boost::random::cauchy_distribution;
using boost::random::chi_squared_distribution;
using boost::random::fisher_f_distribution;
using boost::random::lognormal_distribution;
using boost::random::normal_distribution;
using boost::random::student_t_distribution;

using boost::random::discrete_distribution;
using boost::random::piecewise_constant_distribution;
using boost::random::piecewise_linear_distribution;

#endif // V_SMC_USE_STD_RANDOM

} } // namespace vSMC::rng

// #include <vSMC/rng/beta_distribution.hpp>
// #include <vSMC/rng/laplace_distribution.hpp>

#endif // V_SMC_INTERNAL_RANDOM_HPP
