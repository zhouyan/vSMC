#ifndef V_SMC_RNG_RANDOM_HPP
#define V_SMC_RNG_RANDOM_HPP

#include <vSMC/internal/config.hpp>
#include <vSMC/rng/common.hpp>
#include <vSMC/rng/r123_engine.hpp>

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
using std::geometric_distribution;
using std::negative_binomial_distribution;

using std::exponential_distribution;
using std::extreme_value_distribution;
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

typedef boost::random::bernoulli_distribution<double> bernoulli_distribution;
using boost::random::binomial_distribution;
using boost::random::geometric_distribution;
using boost::random::negative_binomial_distribution;

using boost::random::exponential_distribution;
using boost::random::extreme_value_distribution;
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

#include <vSMC/rng/beta_distribution.hpp>
#include <vSMC/rng/laplace_distribution.hpp>

#endif // V_SMC_RNG_RANDOM_HPP
