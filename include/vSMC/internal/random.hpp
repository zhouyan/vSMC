#include <vSMC/internal/config.hpp>

#ifdef V_SMC_USE_STD_RANDOM

#include <random>

namespace vSMC { namespace internal {
using std::binomial_distribution;
using std::uniform_real_distribution;
} }

#else // V_SMC_USE_STD_RANDOM

#include <boost/random/binomial_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
namespace vSMC { namespace internal {
#if BOOST_VERSION < 104700
using boost::binomial_distribution;
using boost::uniform_real_distribution;
#else // BOOST_VERSION < 104700
using boost::random::binomial_distribution;
using boost::random::uniform_real_distribution;
#endif // BOOST_VERSION < 104700
} }
 
#endif // V_SMC_USE_STD_RANDOM
