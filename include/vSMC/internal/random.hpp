#include <vSMC/internal/config.hpp>

#ifdef V_SMC_USE_STD_RANDOM

#include <random>
namespace vSMC { namespace internal {
using std::binomial_distribution;
using std::uniform_real_distribution;
} }

#else // V_SMC_USE_STD_RANDOM

#ifdef V_SMC_USE_TR1_RANDOM

#include <tr1/random>
namespace vSMC { namespace internal {
using std::tr1::binomial_distribution;
template <typename RealType = double>
class uniform_real_distribution : public std::tr1::uniform_real<RealType>
{
    public :

    explicit uniform_real_distribution (
            RealType min_arg = RealType(0),
            RealType max_arg = RealType(1)):
        std::tr1::uniform_real<RealType>(min_arg, max_arg) {}
}; // class uniform_real_distribution
} }

#else // V_SMC_USE_TR1_RANDOM

#include <boost/random.hpp>
#if BOOST_VERSION < 104700
namespace vSMC { namespace internal {
using boost::binomial_distribution;
using boost::uniform_real_distribution;
} }
#else // BOOST_VERSION < 104700
namespace vSMC { namespace internal {
using boost::random::binomial_distribution;
using boost::random::uniform_real_distribution;
} }
#endif // BOOST_VERSION < 104700

#endif // V_SMC_USE_TR1_RANDOM
 
#endif // V_SMC_USE_STD_RANDOM
