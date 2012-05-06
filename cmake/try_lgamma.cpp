#include <cassert>
#include <cmath>

#ifdef V_SMC_USE_STD_LGAMMA
using std::lgamma;
#elif defined(V_SMC_USE_BOOST_LGAMMA)
#include <boost/math/special_functions/gamma.hpp>
using boost::math::lgamma;
#endif

int main ()
{
    assert(std::abs(lgamma(2)) < 1e-10);

    return 0;
}
