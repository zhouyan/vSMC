#include <cassert>
#include <cmath>

#ifdef VSMC_USE_STD_LGAMMA
using std::lgamma;
#elif defined(VSMC_USE_BOOST_LGAMMA)
#include <boost/math/special_functions/gamma.hpp>
using boost::math::lgamma;
#endif

int main ()
{
    assert(std::abs(lgamma(2)) < 1e-10);

    return 0;
}
