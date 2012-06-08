#include <cassert>
#include <cmath>

#ifdef VSMC_USE_STD_ERF
using std::erf;
#elif defined(VSMC_USE_BOOST_ERF)
#include <boost/math/special_functions/erf.hpp>
using boost::math::erf;
#endif

int main ()
{
    assert(std::abs(erf(0)) < 1e-10);

    return 0;
}
