#include <cassert>
#include <limits>
#include <cmath>

#ifdef VSMC_USE_STD_ISFINITE
using std::isfintie;
#elif defined(VSMC_USE_BOOST_ISFINITE)
#include <boost/math/special_functions/fpclassify.hpp>
using boost::math::isfinite;
#endif

int main ()
{
    double x =  std::numeric_limits<double>::infinity();
    assert(!isfinite(x));

    return 0;
}
