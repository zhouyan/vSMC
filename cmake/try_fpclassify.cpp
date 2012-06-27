#include <cassert>
#include <limits>
#include <cmath>

#ifdef VSMC_USE_STD_FPCLASSIFY
using std::isinf;
using std::isfinite;
#elif defined(VSMC_USE_BOOST_FPCLASSIFY)
#include <boost/math/special_functions/fpclassify.hpp>
using boost::math::isinf;
using boost::math::isfinite;
#endif

int main ()
{
    double x =  std::numeric_limits<double>::infinity();
    assert(isinf(x));
    assert(!isfinite(x));

    return 0;
}
