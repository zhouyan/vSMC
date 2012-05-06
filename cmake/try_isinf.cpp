#include <cassert>
#include <limits>
#include <cmath>

#ifdef V_SMC_USE_STD_ISINF
using std::isinf;
#elif defined(V_SMC_USE_BOOST_ISINF)
#include <boost/math/special_functions/fpclassify.hpp>
using boost::math::isinf;
#endif

int main ()
{
    double x =  std::numeric_limits<double>::infinity();
    assert(isinf(x));

    return 0;
}
