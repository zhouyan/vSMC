#include <cassert>
#include <cmath>

#ifdef CXX_MATH_LGAMMA_C99_FOUND
#define CXX_MATH_LGAMMA lgamma
#endif

#ifdef CXX_MATH_LGAMMA_STD_FOUND
#define CXX_MATH_LGAMMA std::lgamma
#endif

#ifdef CXX_MATH_LGAMMA_BOOST_FOUND
#include <boost/math/special_functions/gamma.hpp>
#define CXX_MATH_LGAMMA boost::math::lgamma
#endif

int main ()
{
    assert(std::abs(CXX_MATH_LGAMMA(2)) < 1e-10);

    return 0;
}
