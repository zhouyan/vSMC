//============================================================================
// cmake/vSMCFindCXX11Math.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <vsmc/cxx11/cmath.hpp>
#include <cmath>
#include <cassert>

#define ASSERT_CLOSE(expr, val) assert(std::fabs((expr) - (val)) < 1e-6)
#define ASSERT_EQUAL(expr, val) assert((expr) == (val))

int main ()
{
    ASSERT_CLOSE(vsmc::cxx11::exp2(1.), 2.);
    ASSERT_CLOSE(vsmc::cxx11::log2(1.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::expm1(0.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::log1p(0.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::cbrt(1.), 1.);
    ASSERT_CLOSE(vsmc::cxx11::hypot(1., 1.), std::sqrt(2.));
    ASSERT_CLOSE(vsmc::cxx11::asinh(0.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::acosh(1.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::atanh(0.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::erf(0.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::erfc(0.), 1.);
    ASSERT_CLOSE(vsmc::cxx11::lgamma(1.), 0.);
    ASSERT_CLOSE(vsmc::cxx11::tgamma(1.), 1.);

    return 0;
}
