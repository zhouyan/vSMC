//============================================================================
// cmake/FindMKL.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <vector>
#include <mkl_vml.h>

int main ()
{
    const std::size_t N = 1000;
    std::vector<double> x(N);
    std::vector<double> y(N);
    for (std::size_t i = 0; i != N; ++i) {
        x[i] = static_cast<double>(i) / 1000.0;
        y[i] = static_cast<double>(i) / 1000.0;
    }
    ::vdExp(N, &x[0], &y[0]);
}
