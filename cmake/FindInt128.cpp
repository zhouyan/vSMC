//============================================================================
// cmake/FindInt128.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <cassert>

int main ()
{
    unsigned a = 0;
    unsigned b = 0;
    INT128 c = static_cast<INT128>(a) * static_cast<INT128>(b);
    assert(static_cast<unsigned>(c) == 0);
    assert(static_cast<unsigned>(c>>64) == 0);

    return 0;
}
