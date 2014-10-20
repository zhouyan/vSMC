//============================================================================
// cmake/FindRDTSCP.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <cassert>

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

int main ()
{
    unsigned tsc = 0;
    __rdtscp(&tsc);

    return 0;
}
