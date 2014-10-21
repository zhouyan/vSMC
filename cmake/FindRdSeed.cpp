//============================================================================
// cmake/FindRdSeed.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifdef _MSC_VER
#include <intrin.h>
#endif

int main ()
{
    unsigned short r;
#ifdef _MSC_VER
    _rdseed16_step(&r);
#else
    unsigned char cf = 0;
    __asm__ volatile(
            "rdseedw %0; setcb %1\\n"
            : "=r" (r), "=qm" (cf));
#endif

    return 0;
}

