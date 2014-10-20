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
#endif

int main ()
{
#ifdef _MSC_VER
        __rdtscp(aux)
#else // _MSC_VER
        unsigned eax = 0x00;
        unsigned edx = 0x00;
        unsigned ecx = 0x00;
        __asm__(
                "rdtscp\\n"
                : "=a" (eax), "=d" (edx), "=c" (ecx)
               );
#endif // _MSC_VER

    return 0;
}
