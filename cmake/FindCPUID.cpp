//============================================================================
// cmake/FindCPUID.cpp
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
    int CPUInfo[4] = {0};
    int InfoType[2] = {0};
    __cpuidex(CPUInfo, InfoType[0], InfoType[1]);
    assert(CPUInfo[0] > 0);
#else
    unsigned eax = 0;
    unsigned ebx = 0;
    unsigned ecx = 0;
    unsigned edx = 0;
    __asm__(
            "cpuid\\n"
            : "=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
            :  "a" (eax),  "c" (ecx)
           );
    assert(eax > 0);
#endif

    return 0;
}
