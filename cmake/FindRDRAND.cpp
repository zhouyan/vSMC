//============================================================================
// cmake/FindRDRAND.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <stdint.h>
#endif

int main ()
{
#ifdef _MSC_VER
    unsigned short r16;
    unsigned r32;
    unsigned __int64 r64;
    _rdrand16_step(&r16);
    _rdrand32_step(&r32);
    _rdrand64_step(&r64);
#else
    unsigned char cf = 0;
    uint16_t r16;
    uint32_t r32;
    uint64_t r64;
    __asm__ volatile
        (
         "rdrand %0\\n\\t"
         "rdrand %1\\n\\t"
         "rdrand %2\\n\\t"
         "setcb %3\\n"
         : "=r" (r16), "=r" (r32), "=r" (r64), "=qm" (cf)
         );
#endif

    return 0;
}
