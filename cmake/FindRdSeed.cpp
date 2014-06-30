//============================================================================
// cmake/FindRdSeed.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <immintrin.h>
#include <iostream>

int main ()
{
    unsigned short r;
    while (!(_rdseed16_step(&r)))
        ;
    std::cout << r << std::endl;
}
