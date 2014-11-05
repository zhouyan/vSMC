//============================================================================
// cmake/FindAESNI.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <wmmintrin.h>
#include <iostream>

int main ()
{
    __m128i m = _mm_setzero_si128();
    char a[32];
    for (std::size_t i = 0; i != 32; ++i)
        a[i] = static_cast<char>(i);
    m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(a));
    m = _mm_aeskeygenassist_si128(m, 1);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(a), m);
    std::cout << a[0] << std::endl;
}
