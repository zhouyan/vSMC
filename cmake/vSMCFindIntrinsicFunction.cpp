//============================================================================
// cmake/vSMCFindIntrinsicFunction.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <cassert>
#include <cstring>

#include <vsmc/internal/intrin.hpp>

#undef VSMC_INTRISIC_INT64

#if defined(VSMC_INTRINSIC_INT64_FOUND)
#define VSMC_INTRINSIC_INT64 __int64
#elif defined(VSMC_INTRINSIC_INT64_LONG_LONG_FOUND)
#define VSMC_INTRINSIC_INT64 long long
#else
#define VSMC_VSMC_INTRINSIC_INT64 long
#endif

int main ()
{

    unsigned tsc_aux = 0;
    unsigned VSMC_INTRINSIC_INT64 r = __rdtscp(&tsc_aux);

    VSMC_INTRINSIC_INT64 a = 0;
    VSMC_INTRINSIC_INT64 b = 1;
    _mm_stream_si64(&b, a);
    assert(b == 0);

    return 0;
}
