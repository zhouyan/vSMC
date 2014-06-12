#include <cassert>
#include <cstring>

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <emmintrin.h>
#endif

int main ()
{
#if defined(INTRINSIC_INT64_FOUND)
    __int64 a = 0;
    __int64 b = 1;
#elif defined(INTRINSIC_INT64_LONG_LONG_FOUND)
    long long a = 0;
    long long b = 1;
#else
    long a = 0;
    long b = 1;
#endif

    _mm_stream_si64(&b, a);
    assert(b == 0);

    return 0;
}
