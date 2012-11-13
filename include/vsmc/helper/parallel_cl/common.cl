#ifndef VSMC_HELPER_PARALLEL_CL_COMMON_CL
#define VSMC_HELPER_PARALLEL_CL_COMMON_CL

#if VSMC_USE_RANDOM123
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#endif

__kernel
void copy (__global state_struct *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    state[to] = state[copy_from[to]];
}

#endif // VSMC_HELPER_PARALLEL_CL_COMMON_CL
