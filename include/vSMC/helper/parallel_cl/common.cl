#ifndef V_SMC_HELPER_PARALLEL_CL_COMMON_CL
#define V_SMC_HELPER_PARALLEL_CL_COMMON_CL

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>

__kernel
void copy (__global state_struct *state, __global uint *source)
{
    ulong to = get_global_id(0);

    if (to >= Size)
        return;

    size_t from = source[to];
    state[to] = state[from];
}

#endif // V_SMC_HELPER_PARALLEL_CL_COMMON_CL
