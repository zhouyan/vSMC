#ifndef VSMC_HELPER_PARALLEL_CL_COMMON_CL
#define VSMC_HELPER_PARALLEL_CL_COMMON_CL

#if defined(cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#elif defined(cl_amd_fp64)
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#else
#define R123_USE_U01_DOUBLE 0
#endif

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>

__kernel
void copy (__global state_struct *state, __global uint *source)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    size_t from = source[to];
    if (to != from)
        state[to] = state[from];
}

#endif // VSMC_HELPER_PARALLEL_CL_COMMON_CL
