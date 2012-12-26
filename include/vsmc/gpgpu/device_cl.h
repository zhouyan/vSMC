#ifndef VSMC_GPGPU_DEVICE_CL_H
#define VSMC_GPGPU_DEVICE_CL_H

#include <vsmc/cl/config.h>

__kernel
void copy (__global state_struct *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    state[to] = state[copy_from[to]];
}

#endif // VSMC_GPGPU_DEVICE_CL_H
