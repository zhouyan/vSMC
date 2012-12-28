#ifndef VSMC_GPGPU_DEVICE_CL_H
#define VSMC_GPGPU_DEVICE_CL_H

#include <vsmc/gpgpu/config_cl.h>

__kernel
void copy (__global state_type *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    __global state_type *state_from = state + copy_from[to] * Dim;
    __global state_type *state_to   = state + to * Dim;
    for (uint i = 0; i != Dim; ++i)
        state_to[i] = state_from[i];
}

#endif // VSMC_GPGPU_DEVICE_CL_H
