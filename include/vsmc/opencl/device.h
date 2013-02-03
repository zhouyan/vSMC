#ifndef VSMC_OPENCL_DEVICE_H
#define VSMC_OPENCL_DEVICE_H

#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#define VSMC_STATIC_INLINE static inline
#else
#define VSMC_STATIC_INLINE inline
#endif

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

#endif // VSMC_OPENCL_DEVICE_H
