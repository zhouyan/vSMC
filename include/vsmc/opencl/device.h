#ifndef VSMC_OPENCL_DEVICE_H
#define VSMC_OPENCL_DEVICE_H

#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#define VSMC_STATIC_INLINE static inline
#else
#define VSMC_STATIC_INLINE inline
#endif

__kernel
void copy (__global copy_state_struct *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    size_type from = copy_from[to];
    state[to] = state[from];
}

#endif // VSMC_OPENCL_DEVICE_H
