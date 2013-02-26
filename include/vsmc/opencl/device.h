#ifndef VSMC_OPENCL_DEVICE_H
#define VSMC_OPENCL_DEVICE_H

#if defined(__OPENCL_C_VERSION__) && __OPENCL_C_VERSION__ >= 120
#define VSMC_STATIC_INLINE static inline
#else
#define VSMC_STATIC_INLINE inline
#endif

__kernel
void copy (__global char *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    size_type from = copy_from[to];
    __global char *state_to = state + StateSize * to;
    __global char *state_from = state + StateSize * from;
    for (size_type i = 0; i != StateSize; ++i)
        state_to[i] = state_from[i];
}

#endif // VSMC_OPENCL_DEVICE_H
