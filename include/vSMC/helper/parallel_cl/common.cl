#include <vSMC/helper/parallel_cl/rng.h>

__kernel
void copy (size_t size, __global state_struct *state, __global uint *source)
{
    size_t to = get_global_id(0);
    
    if (to >= size)
        return;

    size_t from = source[to];
    state[to] = state[from];
}

__kernel
void monitor_eval (size_t size, uint iter, uint dim,
	__global state_type *buffer, __global state_type *weight,
	__global state_type *result)
{
}
