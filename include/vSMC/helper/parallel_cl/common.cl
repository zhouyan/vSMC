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
void monitor_eval (size_t size, uint dim,
	__global state_type *buffer, __global state_type *weight,
	__global state_type *result)
{
    size_t i = get_global_id(0);

    if (i >= size)
	return;

    for (uint d = 0; d != dim; ++d)
	buffer[i * dim + d] *= weight[i];
    barrier(CLK_GLOBAL_MEM_FENCE);
    
    // TODO Not efficient sum
    if (i < dim) {
	state_type sum = 0;
	for (size_t j = 0; j != size; ++j)
	    sum += buffer[j * dim + i];
	result[i] = sum;
    }
}
