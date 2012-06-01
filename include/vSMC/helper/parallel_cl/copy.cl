__kernel
void vSMC_copy (__global state_struct *state, __global uint *from)
{
    size_t i = get_global_id(0);

    state[i] = state[from[i]];
}
