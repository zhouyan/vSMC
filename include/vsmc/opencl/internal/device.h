//============================================================================
// include/vsmc/opencl/internal/device.h
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_DEVICE_H
#define VSMC_OPENCL_DEVICE_H

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
