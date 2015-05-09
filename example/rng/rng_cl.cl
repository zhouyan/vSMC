//============================================================================
// vSMC/example/rng/rng_cl.cl
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#if VSMC_HAS_OPENCL_DOUBLE
#if defined(cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#elif defined(cl_amd_fp64)
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#endif
typedef double fp_type;
#else
typedef float fp_type;
#endif

#include <vsmc/rngc/rngc.h>

__kernel void kernel_Philox2x32(ulong n, __global uint32_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_philox2x32 rng;
    vsmc_philox2x32_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 2 + 0] = vsmc_philox2x32_rand(&rng);
    buffer[i * 2 + 1] = vsmc_philox2x32_rand(&rng);
}

__kernel void kernel_Philox4x32(ulong n, __global uint32_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_philox4x32 rng;
    vsmc_philox4x32_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 4 + 0] = vsmc_philox4x32_rand(&rng);
    buffer[i * 4 + 1] = vsmc_philox4x32_rand(&rng);
    buffer[i * 4 + 2] = vsmc_philox4x32_rand(&rng);
    buffer[i * 4 + 3] = vsmc_philox4x32_rand(&rng);
}

__kernel void kernel_Threefry2x32(ulong n, __global uint32_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_threefry2x32 rng;
    vsmc_threefry2x32_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 2 + 0] = vsmc_threefry2x32_rand(&rng);
    buffer[i * 2 + 1] = vsmc_threefry2x32_rand(&rng);
}

__kernel void kernel_Threefry4x32(ulong n, __global uint32_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_threefry4x32 rng;
    vsmc_threefry4x32_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 4 + 0] = vsmc_threefry4x32_rand(&rng);
    buffer[i * 4 + 1] = vsmc_threefry4x32_rand(&rng);
    buffer[i * 4 + 2] = vsmc_threefry4x32_rand(&rng);
    buffer[i * 4 + 3] = vsmc_threefry4x32_rand(&rng);
}

__kernel void kernel_Threefry2x64(ulong n, __global uint64_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_threefry2x64 rng;
    vsmc_threefry2x64_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 2 + 0] = vsmc_threefry2x64_rand(&rng);
    buffer[i * 2 + 1] = vsmc_threefry2x64_rand(&rng);
}
__kernel void kernel_Threefry4x64(ulong n, __global uint64_t *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_threefry4x64 rng;
    vsmc_threefry4x64_init(&rng, 1);
    rng.ctr.v[0] = i;

    buffer[i * 4 + 0] = vsmc_threefry4x64_rand(&rng);
    buffer[i * 4 + 1] = vsmc_threefry4x64_rand(&rng);
    buffer[i * 4 + 2] = vsmc_threefry4x64_rand(&rng);
    buffer[i * 4 + 3] = vsmc_threefry4x64_rand(&rng);
}

__kernel void kernel_u01(ulong n, __global fp_type *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_rng rng;
    vsmc_rng_init(&rng, i);

    buffer[i] = vsmc_u01_open_closed_32(vsmc_rng_rand(&rng));
}

__kernel void kernel_normal01(ulong n, __global fp_type *buffer)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_rng rng;
    vsmc_rng_init(&rng, i);

    vsmc_normal01 dist;
    vsmc_normal01_init(&dist, &rng);

    buffer[i] = vsmc_normal01_rand(&dist, &rng);
}

__kernel void kernel_gammak1(ulong n, __global fp_type *buffer, fp_type shape)
{
    ulong i = get_global_id(0);
    if (i >= n)
        return;

    vsmc_rng rng;
    vsmc_rng_init(&rng, i);

    vsmc_gammak1 dist;
    vsmc_gammak1_init(&dist, &rng, shape);

    buffer[i] = vsmc_gammak1_rand(&dist, &rng);
}
