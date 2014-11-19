//============================================================================
// vSMC/vSMCExample/rng/rng_cl.cl
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#include <vsmc/rng/normal01.h>
#include <vsmc/rng/gammak1.h>
#include <vsmc/rng/u01.h>

#define VSMC_DEFINE_R123_KERNEL(CBRNG, N, W) \
    __kernel void CBRNG##_##N##x##W##_ker (ulong n,                          \
            __global uint *output_ui_0,                                      \
            __global uint *output_ui_1,                                      \
            __global uint *output_ui_2,                                      \
            __global uint *output_ui_3)                                      \
    {                                                                        \
        ulong i = get_global_id(0);                                          \
        if (i >= n) return;                                                  \
        CBRNG##N##x##W##_ctr_t ctr = {{}};                                   \
        CBRNG##N##x##W##_key_t key = {{}};                                   \
        CBRNG##N##x##W##_ukey_t ukey = {{}};                                 \
        ukey.v[0] = i;                                                       \
        key = CBRNG##N##x##W##keyinit(ukey);                                 \
        CBRNG##N##x##W##_ctr_t r = CBRNG##N##x##W(ctr, key);                 \
        output_ui_0[i] = r.v[0];                                             \
        output_ui_1[i] = r.v[1];                                             \
        output_ui_2[i] = (N == 4 ? r.v[2] : 0);                              \
        output_ui_3[i] = (N == 4 ? r.v[3] : 0);                              \
    }

#define VSMC_DEFINE_CBURNG_KERNEL(RNG, N, W) \
    __kernel void cburng_##RNG##_##N##x##W##_ker (ulong n,                   \
            __global uint *output_ui_0, __global uint *output_ui_1,          \
            __global uint *output_ui_2, __global uint *output_ui_3)          \
    {                                                                        \
        ulong i = get_global_id(0);                                          \
        if (i >= n) return;                                                  \
        RNG##N##x##W##_rng_t rng;                                            \
        RNG##N##x##W##_init(&rng, i);                                        \
        output_ui_0[i] = RNG##N##x##W##_rand(&rng);                          \
        output_ui_1[i] = RNG##N##x##W##_rand(&rng);                          \
        output_ui_2[i] = (N == 4 ? RNG##N##x##W##_rand(&rng) : 0);           \
        output_ui_3[i] = (N == 4 ? RNG##N##x##W##_rand(&rng) : 0);           \
    }

#define VSMC_DEFINE_U01_KERNEL(N, W) \
    __kernel void u01_##N##x##W##_ker (ulong n, __global fp_type *output)    \
    {                                                                        \
        ulong i = get_global_id(0);                                          \
        if (i >= n) return ;                                                 \
        cburng##N##x##W##_rng_t rng;                                         \
        cburng##N##x##W##_init(&rng, i);                                     \
        output[i] = U01_OPEN_CLOSED_##W(cburng##N##x##W##_rand(&rng));       \
    }

#define VSMC_DEFINE_NORMAL01_KERNEL(N, W) \
    __kernel void normal01_##N##x##W##_ker (ulong n, __global fp_type *output)\
    {                                                                        \
        ulong i = get_global_id(0);                                          \
        if (i >= n) return ;                                                 \
        cburng##N##x##W##_rng_t rng;                                         \
        cburng##N##x##W##_init(&rng, i);                                     \
        NORMAL01_##N##x##W dist;                                             \
        NORMAL01_##N##x##W##_INIT(&dist, &rng);                              \
        output[i] = NORMAL01_##N##x##W##_RAND(&dist, &rng);                  \
    }

#define VSMC_DEFINE_GAMMAK1_KERNEL(N, W) \
    __kernel void gammak1_##N##x##W##_ker (ulong n, __global fp_type *output,\
            fp_type shape)                                                   \
    {                                                                        \
        ulong i = get_global_id(0);                                          \
        if (i >= n) return ;                                                 \
        cburng##N##x##W##_rng_t rng;                                         \
        cburng##N##x##W##_init(&rng, i);                                     \
        GAMMAK1_##N##x##W dist;                                              \
        GAMMAK1_##N##x##W##_INIT(&dist, &rng, shape);                        \
        output[i] = GAMMAK1_##N##x##W##_RAND(&dist, &rng);                   \
    }

VSMC_DEFINE_R123_KERNEL(threefry, 2, 32)
VSMC_DEFINE_R123_KERNEL(threefry, 4, 32)

VSMC_DEFINE_R123_KERNEL(philox, 2, 32)
VSMC_DEFINE_R123_KERNEL(philox, 4, 32)

VSMC_DEFINE_CBURNG_KERNEL(threefry, 2, 32)
VSMC_DEFINE_CBURNG_KERNEL(threefry, 4, 32)
VSMC_DEFINE_CBURNG_KERNEL(philox, 2, 32)
VSMC_DEFINE_CBURNG_KERNEL(philox, 4, 32)

VSMC_DEFINE_U01_KERNEL(2, 32)
VSMC_DEFINE_U01_KERNEL(4, 32)

VSMC_DEFINE_NORMAL01_KERNEL(2, 32)
VSMC_DEFINE_NORMAL01_KERNEL(4, 32)

VSMC_DEFINE_GAMMAK1_KERNEL(2, 32)
VSMC_DEFINE_GAMMAK1_KERNEL(4, 32)
