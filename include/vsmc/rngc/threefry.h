//============================================================================
// vSMC/include/vsmc/rngc/threefry.h
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_RNGC_THREEFRY_H
#define VSMC_RNGC_THREEFRY_H

#include <vsmc/internal/config.h>

/// \brief Threefry2x32 counter type
/// \ingroup ThreefryC
typedef struct {
    uint32_t v[2];
} vsmc_threefry2x32_ctr_t;

/// \brief Threefry4x32 counter type
/// \ingroup ThreefryC
typedef struct {
    uint32_t v[4];
} vsmc_threefry4x32_ctr_t;

/// \brief Threefry2x64 counter type
/// \ingroup ThreefryC
typedef struct {
    uint64_t v[2];
} vsmc_threefry2x64_ctr_t;

/// \brief Threefry4x64 counter type
/// \ingroup ThreefryC
typedef struct {
    uint64_t v[4];
} vsmc_threefry4x64_ctr_t;

/// \brief Threefry2x32 key type
/// \ingroup ThreefryC
typedef struct {
    uint32_t v[2];
} vsmc_threefry2x32_key_t;

/// \brief Threefry4x32 key type
/// \ingroup ThreefryC
typedef struct {
    uint32_t v[4];
} vsmc_threefry4x32_key_t;

/// \brief Threefry2x64 key type
/// \ingroup ThreefryC
typedef struct {
    uint64_t v[2];
} vsmc_threefry2x64_key_t;

/// \brief Threefry4x64 key type
/// \ingroup ThreefryC
typedef struct {
    uint64_t v[4];
} vsmc_threefry4x64_key_t;

typedef struct {
    uint32_t v[3];
} vsmc_threefry2x32_par_t;

typedef struct {
    uint32_t v[5];
} vsmc_threefry4x32_par_t;

typedef struct {
    uint64_t v[3];
} vsmc_threefry2x64_par_t;

typedef struct {
    uint64_t v[5];
} vsmc_threefry4x64_par_t;

/// \brief Threefry2x32 RNG state structure
/// \ingroup ThreefryC
typedef struct {
    vsmc_threefry2x32_ctr_t state;
    vsmc_threefry2x32_ctr_t ctr;
    vsmc_threefry2x32_key_t key;
    uint32_t index;
} vsmc_threefry2x32;

/// \brief Threefry4x32 RNG state structure
/// \ingroup ThreefryC
typedef struct {
    vsmc_threefry4x32_ctr_t state;
    vsmc_threefry4x32_ctr_t ctr;
    vsmc_threefry4x32_key_t key;
    uint32_t index;
} vsmc_threefry4x32;

/// \brief Threefry2x64 RNG state structure
/// \ingroup ThreefryC
typedef struct {
    vsmc_threefry2x64_ctr_t state;
    vsmc_threefry2x64_ctr_t ctr;
    vsmc_threefry2x64_key_t key;
    uint64_t index;
} vsmc_threefry2x64;

/// \brief Threefry4x64 RNG state structure
/// \ingroup ThreefryC
typedef struct {
    vsmc_threefry4x64_ctr_t state;
    vsmc_threefry4x64_ctr_t ctr;
    vsmc_threefry4x64_key_t key;
    uint64_t index;
} vsmc_threefry4x64;

static inline void vsmc_threefry2x32_inc(vsmc_threefry2x32_ctr_t *ctr)
{
    if (++ctr->v[0] != 0)
        return;
    if (++ctr->v[1] != 0)
        return;
}

static inline void vsmc_threefry4x32_inc(vsmc_threefry4x32_ctr_t *ctr)
{
    if (++ctr->v[0] != 0)
        return;
    if (++ctr->v[1] != 0)
        return;
    if (++ctr->v[2] != 0)
        return;
    if (++ctr->v[3] != 0)
        return;
}

static inline void vsmc_threefry2x64_inc(vsmc_threefry2x64_ctr_t *ctr)
{
    if (++ctr->v[0] != 0)
        return;
    if (++ctr->v[1] != 0)
        return;
}

static inline void vsmc_threefry4x64_inc(vsmc_threefry4x64_ctr_t *ctr)
{
    if (++ctr->v[0] != 0)
        return;
    if (++ctr->v[1] != 0)
        return;
    if (++ctr->v[2] != 0)
        return;
    if (++ctr->v[3] != 0)
        return;
}

static inline void vsmc_threefry2x32_initpar(
    const vsmc_threefry2x32_key_t *key, vsmc_threefry2x32_par_t *par)
{
    par->v[0] = key->v[0];
    par->v[1] = key->v[1];

    par->v[2] = UINT32_C(0x1BD11BDA);
    par->v[2] ^= par->v[0];
    par->v[2] ^= par->v[1];
}

static inline void vsmc_threefry4x32_initpar(
    const vsmc_threefry4x32_key_t *key, vsmc_threefry4x32_par_t *par)
{
    par->v[0] = key->v[0];
    par->v[1] = key->v[1];
    par->v[2] = key->v[2];
    par->v[3] = key->v[3];

    par->v[4] = UINT32_C(0x1BD11BDA);
    par->v[4] ^= par->v[0];
    par->v[4] ^= par->v[1];
    par->v[4] ^= par->v[2];
    par->v[4] ^= par->v[3];
}

static inline void vsmc_threefry2x64_initpar(
    const vsmc_threefry2x64_key_t *key, vsmc_threefry2x64_par_t *par)
{
    par->v[0] = key->v[0];
    par->v[1] = key->v[1];

    par->v[2] = UINT64_C(0x1BD11BDAA9FC1A22);
    par->v[2] ^= par->v[0];
    par->v[2] ^= par->v[1];
}

static inline void vsmc_threefry4x64_initpar(
    const vsmc_threefry4x64_key_t *key, vsmc_threefry4x64_par_t *par)
{
    par->v[0] = key->v[0];
    par->v[1] = key->v[1];
    par->v[2] = key->v[2];
    par->v[3] = key->v[3];

    par->v[4] = UINT64_C(0x1BD11BDAA9FC1A22);
    par->v[4] ^= par->v[0];
    par->v[4] ^= par->v[1];
    par->v[4] ^= par->v[2];
    par->v[4] ^= par->v[3];
}

static inline void vsmc_threefry2x32_rotate(
    vsmc_threefry2x32_ctr_t *state, uint32_t r)
{
    state->v[0] += state->v[1];
    state->v[1] = ((state->v[1]) << r) | ((state->v[1]) >> (32 - r));
    state->v[1] ^= state->v[0];
}

static inline void vsmc_threefry4x32_rotate(
    vsmc_threefry4x32_ctr_t *state, uint32_t r0, uint32_t r2, int i0, int i2)
{
    state->v[0] += state->v[i0];
    state->v[i0] = ((state->v[i0]) << r0) | ((state->v[i0]) >> (32 - r0));
    state->v[i0] ^= state->v[0];

    state->v[2] += state->v[i2];
    state->v[i2] = ((state->v[i2]) << r2) | ((state->v[i2]) >> (32 - r2));
    state->v[i2] ^= state->v[2];
}

static inline void vsmc_threefry2x64_rotate(
    vsmc_threefry2x64_ctr_t *state, uint64_t r)
{
    state->v[0] += state->v[1];
    state->v[1] = ((state->v[1]) << r) | ((state->v[1]) >> (64 - r));
    state->v[1] ^= state->v[0];
}

static inline void vsmc_threefry4x64_rotate(
    vsmc_threefry4x64_ctr_t *state, uint64_t r0, uint64_t r2, int i0, int i2)
{
    state->v[0] += state->v[i0];
    state->v[i0] = ((state->v[i0]) << r0) | ((state->v[i0]) >> (64 - r0));
    state->v[i0] ^= state->v[0];

    state->v[2] += state->v[i2];
    state->v[i2] = ((state->v[i2]) << r2) | ((state->v[i2]) >> (64 - r2));
    state->v[i2] ^= state->v[2];
}

static inline void vsmc_threefry2x32_insertkey(vsmc_threefry2x32_ctr_t *state,
    const vsmc_threefry2x32_par_t *par, uint32_t inc, int i0, int i1)
{
    state->v[0] += par->v[i0];
    state->v[1] += par->v[i1];
    state->v[1] += inc;
}

static inline void vsmc_threefry4x32_insertkey(vsmc_threefry4x32_ctr_t *state,
    const vsmc_threefry4x32_par_t *par, uint32_t inc, int i0, int i1, int i2,
    int i3)
{
    state->v[0] += par->v[i0];
    state->v[1] += par->v[i1];
    state->v[2] += par->v[i2];
    state->v[3] += par->v[i3];
    state->v[3] += inc;
}

static inline void vsmc_threefry2x64_insertkey(vsmc_threefry2x64_ctr_t *state,
    const vsmc_threefry2x64_par_t *par, uint64_t inc, int i0, int i1)
{
    state->v[0] += par->v[i0];
    state->v[1] += par->v[i1];
    state->v[1] += inc;
}

static inline void vsmc_threefry4x64_insertkey(vsmc_threefry4x64_ctr_t *state,
    const vsmc_threefry4x64_par_t *par, uint64_t inc, int i0, int i1, int i2,
    int i3)
{
    state->v[0] += par->v[i0];
    state->v[1] += par->v[i1];
    state->v[2] += par->v[i2];
    state->v[3] += par->v[i3];
    state->v[3] += inc;
}

/// \brief Generate Threefry2x32 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry2x32_gen(const vsmc_threefry2x32_ctr_t *ctr,
    const vsmc_threefry2x32_key_t *key, vsmc_threefry2x32_ctr_t *state)
{
    *state = *ctr;
    vsmc_threefry2x32_par_t par;
    vsmc_threefry2x32_initpar(key, &par);

    vsmc_threefry2x32_insertkey(state, &par, 0, 0, 1); // N = 0
    vsmc_threefry2x32_rotate(state, 13);               // N = 1
    vsmc_threefry2x32_rotate(state, 15);               // N = 2
    vsmc_threefry2x32_rotate(state, 26);               // N = 3
    vsmc_threefry2x32_rotate(state, 6);                // N = 4
    vsmc_threefry2x32_insertkey(state, &par, 1, 1, 2); // N = 4
    vsmc_threefry2x32_rotate(state, 17);               // N = 5
    vsmc_threefry2x32_rotate(state, 29);               // N = 6
    vsmc_threefry2x32_rotate(state, 16);               // N = 7
    vsmc_threefry2x32_rotate(state, 24);               // N = 8
    vsmc_threefry2x32_insertkey(state, &par, 2, 2, 0); // N = 8
    vsmc_threefry2x32_rotate(state, 13);               // N = 9
    vsmc_threefry2x32_rotate(state, 15);               // N = 10
    vsmc_threefry2x32_rotate(state, 26);               // N = 11
    vsmc_threefry2x32_rotate(state, 6);                // N = 12
    vsmc_threefry2x32_insertkey(state, &par, 3, 0, 1); // N = 12
    vsmc_threefry2x32_rotate(state, 17);               // N = 13
    vsmc_threefry2x32_rotate(state, 29);               // N = 14
    vsmc_threefry2x32_rotate(state, 16);               // N = 15
    vsmc_threefry2x32_rotate(state, 24);               // N = 16
    vsmc_threefry2x32_insertkey(state, &par, 4, 1, 2); // N = 16
    vsmc_threefry2x32_rotate(state, 13);               // N = 17
    vsmc_threefry2x32_rotate(state, 15);               // N = 18
    vsmc_threefry2x32_rotate(state, 26);               // N = 19
    vsmc_threefry2x32_rotate(state, 6);                // N = 20
    vsmc_threefry2x32_insertkey(state, &par, 5, 2, 0); // N = 20
}

/// \brief Generate Threefry4x32 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry4x32_gen(const vsmc_threefry4x32_ctr_t *ctr,
    const vsmc_threefry4x32_key_t *key, vsmc_threefry4x32_ctr_t *state)
{
    *state = *ctr;
    vsmc_threefry4x32_par_t par;
    vsmc_threefry4x32_initpar(key, &par);

    vsmc_threefry4x32_insertkey(state, &par, 0, 0, 1, 2, 3); // N = 0
    vsmc_threefry4x32_rotate(state, 10, 26, 1, 3);           // N = 1
    vsmc_threefry4x32_rotate(state, 11, 21, 3, 1);           // N = 2
    vsmc_threefry4x32_rotate(state, 13, 27, 1, 3);           // N = 3
    vsmc_threefry4x32_rotate(state, 23, 5, 3, 1);            // N = 4
    vsmc_threefry4x32_insertkey(state, &par, 1, 1, 2, 3, 4); // N = 4
    vsmc_threefry4x32_rotate(state, 6, 20, 1, 3);            // N = 5
    vsmc_threefry4x32_rotate(state, 17, 11, 3, 1);           // N = 6
    vsmc_threefry4x32_rotate(state, 25, 10, 1, 3);           // N = 7
    vsmc_threefry4x32_rotate(state, 18, 20, 3, 1);           // N = 8
    vsmc_threefry4x32_insertkey(state, &par, 2, 2, 3, 4, 0); // N = 8
    vsmc_threefry4x32_rotate(state, 10, 26, 1, 3);           // N = 9
    vsmc_threefry4x32_rotate(state, 11, 21, 3, 1);           // N = 10
    vsmc_threefry4x32_rotate(state, 13, 27, 1, 3);           // N = 11
    vsmc_threefry4x32_rotate(state, 23, 5, 3, 1);            // N = 12
    vsmc_threefry4x32_insertkey(state, &par, 3, 3, 4, 0, 1); // N = 12
    vsmc_threefry4x32_rotate(state, 6, 20, 1, 3);            // N = 13
    vsmc_threefry4x32_rotate(state, 17, 11, 3, 1);           // N = 14
    vsmc_threefry4x32_rotate(state, 25, 10, 1, 3);           // N = 15
    vsmc_threefry4x32_rotate(state, 18, 20, 3, 1);           // N = 16
    vsmc_threefry4x32_insertkey(state, &par, 4, 4, 0, 1, 2); // N = 16
    vsmc_threefry4x32_rotate(state, 10, 26, 1, 3);           // N = 17
    vsmc_threefry4x32_rotate(state, 11, 21, 3, 1);           // N = 18
    vsmc_threefry4x32_rotate(state, 13, 27, 1, 3);           // N = 19
    vsmc_threefry4x32_rotate(state, 23, 5, 3, 1);            // N = 20
    vsmc_threefry4x32_insertkey(state, &par, 5, 0, 1, 2, 3); // N = 20
}

/// \brief Generate Threefry2x64 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry2x64_gen(const vsmc_threefry2x64_ctr_t *ctr,
    const vsmc_threefry2x64_key_t *key, vsmc_threefry2x64_ctr_t *state)
{
    *state = *ctr;
    vsmc_threefry2x64_par_t par;
    vsmc_threefry2x64_initpar(key, &par);

    vsmc_threefry2x64_insertkey(state, &par, 0, 0, 1); // N = 0
    vsmc_threefry2x64_rotate(state, 16);               // N = 1
    vsmc_threefry2x64_rotate(state, 42);               // N = 2
    vsmc_threefry2x64_rotate(state, 12);               // N = 3
    vsmc_threefry2x64_rotate(state, 31);               // N = 4
    vsmc_threefry2x64_insertkey(state, &par, 1, 1, 2); // N = 4
    vsmc_threefry2x64_rotate(state, 16);               // N = 5
    vsmc_threefry2x64_rotate(state, 32);               // N = 6
    vsmc_threefry2x64_rotate(state, 24);               // N = 7
    vsmc_threefry2x64_rotate(state, 21);               // N = 8
    vsmc_threefry2x64_insertkey(state, &par, 2, 2, 0); // N = 8
    vsmc_threefry2x64_rotate(state, 16);               // N = 9
    vsmc_threefry2x64_rotate(state, 42);               // N = 10
    vsmc_threefry2x64_rotate(state, 12);               // N = 11
    vsmc_threefry2x64_rotate(state, 31);               // N = 12
    vsmc_threefry2x64_insertkey(state, &par, 3, 0, 1); // N = 12
    vsmc_threefry2x64_rotate(state, 16);               // N = 13
    vsmc_threefry2x64_rotate(state, 32);               // N = 14
    vsmc_threefry2x64_rotate(state, 24);               // N = 15
    vsmc_threefry2x64_rotate(state, 21);               // N = 16
    vsmc_threefry2x64_insertkey(state, &par, 4, 1, 2); // N = 16
    vsmc_threefry2x64_rotate(state, 16);               // N = 17
    vsmc_threefry2x64_rotate(state, 42);               // N = 18
    vsmc_threefry2x64_rotate(state, 12);               // N = 19
    vsmc_threefry2x64_rotate(state, 31);               // N = 20
    vsmc_threefry2x64_insertkey(state, &par, 5, 2, 0); // N = 20
}

/// \brief Generate Threefry4x64 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry4x64_gen(const vsmc_threefry4x64_ctr_t *ctr,
    const vsmc_threefry4x64_key_t *key, vsmc_threefry4x64_ctr_t *state)
{
    *state = *ctr;
    vsmc_threefry4x64_par_t par;
    vsmc_threefry4x64_initpar(key, &par);

    vsmc_threefry4x64_insertkey(state, &par, 0, 0, 1, 2, 3); // N = 0
    vsmc_threefry4x64_rotate(state, 14, 16, 1, 3);           // N = 1
    vsmc_threefry4x64_rotate(state, 52, 57, 3, 1);           // N = 2
    vsmc_threefry4x64_rotate(state, 23, 40, 1, 3);           // N = 3
    vsmc_threefry4x64_rotate(state, 5, 37, 3, 1);            // N = 4
    vsmc_threefry4x64_insertkey(state, &par, 1, 1, 2, 3, 4); // N = 4
    vsmc_threefry4x64_rotate(state, 25, 33, 1, 3);           // N = 5
    vsmc_threefry4x64_rotate(state, 46, 12, 3, 1);           // N = 6
    vsmc_threefry4x64_rotate(state, 58, 22, 1, 3);           // N = 7
    vsmc_threefry4x64_rotate(state, 32, 32, 3, 1);           // N = 8
    vsmc_threefry4x64_insertkey(state, &par, 2, 2, 3, 4, 0); // N = 8
    vsmc_threefry4x64_rotate(state, 14, 16, 1, 3);           // N = 9
    vsmc_threefry4x64_rotate(state, 52, 57, 3, 1);           // N = 10
    vsmc_threefry4x64_rotate(state, 23, 40, 1, 3);           // N = 11
    vsmc_threefry4x64_rotate(state, 5, 37, 3, 1);            // N = 12
    vsmc_threefry4x64_insertkey(state, &par, 3, 3, 4, 0, 1); // N = 12
    vsmc_threefry4x64_rotate(state, 25, 33, 1, 3);           // N = 13
    vsmc_threefry4x64_rotate(state, 46, 12, 3, 1);           // N = 14
    vsmc_threefry4x64_rotate(state, 58, 22, 1, 3);           // N = 15
    vsmc_threefry4x64_rotate(state, 32, 32, 3, 1);           // N = 16
    vsmc_threefry4x64_insertkey(state, &par, 4, 4, 0, 1, 2); // N = 16
    vsmc_threefry4x64_rotate(state, 14, 16, 1, 3);           // N = 17
    vsmc_threefry4x64_rotate(state, 52, 57, 3, 1);           // N = 18
    vsmc_threefry4x64_rotate(state, 23, 40, 1, 3);           // N = 19
    vsmc_threefry4x64_rotate(state, 5, 37, 3, 1);            // N = 20
    vsmc_threefry4x64_insertkey(state, &par, 5, 0, 1, 2, 3); // N = 20
}

/// \brief Initialize Threefry2x32 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry2x32_init(
    vsmc_threefry2x32 *rng, uint32_t seed)
{
    rng->ctr.v[0] = 0;
    rng->ctr.v[1] = 0;
    rng->key.v[0] = seed;
    rng->key.v[1] = 0;
    rng->index = 2;
}

/// \brief Initialize Threefry4x32 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry4x32_init(
    vsmc_threefry4x32 *rng, uint32_t seed)
{
    rng->ctr.v[0] = 0;
    rng->ctr.v[1] = 0;
    rng->ctr.v[2] = 0;
    rng->ctr.v[3] = 0;
    rng->key.v[0] = seed;
    rng->key.v[1] = 0;
    rng->key.v[2] = 0;
    rng->key.v[3] = 0;
    rng->index = 4;
}

/// \brief Initialize Threefry2x64 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry2x64_init(
    vsmc_threefry2x64 *rng, uint64_t seed)
{
    rng->ctr.v[0] = 0;
    rng->ctr.v[1] = 0;
    rng->key.v[0] = seed;
    rng->key.v[1] = 0;
    rng->index = 2;
}

/// \brief Initialize Threefry4x64 RNG state
/// \ingroup ThreefryC
static inline void vsmc_threefry4x64_init(
    vsmc_threefry4x64 *rng, uint64_t seed)
{
    rng->ctr.v[0] = 0;
    rng->ctr.v[1] = 0;
    rng->ctr.v[2] = 0;
    rng->ctr.v[3] = 0;
    rng->key.v[0] = seed;
    rng->key.v[1] = 0;
    rng->key.v[2] = 0;
    rng->key.v[3] = 0;
    rng->index = 4;
}

/// \brief Generate random 32-bit integers from Threefry2x32 RNG
/// \ingroup ThreefryC
static inline uint32_t vsmc_threefry2x32_rand(vsmc_threefry2x32 *rng)
{
    if (rng->index == 2) {
        vsmc_threefry2x32_inc(&rng->ctr);
        vsmc_threefry2x32_gen(&rng->ctr, &rng->key, &rng->state);
        rng->index = 0;
    }

    return rng->state.v[rng->index++];
}

/// \brief Generate random 32-bit integers from Threefry4x32 RNG
/// \ingroup ThreefryC
static inline uint32_t vsmc_threefry4x32_rand(vsmc_threefry4x32 *rng)
{
    if (rng->index == 4) {
        vsmc_threefry4x32_inc(&rng->ctr);
        vsmc_threefry4x32_gen(&rng->ctr, &rng->key, &rng->state);
        rng->index = 0;
    }

    return rng->state.v[rng->index++];
}

/// \brief Generate random 64-bit integers from Threefry2x64 RNG
/// \ingroup ThreefryC
static inline uint64_t vsmc_threefry2x64_rand(vsmc_threefry2x64 *rng)
{
    if (rng->index == 2) {
        vsmc_threefry2x64_inc(&rng->ctr);
        vsmc_threefry2x64_gen(&rng->ctr, &rng->key, &rng->state);
        rng->index = 0;
    }

    return rng->state.v[rng->index++];
}

/// \brief Generate random 64-bit integers from Threefry4x64 RNG
/// \ingroup ThreefryC
static inline uint64_t vsmc_threefry4x64_rand(vsmc_threefry4x64 *rng)
{
    if (rng->index == 4) {
        vsmc_threefry4x64_inc(&rng->ctr);
        vsmc_threefry4x64_gen(&rng->ctr, &rng->key, &rng->state);
        rng->index = 0;
    }

    return rng->state.v[rng->index++];
}

#endif // VSMC_RNGC_THREEFRY_H
