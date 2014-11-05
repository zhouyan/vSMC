//============================================================================
// include/vsmc/rng/urng.h
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_URNG_H
#define VSMC_RNG_URNG_H

/// \page rng Generating random numbers
///
/// ## The problem
///
/// The [Random123][Random123] library is ideal for generating random numbers
/// on OpenCL device. It is easy to use, for example, say inside a kernel, to
/// generate four standard normal random variates,
/// ~~~{.c}
/// #include <Random123/threefry64.h>
/// #include <vsmc/opencl/u01.h>
///
/// threefry4x32_ctr_t c = {{}};
/// threefry4x32_key_t k = {{}};
/// k.v[0] = get_global_id(0);
/// c.v[0] = iter; // some iteration number change between kernel calls.
/// threefry4x32_ctr_t r = threefry4x32(c, k);
/// float u[4];
/// float z[4];
/// for (int i = 0; i != 4; ++i)
///     u[i] = u01_open_closed_32_24(r.v[i]);
/// z[0] = sqrt(-2 * log(u[0])) * cos(2 * M_PI_F * u[1]);
/// z[1] = sqrt(-2 * log(u[0])) * sin(2 * M_PI_F * u[1]);
/// z[2] = sqrt(-2 * log(u[2])) * cos(2 * M_PI_F * u[3]);
/// z[3] = sqrt(-2 * log(u[2])) * sin(2 * M_PI_F * u[3]);
/// ~~~
/// As seen above, for every call to `threefry4x32` we can generate four
/// standard normal random variates. It is fine if we just happen to need four
/// random numbers. But this is rarely the case. Then we need some tricky code
/// to keep track of when to call `threefry4x32` again etc., or call it every
/// time we need a random number and waste a lot of time and generated numbers.
///
/// vSMC's RNG module provide some facilities to partially solve this problem.
///
/// [Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
///
/// ## RNG engines
///
/// ~~~{.c}
/// <vsmc/opencl/urng.h>
/// ~~~
/// The counter-based RNGs are wrapped in the above header. Eight are defined,
/// ~~~{.c}
/// threefry2x32_rng_t
/// threefry4x32_rng_t
/// threefry2x64_rng_t
/// threefry4x64_rng_t
/// philox2x32_rng_t
/// philox4x32_rng_t
/// philox2x64_rng_t
/// philox4x64_rng_t
/// ~~~
/// The default engines use `threefry4x32` etc. To use the Philox engine,
/// define macros `CBRNG4x32` *and* `CBRNG4x32KEYINIT` etc., before including
/// vSMC headers. For example,
/// ~~~{.c}
/// #define CBRNG4x32 philox4x32
/// #define CBRNG4x32KEYINIT philox4x32keyinit
/// ~~~
///
/// Each RNG engine is used as the following,
/// ~~~{.c}
/// #include <vsmc/opencl/urng.h>
///
/// threefry4x32_rng_t rng;
/// threefry4x32_init(&rng);
/// uint32_t res = threefry4x32_rand(&rng);
/// ~~~
/// The function `threefry4x32_rand` will be responsible for increasing the
/// counters. The keys and counters can also be set manually. For example,
/// ~~~{.c}
/// rng.key.v[0] = get_global_id(0);
/// rng.ctr.v[0] = 0;
/// ~~~
///
/// One may keep the states of counters between OpenCL kernel calls as in the
/// following example,
/// ~~~{.c}
/// __kernel void ker (__global struct threefry4x32_ctr_t *counter)
/// {
///     threefry4x32_rng_t rng;
///     threefry4x32_init(&rng, get_global_id(0));
///     rng.ctr = counter[get_global_id(0)];
///     // use engine rng
///     counter[i] = rng.ctr;
/// }
/// ~~~
/// Since OpenCL does not support static local variables, this is the most
/// convenient way to ensure that the RNG used in each kernel call does not
/// overlap their counters.
///
/// ## Distributions
///
/// ### Overview
///
/// For each distribution, a set of types and functions are defined. Each of
/// them use either 32- or 64-bits RNG and generate `float` or `double`
/// precision results. For example,
/// ~~~{.c}
/// float u01_open_closed_32_24 (uint32_t);
/// ~~~
/// generate `float` precision uniform random variates on `(0, 1]` using
/// 32-bits integers. Another example,
/// ~~~{.c}
/// normal01_4x32_24
/// ~~~
/// is the type used to construct objects that can be used to generate `float`
/// precision standard Normal random variates using `threefry4x32` engines.
///
/// Not all OpenCL devices have `double` precision support. Therefore by
/// default, only 32-bits and `float` versions of these types and functions are
/// defined. To enable 64-bits and `double` versions, define the macro
/// `VSMC_HAS_OPENCL_DOUBLE` with a non-zero value.
///
/// In a single program, usually only `float` or `double` precision is used.
/// Macros are defined according to the value of `VSMC_HAS_OPENCL_DOUBLE`. For
/// example,
/// ~~~{.c}
/// NORMAL01_4x32
/// ~~~
/// is the type used to construct objects that can be used to generate standard
/// Normal random variates using `threefry4x32` engines. The generated results
/// is `double` if `VSMC_HAS_OPENCL_DOUBLE` is defined and non-zero or `float`
/// otherwise.
///
/// In the documentation of each distribution, the following notations are
/// used,
///
/// - `<N>`: 2 or 4, the rounds of counter-based RNG algorithm
/// - `<W>`: 32 or 64, the bits of integer RNG
/// - `<FT>`: float or double
/// - `<F>`: 24 if `<FT>` is float, 53 if `<FT>` is double
///
/// \sa
///
/// \subpage u01
///
/// \subpage normal01
///
/// \subpage gammak1

#include <vsmc/rng/internal/common.h>
#include <Random123/threefry.h>
#include <Random123/philox.h>

#define VSMC_DEFINE_RNG_URNG_RNG_T(RNG, N, W) \
    typedef struct {                                                         \
        RNG##N##x##W##_key_t key;                                            \
        RNG##N##x##W##_ctr_t ctr;                                            \
        RNG##N##x##W##_ctr_t rnd;                                            \
        unsigned char remain;                                                \
    } RNG##N##x##W##_rng_t;

#define VSMC_DEFINE_RNG_URNG_INIT(RNG, N, W) \
    VSMC_STATIC_INLINE void RNG##N##x##W##_init(                             \
            RNG##N##x##W##_rng_t *rng, uint##W##_t seed)                     \
    {                                                                        \
        RNG##N##x##W##_ctr_t init_ctr = {{}};                                \
        RNG##N##x##W##_ukey_t ukey = {{}};                                   \
        rng->ctr = init_ctr;                                                 \
        rng->rnd = init_ctr;                                                 \
        ukey.v[0] = seed;                                                    \
        rng->key = RNG##N##x##W##keyinit(ukey);                              \
        rng->remain = 0;                                                     \
    }

#define VSMC_DEFINE_RNG_URNG_RAND(RNG, N, W) \
    VSMC_STATIC_INLINE uint##W##_t RNG##N##x##W##_rand(                      \
            RNG##N##x##W##_rng_t *rng)                                       \
    {                                                                        \
        unsigned char remain = rng->remain;                                  \
        RNG##N##x##W##_ctr_t rnd = rng->rnd;                                 \
                                                                             \
        if (remain > 0) {                                                    \
            --remain;                                                        \
            rng->remain = remain;                                            \
            return rnd.v[remain];                                            \
        }                                                                    \
                                                                             \
        RNG##N##x##W##_ctr_t ctr = rng->ctr;                                 \
        RNG##N##x##W##_key_t key = rng->key;                                 \
                                                                             \
        remain = N - 1;                                                      \
        ctr.v[0]++;                                                          \
        rnd = RNG##N##x##W(ctr, key);                                        \
                                                                             \
        rng->remain = remain;                                                \
        rng->rnd = rnd;                                                      \
        rng->ctr = ctr;                                                      \
        rng->key = key;                                                      \
                                                                             \
        return rnd.v[remain];                                                \
    }

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(threefry, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(threefry, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(threefry, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(threefry, 4, 64)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(threefry, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(threefry, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(threefry, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(threefry, 4, 64)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(threefry, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(threefry, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(threefry, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(threefry, 4, 64)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(philox, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(philox, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(philox, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RNG_T(philox, 4, 64)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(philox, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(philox, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(philox, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_INIT(philox, 4, 64)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(philox, 2, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(philox, 4, 32)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(philox, 2, 64)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_URNG_RAND(philox, 4, 64)

#if VSMC_USE_THREEFRY_CBURNG
/// \ingroup CLRNG
typedef threefry2x32_rng_t cburng2x32_rng_t;
/// \ingroup CLRNG
typedef threefry4x32_rng_t cburng4x32_rng_t;
/// \ingroup CLRNG
typedef threefry2x64_rng_t cburng2x64_rng_t;
/// \ingroup CLRNG
typedef threefry4x64_rng_t cburng4x64_rng_t;
/// \ingroup CLRNG
#define cburng2x32_init threefry2x32_init
/// \ingroup CLRNG
#define cburng4x32_init threefry4x32_init
/// \ingroup CLRNG
#define cburng2x64_init threefry2x64_init
/// \ingroup CLRNG
#define cburng4x64_init threefry4x64_init
/// \ingroup CLRNG
#define cburng2x32_rand threefry2x32_rand
/// \ingroup CLRNG
#define cburng4x32_rand threefry4x32_rand
/// \ingroup CLRNG
#define cburng2x64_rand threefry2x64_rand
/// \ingroup CLRNG
#define cburng4x64_rand threefry4x64_rand
#else
/// \ingroup CLRNG
typedef philox2x32_rng_t cburng2x32_rng_t;
/// \ingroup CLRNG
typedef philox4x32_rng_t cburng4x32_rng_t;
/// \ingroup CLRNG
typedef philox2x64_rng_t cburng2x64_rng_t;
/// \ingroup CLRNG
typedef philox4x64_rng_t cburng4x64_rng_t;
/// \ingroup CLRNG
#define cburng2x32_init philox2x32_init
/// \ingroup CLRNG
#define cburng4x32_init philox4x32_init
/// \ingroup CLRNG
#define cburng2x64_init philox2x64_init
/// \ingroup CLRNG
#define cburng4x64_init philox4x64_init
/// \ingroup CLRNG
#define cburng2x32_rand philox2x32_rand
/// \ingroup CLRNG
#define cburng4x32_rand philox4x32_rand
/// \ingroup CLRNG
#define cburng2x64_rand philox2x64_rand
/// \ingroup CLRNG
#define cburng4x64_rand philox4x64_rand
#endif

#endif // VSMC_RNG_URNG_H
