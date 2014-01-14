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
/// The counter-based RNGs are wrapped in the above header. Four are defined,
/// ~~~{.c}
/// cburng2x32
/// cburng4x32
/// cburng2x64
/// cburng4x64
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
/// cburng4x32 rng;
/// cburng4x32_init(&rng);
/// uint32_t res = cburng4x32_rand(&rng);
/// ~~~
/// The function `cburng4x32_rand` will be responsible for increasing the
/// counters.  The keys and counters can also be set manually. For example,
/// ~~~{.c}
/// rng.key.v[0] = get_global_id(0);
/// rng.ctr.v[0] = 0;
/// ~~~
///
/// One may keep the states of counters between OpenCL kernel calls as in the
/// following example,
/// ~~~{.c}
/// __kernel void ker (__global struct r123array4x32 *counter)
/// {
///     cburng4x32 rng;
///     cburng4x32_init(&rng, get_global_id(0));
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
/// precision standard Normal random variates using `cburng4x32` engines.
///
/// Not all OpenCL devices have `double` precision support. Therefore by
/// default, only 32-bits and `float` versions of these types and functions are
/// defined. To enable 64-bits and `double` versions, define the macro
/// `VSMC_FP_TYPE_IS_DOUBLE` with a non-zero value.
///
/// In a single program, usually only `float` or `double` precision is used.
/// Macros are defined according to the value of `VSMC_FP_TYPE_IS_DOUBLE`. For
/// example,
/// ~~~{.c}
/// NORMAL01_4x32
/// ~~~
/// is the type used to construct objects that can be used to generate standard
/// Normal random variates using `cburng4x32` engines. The generated results is
/// `double` if `VSMC_FP_TYPE_IS_DOUBLE` is defined and non-zero or `float`
/// otherwise.
///
/// In the documentation of each distribution, the following notations are used,
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

#ifndef VSMC_RNG_URNG_H
#define VSMC_RNG_URNG_H

#include <vsmc/rng/defines.h>
#include <Random123/threefry.h>

#ifndef CBRNG2x32
#define CBRNG2x32 threefry2x32
#define CBRNG2x32KEYINIT threefry2x32keyinit
#endif

#ifndef CBRNG4x32
#define CBRNG4x32 threefry4x32
#define CBRNG4x32KEYINIT threefry4x32keyinit
#endif

#ifndef CBRNG2x64
#define CBRNG2x64 threefry2x64
#define CBRNG2x64KEYINIT threefry2x64keyinit
#endif

#ifndef CBRNG4x64
#define CBRNG4x64 threefry4x64
#define CBRNG4x64KEYINIT threefry4x64keyinit
#endif

#define VSMC_DEFINE_CBURNG(N, W) \
    typedef struct {                                                         \
        struct r123array##N##x##W key;                                       \
        struct r123array##N##x##W ctr;                                       \
        struct r123array##N##x##W rnd;                                       \
        unsigned char remain;                                                \
    } cburng##N##x##W;

#define VSMC_DEFINE_CBURNG_INIT(N, W) \
    VSMC_STATIC_INLINE void cburng##N##x##W##_init(                          \
            cburng##N##x##W *rng, uint##W##_t seed)                          \
    {                                                                        \
        struct r123array##N##x##W ukey = {{0}};                              \
        rng->ctr = rng->rnd = ukey;                                          \
        ukey.v[0] = seed;                                                    \
        rng->key = CBRNG##N##x##W##KEYINIT(ukey);                            \
        rng->remain = 0;                                                     \
    }

#define VSMC_DEFINE_CBURNG_RAND(N, W) \
    VSMC_STATIC_INLINE uint##W##_t cburng##N##x##W##_rand(                   \
            cburng##N##x##W *rng)                                            \
    {                                                                        \
        if (!rng->remain) {                                                  \
            rng->ctr.v[0]++;                                                 \
            rng->rnd = CBRNG##N##x##W(rng->ctr, rng->key);                   \
            rng->remain = N;                                                 \
        }                                                                    \
        rng->remain--;                                                       \
                                                                             \
        return rng->rnd.v[rng->remain];                                      \
    }

/// \ingroup RNG
VSMC_DEFINE_CBURNG(2, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG(4, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG(2, 64)
/// \ingroup RNG
VSMC_DEFINE_CBURNG(4, 64)

/// \ingroup RNG
VSMC_DEFINE_CBURNG_INIT(2, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_INIT(4, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_INIT(2, 64)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_INIT(4, 64)

/// \ingroup RNG
VSMC_DEFINE_CBURNG_RAND(2, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_RAND(4, 32)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_RAND(2, 64)
/// \ingroup RNG
VSMC_DEFINE_CBURNG_RAND(4, 64)

#endif // VSMC_RNG_URNG_H
