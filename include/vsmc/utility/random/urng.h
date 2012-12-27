#ifndef VSMC_UTILITY_RANDOM_URNG_H
#define VSMC_UTILITY_RANDOM_URNG_H

#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>

#ifndef CBRNG2x32
#define CBRNG2x32 threefry2x32
#endif

#ifndef CBRNG2x64
#define CBRNG2x64 threefry2x64
#endif

#ifndef CBRNG4x32
#define CBRNG4x32 threefry4x32
#endif

#ifndef CBRNG4x64
#define CBRNG4x64 threefry4x64
#endif

#define VSMC_DEFINE_CBURNG(N, W) \
    typedef struct {                                                         \
        struct r123array##N##x##W key;                                       \
        struct r123array##N##x##W ctr;                                       \
        struct r123array##N##x##W rnd;                                       \
        unsigned char remain;                                                \
    } cburng##N##x##W;

#define VSMC_DEFINE_CBURNG_INIT(N, W) \
    VSMC_STATIC_INLINE void cburng##N##x##W##_init(cburng##N##x##W *rng)     \
    {                                                                        \
        struct r123array##N##x##W v = {{}};                                  \
        rng->key = rng->ctr = rng->rnd = v;                                  \
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

/// \ingroup Random
VSMC_DEFINE_CBURNG(2, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG(2, 64);
/// \ingroup Random
VSMC_DEFINE_CBURNG(4, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG(4, 64);

/// \ingroup Random
VSMC_DEFINE_CBURNG_INIT(2, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG_INIT(2, 64);
/// \ingroup Random
VSMC_DEFINE_CBURNG_INIT(4, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG_INIT(4, 64);

/// \ingroup Random
VSMC_DEFINE_CBURNG_RAND(2, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG_RAND(2, 64);
/// \ingroup Random
VSMC_DEFINE_CBURNG_RAND(4, 32);
/// \ingroup Random
VSMC_DEFINE_CBURNG_RAND(4, 64);

#endif // VSMC_UTILITY_RANDOM_URNG_H
