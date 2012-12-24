#ifndef VSMC_CL_DEVICE_H
#define VSMC_CL_DEVICE_H

#define VSMC_PASTE2(a1, a2) a1##a2

#define M_PI_24 3.1415926535897932385F
#define M_PI_53 3.1415926535897932385

#ifndef CBRNG4x32
#define CBRNG4x32       threefry4x32
#define CBRNG4x32_KEY_T threefry4x32_key_t
#define CBRNG4x32_CTR_T threefry4x32_ctr_t
#endif

#ifndef CBRNG4x64
#define CBRNG4x64 threefry4x32
#define CBRNG4x64_KEY_T threefry4x64_key_t
#define CBRNG4x64_CTR_T threefry4x64_ctr_t
#endif

#if VSMC_STATE_TYPE_IS_FLOAT
#define U01_OPEN_OPEN_32     u01_open_open_32_24
#define U01_OPEN_CLOSED_32   u01_open_closed_32_24
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_24
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_24

#define NORMAL01_32      normal01_32_24
#define NORMAL01_32_INIT normal01_32_24_init
#define NORMAL01_32_RAND normal01_32_24_rand
#endif // VSMC_STATE_TYPE_IS_FLOAT

#if VSMC_STATE_TYPE_IS_DOUBLE
#define U01_OPEN_OPEN_32     u01_open_open_32_53
#define U01_OPEN_CLOSED_32   u01_open_closed_32_53
#define U01_CLOSED_OPEN_32   u01_closed_closed_32_53
#define U01_CLOSED_CLOSED_32 u01_closed_closed_32_53

#define U01_OPEN_OPEN_64     u01_open_open_64_53
#define U01_OPEN_CLOSED_64   u01_open_closed_64_53
#define U01_CLOSED_OPEN_64   u01_closed_closed_64_53
#define U01_CLOSED_CLOSED_64 u01_closed_closed_64_53

#define NORMAL01_32      normal01_32_53
#define NORMAL01_32_INIT normal01_32_53_init
#define NORMAL01_32_RAND normal01_32_53_rand

#define NORMAL01_64      normal01_64_53
#define NORMAL01_64_INIT normal01_64_53_init
#define NORMAL01_64_RAND normal01_64_53_rand
#endif // VSMC_STATE_TYPE_IS_DOUBLE

// W  Integer width
// F  Float width
// FT Float type
#define VSMC_DEFINE_NORMAL01(W, F, FT) \
    typedef struct {                                                         \
        CBRNG4x##W##_KEY_T *k;                                               \
        CBRNG4x##W##_CTR_T *c;                                               \
        CBRNG4x##W##_CTR_T r;                                                \
        FT##4 u;                                                             \
        FT##4 z;                                                             \
        uint8_t count;                                                       \
    } normal01_##W##_##F;                                                    \
                                                                             \
    void normal01_##W##_##F##_init (normal01_##W##_##F *,                    \
            CBRNG4x##W##_KEY_T *, CBRNG4x##W##_CTR_T *);                     \
    FT normal01_##W##_##F##_rand (normal01_##W##_##F *);                     \
                                                                             \
    void normal01_##W##_##F##_init (normal01_##W##_##F *norm,                \
            CBRNG4x##W##_KEY_T *k, CBRNG4x##W##_CTR_T *c)                    \
    {                                                                        \
        norm->k = k;                                                         \
        norm->c = c;                                                         \
        norm->count = 0;                                                     \
    }                                                                        \
                                                                             \
    FT normal01_##W##_##F##_rand (normal01_##W##_##F *norm)                  \
    {                                                                        \
        if (!norm->count) {                                                  \
            norm->c->v[0]++;                                                 \
            norm->r = CBRNG4x##W(*(norm->c), *(norm->k));                    \
            norm->u.x = u01_open_closed_##W##_##F(norm->r.v[0]);             \
            norm->u.y = u01_open_closed_##W##_##F(norm->r.v[1]);             \
            norm->u.z = u01_open_closed_##W##_##F(norm->r.v[2]);             \
            norm->u.w = u01_open_closed_##W##_##F(norm->r.v[3]);             \
            norm->count = 4;                                                 \
        }                                                                    \
                                                                             \
        --norm->count;                                                       \
        switch (norm->count) {                                               \
            case 0 :                                                         \
                return sqrt(-2 * log(norm->u.x)) *                           \
                    cos(2 * M_PI_##F * norm->u.y);                           \
            case 1 :                                                         \
                return sqrt(-2 * log(norm->u.x)) *                           \
                    sin(2 * M_PI_##F * norm->u.y);                           \
            case 2 :                                                         \
                return sqrt(-2 * log(norm->u.z)) *                           \
                    cos(2 * M_PI_##F * norm->u.w);                           \
            case 3 :                                                         \
                return sqrt(-2 * log(norm->u.z)) *                           \
                    sin(2 * M_PI_##F * norm->u.w);                           \
            default :                                                        \
                return 0;                                                    \
        }                                                                    \
    }

#if VSMC_USE_RANDOM123
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#endif

__kernel
void copy (__global state_struct *state, __global size_type *copy_from)
{
    size_type to = get_global_id(0);

    if (to >= Size)
        return;

    state[to] = state[copy_from[to]];
}

VSMC_DEFINE_NORMAL01(32, 24, float);

#if R123_USE_U01_DOUBLE
VSMC_DEFINE_NORMAL01(32, 53, float);
VSMC_DEFINE_NORMAL01(64, 53, double);
#endif

#endif // VSMC_CL_DEVICE_H

// vim:ft=opencl
