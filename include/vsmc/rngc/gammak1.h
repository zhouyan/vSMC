//============================================================================
// vSMC/include/vsmc/rngc/gammak1.h
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

#ifndef VSMC_RNGC_GAMMAK1_H
#define VSMC_RNGC_GAMMAK1_H

#include <vsmc/rngc/internal/common.h>
#include <vsmc/rngc/philox.h>
#include <vsmc/rngc/normal01.h>
#include <vsmc/rngc/u01.h>

#if VSMC_HAS_RNGC_DOUBLE
#define vsmc_gammak1 vsmc_gammak1_53
#define vsmc_gammak1_init vsmc_gammak1_53_init
#define vsmc_gammak1_rand vsmc_gammak1_53_rand
#else // VSMC_HAS_RNGC_DOUBLE
#define vsmc_gammak1 vsmc_gammak1_24
#define vsmc_gammak1_init vsmc_gammak1_24_init
#define vsmc_gammak1_rand vsmc_gammak1_24_rand
#endif // VSMC_HAS_RNGC_DOUBLE

#define VSMC_DEFINE_RNGC_GAMMAK1(F, FT)                                       \
    typedef struct {                                                          \
        FT c_shape, c_s2, c_s, c_d, c_b, c_si, c_c, c_q0;                     \
        vsmc_normal01_##F rnorm;                                              \
    } vsmc_gammak1_##F;

#define VSMC_DEFINE_RNGC_GAMMAK1_INIT(F, FT)                                  \
    VSMC_STATIC_INLINE void vsmc_gammak1_##F##_init(                          \
        vsmc_gammak1_##F *rgamma, vsmc_rng *rng, FT shape)                    \
    {                                                                         \
        rgamma->c_shape = shape;                                              \
        vsmc_normal01_##F##_init(&(rgamma->rnorm), rng);                      \
                                                                              \
        if (shape <= 1) {                                                     \
            rgamma->c_s2 = 0;                                                 \
            rgamma->c_s = 0;                                                  \
            rgamma->c_d = 0;                                                  \
            rgamma->c_b = 0;                                                  \
            rgamma->c_si = 0;                                                 \
            rgamma->c_c = 0;                                                  \
            rgamma->c_q0 = 0;                                                 \
            return;                                                           \
        }                                                                     \
                                                                              \
        FT c_sqrt32 = VSMC_FP##F##_C(5.6568542494923802);                     \
        FT c_s2 = shape - VSMC_FP##F##_C(0.5);                                \
        FT c_s = VSMC_SQRT##F(c_s2);                                          \
        FT c_d = c_sqrt32 - c_s * 12;                                         \
                                                                              \
        FT c_b, c_si, c_c;                                                    \
        FT r = 1 / shape;                                                     \
        FT q1 = VSMC_FP##F##_C(0.04166669);                                   \
        FT q2 = VSMC_FP##F##_C(0.02083148);                                   \
        FT q3 = VSMC_FP##F##_C(0.00801191);                                   \
        FT q4 = VSMC_FP##F##_C(0.00144121);                                   \
        FT q5 = VSMC_FP##F##_C(-7.388e-5);                                    \
        FT q6 = VSMC_FP##F##_C(2.4511e-4);                                    \
        FT q7 = VSMC_FP##F##_C(2.424e-4);                                     \
        FT c_q0 =                                                             \
            ((((((q7 * r + q6) * r + q5) * r + q4) * r + q3) * r + q2) * r +  \
                q1) *                                                         \
            r;                                                                \
        if (shape <= VSMC_FP##F##_C(3.686)) {                                 \
            c_b = VSMC_FP##F##_C(0.463) + c_s + VSMC_FP##F##_C(0.178) * c_s2; \
            c_si = VSMC_FP##F##_C(1.235);                                     \
            c_c = VSMC_FP##F##_C(0.195) / c_s - VSMC_FP##F##_C(0.079) +       \
                VSMC_FP##F##_C(0.16) * c_s;                                   \
        } else if (shape <= VSMC_FP##F##_C(13.022)) {                         \
            c_b = VSMC_FP##F##_C(1.654) + VSMC_FP##F##_C(0.0076) * c_s2;      \
            c_si = VSMC_FP##F##_C(1.68) / c_s + VSMC_FP##F##_C(0.275);        \
            c_c = VSMC_FP##F##_C(0.062) / c_s + VSMC_FP##F##_C(0.024);        \
        } else {                                                              \
            c_b = VSMC_FP##F##_C(1.77);                                       \
            c_si = VSMC_FP##F##_C(0.75);                                      \
            c_c = VSMC_FP##F##_C(0.1515) / c_s;                               \
        }                                                                     \
                                                                              \
        rgamma->c_s2 = c_s2;                                                  \
        rgamma->c_s = c_s;                                                    \
        rgamma->c_d = c_d;                                                    \
        rgamma->c_b = c_b;                                                    \
        rgamma->c_si = c_si;                                                  \
        rgamma->c_c = c_c;                                                    \
        rgamma->c_q0 = c_q0;                                                  \
    }

#define VSMC_DEFINE_RNGC_GAMMAK1_RAND(F, FT)                                  \
    VSMC_STATIC_INLINE FT vsmc_gammak1_##F##_rand(                            \
        vsmc_gammak1_##F *rgamma, vsmc_rng *rng)                              \
    {                                                                         \
        const FT c_shape = rgamma->c_shape;                                   \
        const FT c_s2 = rgamma->c_s2;                                         \
        const FT c_s = rgamma->c_s;                                           \
        const FT c_d = rgamma->c_d;                                           \
        const FT c_b = rgamma->c_b;                                           \
        const FT c_si = rgamma->c_si;                                         \
        const FT c_c = rgamma->c_c;                                           \
        const FT c_q0 = rgamma->c_q0;                                         \
                                                                              \
        if (c_shape < 0)                                                      \
            return -1;                                                        \
                                                                              \
        if (!(c_shape > 0))                                                   \
            return 0;                                                         \
                                                                              \
        if (!(c_shape < 1) && !(c_shape > 1)) {                               \
            FT u = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));             \
            return -VSMC_LOG##F(u);                                           \
        }                                                                     \
                                                                              \
        if (c_shape < 1) {                                                    \
            const FT c_exp_m1 = VSMC_FP##F##_C(0.3678794411714423);           \
            FT b = 1 + c_exp_m1 * c_shape;                                    \
            while (1) {                                                       \
                FT u = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));         \
                FT v = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));         \
                FT p = b * u;                                                 \
                FT e = -VSMC_LOG##F(v);                                       \
                if (p >= 1) {                                                 \
                    FT x = -VSMC_LOG##F((b - p) / c_shape);                   \
                    if (e >= (1 - c_shape) * VSMC_LOG##F(x))                  \
                        return x * x;                                         \
                } else {                                                      \
                    FT x = VSMC_EXP##F(VSMC_LOG##F(p) / c_shape);             \
                    if (e >= x)                                               \
                        return x * x;                                         \
                }                                                             \
            }                                                                 \
        }                                                                     \
                                                                              \
        FT t = vsmc_normal01_##F##_rand(&(rgamma->rnorm), rng);               \
        FT x = c_s + t * VSMC_FP##F##_C(0.5);                                 \
        if (t >= 0)                                                           \
            return x * x;                                                     \
                                                                              \
        FT u = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));                 \
        if (c_d * u <= t * t * t)                                             \
            return x * x;                                                     \
                                                                              \
        const FT a1 = VSMC_FP##F##_C(0.3333333);                              \
        const FT a2 = VSMC_FP##F##_C(-0.250003);                              \
        const FT a3 = VSMC_FP##F##_C(0.2000062);                              \
        const FT a4 = VSMC_FP##F##_C(-0.1662921);                             \
        const FT a5 = VSMC_FP##F##_C(0.1423657);                              \
        const FT a6 = VSMC_FP##F##_C(-0.1367177);                             \
        const FT a7 = VSMC_FP##F##_C(0.1233795);                              \
                                                                              \
        if (x > 0) {                                                          \
            FT v = t / (c_s + c_s);                                           \
            FT q;                                                             \
            if (fabs(v) <= VSMC_FP##F##_C(0.25)) {                            \
                q = c_q0 +                                                    \
                    VSMC_FP##F##_C(0.5) * t * t *                             \
                        ((((((a7 * v + a6) * v + a5) * v + a4) * v + a3) *    \
                                 v +                                          \
                             a2) *                                            \
                                v +                                           \
                            a1) *                                             \
                        v;                                                    \
            } else {                                                          \
                q = c_q0 - c_s * t + VSMC_FP##F##_C(0.25) * t * t +           \
                    (c_s2 + c_s2) * VSMC_LOG##F(1 + v);                       \
            }                                                                 \
                                                                              \
            if (VSMC_LOG##F(1 - u) <= q)                                      \
                return x * x;                                                 \
        }                                                                     \
                                                                              \
        while (1) {                                                           \
            FT e = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));             \
            u = vsmc_u01_open_open_32_##F(vsmc_rng_rand(rng));                \
            e = -VSMC_LOG##F(e);                                              \
            u = u + u - 1;                                                    \
            t = u < 0 ? c_b - c_si * e : c_b + c_si * e;                      \
                                                                              \
            const FT c_tau = VSMC_FP##F##_C(-0.71874483771719);               \
            if (t <= c_tau)                                                   \
                continue;                                                     \
                                                                              \
            FT v = t / (c_s + c_s);                                           \
            FT q;                                                             \
            if (fabs(v) <= VSMC_FP##F##_C(0.25)) {                            \
                q = c_q0 +                                                    \
                    VSMC_FP##F##_C(0.5) * t * t *                             \
                        ((((((a7 * v + a6) * v + a5) * v + a4) * v + a3) *    \
                                 v +                                          \
                             a2) *                                            \
                                v +                                           \
                            a1) *                                             \
                        v;                                                    \
            } else {                                                          \
                q = c_q0 - c_s * t + VSMC_FP##F##_C(0.25) * t * t +           \
                    (c_s2 + c_s2) * VSMC_LOG##F(1 + v);                       \
            }                                                                 \
            if (q <= 0)                                                       \
                continue;                                                     \
            if (c_c * fabs(u) > VSMC_EXPM1##F(q) *                            \
                    VSMC_EXP##F(e - VSMC_FP##F##_C(0.5) * t * t))             \
                continue;                                                     \
                                                                              \
            x = c_s + VSMC_FP##F##_C(0.5) * t;                                \
            return x * x;                                                     \
        }                                                                     \
    }

/// \brief Gamma(k, 1) structure (single precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1(24, float)

/// \brief Initialize Gamma(k, 1) structure (single precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1_INIT(24, float)

/// \brief Generate Gamma(k, 1) random numbers (single precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1_RAND(24, float)

#if VSMC_HAS_RNGC_DOUBLE

/// \brief Gamma(k, 1) structure (double precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1(53, double)

/// \brief Initialize Gamma(k, 1) structure (double precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1_INIT(53, double)

/// \brief Generate Gamma(k, 1) random numbers (double precision)
/// \ingroup GammaK1C
VSMC_DEFINE_RNGC_GAMMAK1_RAND(53, double)

#endif // VSMC_HAS_RNGC_DOUBLE

#endif // VSMC_RNGC_GAMMAK1_H
