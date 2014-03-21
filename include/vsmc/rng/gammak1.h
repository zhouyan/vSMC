/// \page gammak1 Gamma distribution
///
/// - Header: `<vsmc/rng/gammak1.h>`
/// - Distribution: Gamma with scale 1, shape k,
///  \f$ f(x) = \frac{1}{\Gamma(k)}x^{k-1}e^{-x} \f$
/// - Method:
///   - k >= 1: Ahrens, J.H. and Dieter, U. (1982). Generating gamma variates
///   by a modified rejection technique. Comm. ACM, 25, 47-54.
///   - k < 1: Ahrens, J.H. and Dieter, U. (1974). Computer methods for
///   sampling from gamma, beta, poisson and binomial distributions. Computing,
///   12, 223-246.
///
/// ### Types and functions
///
/// ~~~{.c}
/// gammak1_<N>x<W>_<F>;
/// void gammak1_<N>x<W>_<F>_init (gammak1_<N>x<W>_<F> *, cburng<N>x<W>_rng_t *, <FT> shape);
/// <FT> gammak1_<N>x<W>_<F>_init (gammak1_<N>x<W>_<F> *, cburng<N>x<W>_rng_t *);
/// ~~~
///
/// ### Macros
///
/// ~~~{.c}
/// GAMMAK1_<N>x<W>
/// GAMMAK1_<N>x<W>_INIT
/// GAMMAK1_<N>x<W>_RAND
/// ~~~
///
/// ### Examples
///
/// ~~~{.c}
/// #define VSMC_OPENCL_USE_DOUBLE 1
/// #include <vsmc/rng/gammak1.h>
///
/// cburng4x32_rng_t rng;
/// cburng4x32_init(&rng);
///
/// double shape = 2;
/// gammak1_4x32_53 rgamma_53;
/// gammak1_4x32_53_init(&rgamma_53, &rng, shape);
/// double r_53 = gammak1_4x32_53_rand(&rgamma_53, &rng);
///
/// GAMMAK1_4x32 rgamma;
/// GAMMAK1_4x32_INIT(&rgamma, &rng, shape);
/// double r = GAMMAK1_4x32_RAND(&rgamma, &rng);
/// ~~~

#ifndef VSMC_RNG_GAMMAK1_H
#define VSMC_RNG_GAMMAK1_H

#include <vsmc/rng/urng.h>
#include <vsmc/rng/normal01.h>
#include <vsmc/rng/u01.h>

#if VSMC_OPENCL_USE_DOUBLE

#define GAMMAK1_2x32      gammak1_2x32_53
#define GAMMAK1_2x32_INIT gammak1_2x32_53_init
#define GAMMAK1_2x32_RAND gammak1_2x32_53_rand

#define GAMMAK1_2x64      gammak1_2x64_53
#define GAMMAK1_2x64_INIT gammak1_2x64_53_init
#define GAMMAK1_2x64_RAND gammak1_2x64_53_rand

#define GAMMAK1_4x32      gammak1_4x32_53
#define GAMMAK1_4x32_INIT gammak1_4x32_53_init
#define GAMMAK1_4x32_RAND gammak1_4x32_53_rand

#define GAMMAK1_4x64      gammak1_4x64_53
#define GAMMAK1_4x64_INIT gammak1_4x64_53_init
#define GAMMAK1_4x64_RAND gammak1_4x64_53_rand

#else // VSMC_OPENCL_USE_DOUBLE

#define GAMMAK1_2x32      gammak1_2x32_24
#define GAMMAK1_2x32_INIT gammak1_2x32_24_init
#define GAMMAK1_2x32_RAND gammak1_2x32_24_rand

#define GAMMAK1_4x32      gammak1_4x32_24
#define GAMMAK1_4x32_INIT gammak1_4x32_24_init
#define GAMMAK1_4x32_RAND gammak1_4x32_24_rand

#endif // VSMC_OPENCL_USE_DOUBLE

#define VSMC_DEFINE_RNG_GAMMAK1(N, W, F, FT) \
    typedef struct {                                                         \
        FT c_shape, c_s2, c_s, c_d, c_b, c_si, c_c, c_q0;                    \
        normal01_##N##x##W##_##F rnorm;                                      \
    } gammak1_##N##x##W##_##F;

#define VSMC_DEFINE_RNG_GAMMAK1_INIT(N, W, F, FT) \
    VSMC_STATIC_INLINE void gammak1_##N##x##W##_##F##_init (                 \
            gammak1_##N##x##W##_##F *rgamma,                                 \
            cburng##N##x##W##_rng_t *rng, FT shape)                          \
    {                                                                        \
        rgamma->c_shape = shape;                                             \
        normal01_##N##x##W##_##F##_init(&(rgamma->rnorm), rng);              \
                                                                             \
        if (shape <= 1) {                                                    \
            rgamma->c_s2 = 0;                                                \
            rgamma->c_s = 0;                                                 \
            rgamma->c_d = 0;                                                 \
            rgamma->c_b = 0;                                                 \
            rgamma->c_si = 0;                                                \
            rgamma->c_c = 0;                                                 \
            rgamma->c_q0 = 0;                                                \
            return;                                                          \
        }                                                                    \
                                                                             \
        FT c_sqrt32 = 5.6568542494923802;                                    \
        FT c_s2 = shape - 0.5f;                                              \
        FT c_s = sqrt(c_s2);                                                 \
        FT c_d = c_sqrt32 - c_s * 12;                                        \
                                                                             \
        FT c_b, c_si, c_c;                                                   \
        FT r = 1 / shape;                                                    \
        FT q1 = 0.04166669;                                                  \
        FT q2 = 0.02083148;                                                  \
        FT q3 = 0.00801191;                                                  \
        FT q4 = 0.00144121;                                                  \
        FT q5 = -7.388e-5;                                                   \
        FT q6 = 2.4511e-4;                                                   \
        FT q7 = 2.424e-4;                                                    \
        FT c_q0 = ((((((q7 * r + q6) * r + q5) * r + q4) * r + q3) * r +     \
                    q2) * r + q1) * r;                                       \
        if (shape <= 3.686f) {                                               \
            c_b = 0.463f + c_s + 0.178f * c_s2;                              \
            c_si = 1.235f;                                                   \
            c_c = 0.195f / c_s - 0.079f + 0.16f * c_s;                       \
        } else if (shape <= 13.022f) {                                       \
            c_b = 1.654f + 0.0076f * c_s2;                                   \
            c_si = 1.68f / c_s + 0.275f;                                     \
            c_c = 0.062f / c_s + 0.024f;                                     \
        } else {                                                             \
            c_b = 1.77f;                                                     \
            c_si = 0.75f;                                                    \
            c_c = 0.1515f / c_s;                                             \
        }                                                                    \
                                                                             \
        rgamma->c_s2    = c_s2;                                              \
        rgamma->c_s     = c_s;                                               \
        rgamma->c_d     = c_d;                                               \
        rgamma->c_b     = c_b;                                               \
        rgamma->c_si    = c_si;                                              \
        rgamma->c_c     = c_c;                                               \
        rgamma->c_q0    = c_q0;                                              \
    }

#define VSMC_DEFINE_RNG_GAMMAK1_RAND(N, W, F, FT) \
    VSMC_STATIC_INLINE FT gammak1_##N##x##W##_##F##_rand (                   \
            gammak1_##N##x##W##_##F *rgamma, cburng##N##x##W##_rng_t *rng)   \
    {                                                                        \
        const FT c_shape = rgamma->c_shape;                                  \
        const FT c_s2    = rgamma->c_s2;                                     \
        const FT c_s     = rgamma->c_s;                                      \
        const FT c_d     = rgamma->c_d;                                      \
        const FT c_b     = rgamma->c_b;                                      \
        const FT c_si    = rgamma->c_si;                                     \
        const FT c_c     = rgamma->c_c;                                      \
        const FT c_q0    = rgamma->c_q0;                                     \
                                                                             \
        if (c_shape < 0)                                                     \
            return -1;                                                       \
                                                                             \
        if (c_shape == 0)                                                    \
            return 0;                                                        \
                                                                             \
        if (c_shape == 1) {                                                  \
            FT u = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng));     \
            return -log(u);                                                  \
        }                                                                    \
                                                                             \
        if (c_shape < 1) {                                                   \
            const FT c_exp_m1 = 0.3678794411714423;                          \
            FT b = 1 + c_exp_m1 * c_shape;                                   \
            while (true) {                                                   \
                FT u = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng)); \
                FT v = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng)); \
                FT p = b * u;                                                \
                FT e = -log(v);                                              \
                if (p >= 1) {                                                \
                    FT x = -log((b - p) / c_shape);                          \
                    if (e >= (1 - c_shape) * log(x))                         \
                        return x * x;                                        \
                } else {                                                     \
                    FT x = exp(log(p) / c_shape);                            \
                    if (e >= x)                                              \
                        return x * x;                                        \
                }                                                            \
            }                                                                \
        }                                                                    \
                                                                             \
        FT t = normal01_##N##x##W##_##F##_rand(&(rgamma->rnorm), rng);       \
        FT x = c_s + t * 0.5f;                                               \
        if (t >= 0)                                                          \
            return x * x;                                                    \
                                                                             \
        FT u = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng));         \
        if (c_d * u <= t * t * t)                                            \
            return x * x;                                                    \
                                                                             \
        const FT a1 = 0.3333333;                                             \
        const FT a2 = -0.250003;                                             \
        const FT a3 = 0.2000062;                                             \
        const FT a4 = -0.1662921;                                            \
        const FT a5 = 0.1423657;                                             \
        const FT a6 = -0.1367177;                                            \
        const FT a7 = 0.1233795;                                             \
                                                                             \
        if (x > 0) {                                                         \
            FT v = t / (c_s + c_s);                                          \
            FT q;                                                            \
            if (fabs(v) <= 0.25f) {                                          \
                q = c_q0 + 0.5f * t * t *                                    \
                    ((((((a7 * v + a6) * v + a5) * v + a4) *                 \
                       v + a3) * v + a2) * v + a1) * v;                      \
            } else {                                                         \
                q = c_q0 - c_s * t + 0.25f * t * t +                         \
                    (c_s2 + c_s2) * log(1 + v);                              \
            }                                                                \
                                                                             \
            if (log(1 - u) <= q)                                             \
                return x * x;                                                \
        }                                                                    \
                                                                             \
        while (true) {                                                       \
            FT e = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng));     \
            FT u = u01_open_open_##W##_##F(cburng##N##x##W##_rand(rng));     \
            e = -log(e);                                                     \
            u = u + u - 1;                                                   \
            FT t = u < 0 ?  c_b - c_si * e : c_b + c_si * e;                 \
                                                                             \
            const FT c_tau = -0.71874483771719;                              \
            if (t <= c_tau)                                                  \
                continue;                                                    \
                                                                             \
            FT v = t / (c_s + c_s);                                          \
            FT q;                                                            \
            if (fabs(v) <= 0.25f) {                                          \
                q = c_q0 + 0.5f * t * t *                                    \
                    ((((((a7 * v + a6) * v + a5) * v + a4) *                 \
                       v + a3) * v + a2) * v + a1) * v;                      \
            } else {                                                         \
                q = c_q0 - c_s * t + 0.25f * t * t +                         \
                    (c_s2 + c_s2) * log(1 + v);                              \
            }                                                                \
            if (q <= 0)                                                      \
                continue;                                                    \
            if (c_c * fabs(u) > expm1(q) * exp(e - 0.5f * t * t))            \
                continue;                                                    \
                                                                             \
            FT x = c_s + 0.5f * t;                                           \
            return x * x;                                                    \
        }                                                                    \
    }

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(2, 32, 24, float)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(4, 32, 24, float)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(2, 32, 24, float)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(4, 32, 24, float)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(2, 32, 24, float)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(4, 32, 24, float)

#if VSMC_OPENCL_USE_DOUBLE

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(2, 32, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(4, 32, 53, double)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(2, 32, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(4, 32, 53, double)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(2, 32, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(4, 32, 53, double)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(2, 64, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1(4, 64, 53, double)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(2, 64, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_INIT(4, 64, 53, double)

/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(2, 64, 53, double)
/// \ingroup CLRNG
VSMC_DEFINE_RNG_GAMMAK1_RAND(4, 64, 53, double)

#endif // VSMC_OPENCL_USE_DOUBLE

#endif // VSMC_RNG_GAMMAK1_H
