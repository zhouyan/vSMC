//============================================================================
// vSMC/lib/src/vsmc_mkl_brng.cpp
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

#include <vsmc/vsmc.h>
#include <vsmc/rng/engine.hpp>

#ifdef VSMC_DEFINE_RNG_C_API_ENGINE
#undef VSMC_DEFINE_RNG_C_API_ENGINE
#endif

#define VSMC_DEFINE_RNG_C_API_ENGINE(RNGType, name)                           \
    int vsmc_mkl_init_##name(int, VSLStreamStatePtr, int, const unsigned *);  \
    int vsmc_mkl_init_##name(                                                 \
        int method, VSLStreamStatePtr stream, int n, const unsigned *param)   \
    {                                                                         \
        return ::vsmc::internal::mkl_init<RNGType>(method, stream, n, param); \
    }                                                                         \
                                                                              \
    int vsmc_mkl_sbrng_##name(VSLStreamStatePtr, int, float *, float, float); \
    int vsmc_mkl_sbrng_##name(                                                \
        VSLStreamStatePtr stream, int n, float *r, float a, float b)          \
    {                                                                         \
        return ::vsmc::internal::mkl_uniform_real<RNGType, float>(            \
            stream, n, r, a, b);                                              \
    }                                                                         \
                                                                              \
    int vsmc_mkl_dbrng_##name(                                                \
        VSLStreamStatePtr, int, double *, double, double);                    \
    int vsmc_mkl_dbrng_##name(                                                \
        VSLStreamStatePtr stream, int n, double *r, double a, double b)       \
    {                                                                         \
        return ::vsmc::internal::mkl_uniform_real<RNGType, double>(           \
            stream, n, r, a, b);                                              \
    }                                                                         \
                                                                              \
    int vsmc_mkl_ibrng_##name(VSLStreamStatePtr, int, unsigned *);            \
    int vsmc_mkl_ibrng_##name(VSLStreamStatePtr stream, int n, unsigned *r)   \
    {                                                                         \
        return ::vsmc::internal::mkl_uniform_int<RNGType>(stream, n, r);      \
    }                                                                         \
                                                                              \
    int vsmc_mkl_brng_##name(void)                                            \
    {                                                                         \
        VSLBRngProperties properties;                                         \
        properties.StreamStateSize =                                          \
            sizeof(::vsmc::internal::MKLSTDStreamState<RNGType>);             \
        properties.NSeeds = 1;                                                \
        properties.IncludesZero = RNGType::min() == 0 ? 1 : 0;                \
        properties.WordSize = sizeof(typename RNGType::result_type);          \
        properties.NBits = properties.WordSize * 8;                           \
        properties.InitStream = vsmc_mkl_init_##name;                         \
        properties.sBRng = vsmc_mkl_sbrng_##name;                             \
        properties.dBRng = vsmc_mkl_dbrng_##name;                             \
        properties.iBRng = vsmc_mkl_ibrng_##name;                             \
                                                                              \
        return ::vslRegisterBrng(&properties);                                \
    }

namespace vsmc
{

namespace internal
{

template <typename RNGType>
class MKLSTDStreamState
{
    public:
    unsigned reserved1[2];
    unsigned reserved2[2];
    RNGType rng;
}; // class MKLSTDStreamState

template <typename RNGType>
inline int mkl_init(
    int method, ::VSLStreamStatePtr stream, int n, const unsigned *param)
{
    RNGType &rng =
        (*reinterpret_cast<MKLSTDStreamState<RNGType> *>(stream)).rng;
    if (method == VSL_INIT_METHOD_STANDARD)
        rng = RNGType(static_cast<typename RNGType::result_type>(param[0]));
    if (method == VSL_INIT_METHOD_LEAPFROG)
        return VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED;
    if (method == VSL_INIT_METHOD_SKIPAHEAD)
        rng.discard(static_cast<unsigned>(n));

    return 0;
}

template <typename RNGType, typename RealType>
inline int mkl_uniform_real(
    ::VSLStreamStatePtr stream, int n, RealType *r, RealType a, RealType b)
{
    RNGType &rng =
        (*reinterpret_cast<MKLSTDStreamState<RNGType> *>(stream)).rng;
    std::uniform_real_distribution<RealType> runif(a, b);
    for (int i = 0; i != n; ++i)
        r[i] = runif(rng);

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int_dispatch(::VSLStreamStatePtr stream, int n,
    unsigned *r, std::integral_constant<std::size_t, sizeof(unsigned)>)
{
    RNGType &rng =
        (*reinterpret_cast<MKLSTDStreamState<RNGType> *>(stream)).rng;
    for (int i = 0; i != n; ++i)
        r[i] = static_cast<unsigned>(rng());

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int_dispatch(::VSLStreamStatePtr stream, int n,
    unsigned *r, std::integral_constant<std::size_t, sizeof(unsigned) / 2>)
{
    RNGType &rng =
        (*reinterpret_cast<MKLSTDStreamState<RNGType> *>(stream)).rng;
    typename RNGType::result_type *result =
        reinterpret_cast<typename RNGType::result_type *>(r);
    for (int i = 0; i != n * 2; ++i)
        result[i] = rng();

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int_dispatch(::VSLStreamStatePtr stream, int n,
    unsigned *r, std::integral_constant<std::size_t, sizeof(unsigned) * 2>)
{
    RNGType &rng =
        (*reinterpret_cast<MKLSTDStreamState<RNGType> *>(stream)).rng;
    typename RNGType::result_type *result =
        reinterpret_cast<typename RNGType::result_type *>(r);
    for (int i = 0; i != n / 2; ++i)
        result[i] = rng();
    if (n % 2 != 0)
        r[n - 1] = static_cast<unsigned>(rng());

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int(::VSLStreamStatePtr stream, int n, unsigned *r)
{
    return mkl_uniform_int_dispatch<RNGType>(
        stream, n, r, std::integral_constant<std::size_t,
                          sizeof(typename RNGType::result_type)>());
}

} // namespace vsmc::internal

} // namespace vsmc

extern "C" {

#include <vsmc/rng/internal/c_api_engine.hpp>

} // extern "C"
