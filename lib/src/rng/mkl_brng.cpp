//============================================================================
// vSMC/lib/src/vsmc_mkl_brng.cpp
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

#include "libvsmc.hpp"

#ifdef VSMC_DEFINE_RNG_MKL_BRNG
#undef VSMC_DEFINE_RNG_MKL_BRNG
#endif

#define VSMC_DEFINE_RNG_MKL_BRNG(RNGType, name)                               \
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
        ::VSLBRngProperties properties;                                       \
        properties.StreamStateSize =                                          \
            sizeof(::vsmc::internal::MKLStreamState<RNGType>);                \
        properties.NSeeds = ::vsmc::internal::mkl_nseeds<RNGType>();          \
        properties.IncludesZero = 1;                                          \
        properties.WordSize = 4;                                              \
        properties.NBits = 32;                                                \
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
class MKLStreamState
{
    public:
    unsigned reserved1[2];
    unsigned reserved2[2];
    RNGType rng;
}; // class MKLStreamState

template <typename RNGType>
inline int mkl_nseeds(std::false_type)
{
    return (sizeof(typename RNGType::ctr_type) +
               sizeof(typename RNGType::key_type)) /
        sizeof(unsigned);
}

template <typename RNGType>
inline int mkl_nseeds(std::true_type)
{
    return 1;
}

template <typename RNGType>
inline int mkl_nseeds()
{
    return mkl_nseeds<RNGType>(std::integral_constant<
        bool, (std::is_same<CtrType<RNGType>, NullType>::value ||
                  std::is_same<KeyType<RNGType>, NullType>::value)>());
}

template <typename RNGType>
inline int mkl_init(
    RNGType &rng, int n, const unsigned *param, std::false_type)
{
    int nc = static_cast<int>(
        sizeof(typename RNGType::ctr_type) / sizeof(unsigned));
    int nk = static_cast<int>(
        sizeof(typename RNGType::key_type) / sizeof(unsigned));
    new (static_cast<void *>(&rng)) RNGType();

    if (n > 0) {
        std::size_t size =
            static_cast<std::size_t>(std::min(n, nk)) * sizeof(unsigned);
        typename RNGType::key_type key;
        std::fill(key.begin(), key.end(), 0);
        std::memcpy(key.data(), param, size);
        rng.key(key);
    }

    if (n > nk) {
        n -= nk;
        param += nk;
        std::size_t size =
            static_cast<std::size_t>(std::min(n, nc)) * sizeof(unsigned);
        typename RNGType::ctr_type ctr;
        std::fill(ctr.begin(), ctr.end(), 0);
        std::memcpy(ctr.data(), param, size);
        rng.ctr(ctr);
    }

    return 0;
}

template <typename RNGType>
inline int mkl_init(RNGType &rng, int n, const unsigned *param, std::true_type)
{
    if (n == 0) {
        new (static_cast<void *>(&rng)) RNGType();
    } else {
        new (static_cast<void *>(&rng))
            RNGType(static_cast<typename RNGType::result_type>(param[0]));
    }

    return 0;
}

template <typename RNGType>
inline int mkl_init(
    int method, ::VSLStreamStatePtr stream, int n, const unsigned *param)
{
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;

    if (method == VSL_INIT_METHOD_STANDARD) {
        return mkl_init(
            rng, n, param,
            std::integral_constant<
                bool, (std::is_same<CtrType<RNGType>, NullType>::value ||
                          std::is_same<KeyType<RNGType>, NullType>::value)>());
    }

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
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;
    uniform_real_distribution(rng, static_cast<std::size_t>(n), r, a, b);

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int(::VSLStreamStatePtr stream, int n, unsigned *r)
{
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;
    uniform_bits_distribution(rng, static_cast<std::size_t>(n), r);

    return 0;
}

} // namespace vsmc::internal

} // namespace vsmc

extern "C" {

#include <vsmc/rng/internal/mkl_brng_defines.hpp>

} // extern "C"
