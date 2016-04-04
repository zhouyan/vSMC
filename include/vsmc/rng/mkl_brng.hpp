//============================================================================
// vSMC/include/vsmc/rng/mkl_brng.hpp
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

#ifndef VSMC_RNG_MKL_BRNG_HPP
#define VSMC_RNG_MKL_BRNG_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/engine.hpp>
#include <vsmc/rng/mkl.hpp>
#include <vsmc/vsmc.h>

namespace vsmc
{

/// \brief Register a C++11 RNG as MKL BRNG
/// \ingroup MKLRNG
///
/// \details
/// Only engines defined in this library and the standard library are
/// specialized. This function requires the C runtime of the library.
template <typename RNGType>
int mkl_brng();

#ifdef VSMC_RNG_DEFINE_MACRO
#undef VSMC_RNG_DEFINE_MACRO
#endif

#define VSMC_RNG_DEFINE_MACRO(RNGType, name)                                  \
    template <>                                                               \
    inline int mkl_brng<RNGType>()                                            \
    {                                                                         \
        static const int brng = ::vsmc_mkl_brng_##name();                     \
                                                                              \
        return brng;                                                          \
    }

#include <vsmc/rng/internal/rng_define_macro.hpp>

/// \brief Consruct a MKLGenerator given RNG type
/// \ingroup MKLRNG
template <typename RNGType>
class MKLAdaptor : public MKLGenerator<0>
{
    public:
    explicit MKLAdaptor(MKL_UINT s = 1)
        : MKLGenerator<0>(mkl_brng<RNGType>(), s)
    {
    }

    template <typename SeedSeq>
    explicit MKLAdaptor(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_UINT,
            MKLAdaptor<RNGType>>::value>::type * = nullptr)
        : MKLGenerator<0>(mkl_brng<RNGType>(), seq)
    {
    }

    void seed(MKL_UINT s) { MKLGenerator<0>::seed(s); }

    template <typename SeedSeq>
    void seed(SeedSeq &seq,
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, MKL_UINT>::value>::type * = nullptr)
    {
        MKLGenerator<0>::seed(this->stream().get_brng(), seq);
    }
}; // class MKLAdaptor

template <typename RNGType>
inline void rng_rand(MKLAdaptor<RNGType> &rng, std::size_t n,
    typename MKLAdaptor<RNGType>::result_type *r)
{
    rng(n, r);
}

template <typename RNGType>
inline void beta_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float alpha, float beta)
{
    internal::size_check<MKL_INT>(n, "beta_distribution)");
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <typename RNGType>
inline void beta_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    internal::size_check<MKL_INT>(n, "beta_distribution)");
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <typename RNGType>
inline void cauchy_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "cauchy_distribution)");
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <typename RNGType>
inline void cauchy_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "cauchy_distribution)");
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <typename RNGType>
inline void exponential_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float lambda)
{
    internal::size_check<MKL_INT>(n, "exponential_distribution)");
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <typename RNGType>
inline void exponential_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double lambda)
{
    internal::size_check<MKL_INT>(n, "exponential_distribution)");
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <typename RNGType>
inline void extreme_value_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "extreme_value_distribution)");
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <typename RNGType>
inline void extreme_value_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "extreme_value_distribution)");
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <typename RNGType>
inline void gamma_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float alpha, float beta)
{
    internal::size_check<MKL_INT>(n, "gamma_distribution)");
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <typename RNGType>
inline void gamma_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    internal::size_check<MKL_INT>(n, "gamma_distribution)");
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <typename RNGType>
inline void laplace_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    float *r, float location, float scale)
{
    internal::size_check<MKL_INT>(n, "lapace_distribution)");
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <typename RNGType>
inline void laplace_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    double *r, double location, double scale)
{
    internal::size_check<MKL_INT>(n, "lapace_distribution)");
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <typename RNGType>
inline void lognormal_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float m, float s)
{
    internal::size_check<MKL_INT>(n, "lognormal_distribution)");
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <typename RNGType>
inline void lognormal_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double m, double s)
{
    internal::size_check<MKL_INT>(n, "lognormal_distribution)");
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <typename RNGType>
inline void normal_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    float *r, float mean, float stddev)
{
    internal::size_check<MKL_INT>(n, "normal_distribution)");
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <typename RNGType>
inline void normal_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    double *r, double mean, double stddev)
{
    internal::size_check<MKL_INT>(n, "normal_distribution)");
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <typename RNGType>
inline void normal_mv_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    float *r, std::size_t m, const float *mean, const float *chol)
{
    internal::size_check<MKL_INT>(n, "normal_mv_distribution)");
    internal::size_check<MKL_INT>(m, "normal_mv_distribution)");
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <typename RNGType>
inline void normal_mv_distribution(MKLAdaptor<RNGType> &rng, std::size_t n,
    double *r, std::size_t m, const double *mean, const double *chol)
{
    internal::size_check<MKL_INT>(n, "normal_mv_distribution)");
    internal::size_check<MKL_INT>(m, "normal_mv_distribution)");
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <typename RNGType>
inline void rayleigh_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float sigma)
{
    internal::size_check<MKL_INT>(n, "rayleigh_distribution)");
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<float>() * sigma);
}

template <typename RNGType>
inline void rayleigh_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double sigma)
{
    internal::size_check<MKL_INT>(n, "rayleigh_distribution)");
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<double>() * sigma);
}

template <typename RNGType>
inline void u01_distribution(MKLAdaptor<RNGType> &rng, std::size_t n, float *r)
{
    internal::size_check<MKL_INT>(n, "u01_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <typename RNGType>
inline void u01_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r)
{
    internal::size_check<MKL_INT>(n, "u01_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <typename RNGType>
inline void uniform_real_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "uniform_real_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <typename RNGType>
inline void uniform_real_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "uniform_real_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <typename RNGType>
inline void weibull_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "weibull_distribution)");
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

template <typename RNGType>
inline void weibull_distribution(
    MKLAdaptor<RNGType> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "weibull_distribution)");
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

} // namespace vsmc

#endif // VSMC_RNG_MKL_BRNG_HPP
