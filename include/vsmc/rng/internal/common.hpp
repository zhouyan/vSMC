//============================================================================
// vSMC/include/vsmc/rng/internal/common.hpp
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

#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/simd.hpp>
#if VSMC_HAS_MKL
#include <vsmc/utility/mkl.hpp>
#endif

#define VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS                                \
    param_type param() const { return param_; }                               \
    void param(const param_type &par) { param_ = par; }                       \
    void reset() { param_.reset(); }                                          \
                                                                              \
    friend bool operator==(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param_ == dist2.param_;                                  \
    }                                                                         \
                                                                              \
    friend bool operator!=(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param_ != dist2.param_;                                  \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_ostream<CharT, Traits> &operator<<(                     \
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist) \
    {                                                                         \
        os << dist.param_;                                                    \
                                                                              \
        return os;                                                            \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_istream<CharT, Traits> &operator>>(                     \
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)       \
    {                                                                         \
        is >> dist.param_;                                                    \
                                                                              \
        return is;                                                            \
    }

namespace vsmc
{

namespace internal
{

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(KeyType, key_type, NullType)

#ifdef VSMC_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"
#endif

template <typename T>
inline bool is_equal(const T &a, const T &b)
{
    return a == b;
}

#ifdef VSMC_CLANG
#pragma clang diagnostic pop
#endif

template <int N>
class RNGBitsNMax
{
    public:
    static constexpr std::uint64_t
        value = std::numeric_limits<std::uint64_t>::max() >> (64 - N);
}; // class RNGBitsNMax

template <std::uint64_t UMax, int N>
class RNGBitsN
{
    static constexpr std::uint64_t bmax = RNGBitsNMax<N>::value;

    public:
    static constexpr int value =
        UMax < bmax ? RNGBitsN<UMax, N - 1>::value : N;
}; // class RNGMaxBitsN

template <std::uint64_t UMax>
class RNGBitsN<UMax, 0>
{
    public:
    static constexpr int value = 0;
}; // class RNGMaxBitsN

template <typename RNGType>
class RNGMinBits
    : public std::integral_constant<int,
          RNGBitsN<static_cast<std::uint64_t>(RNGType::min()), 64>::value>
{
}; // class RNGMinBits

template <typename RNGType>
class RNGMaxBits
    : public std::integral_constant<int,
          RNGBitsN<static_cast<std::uint64_t>(RNGType::max()), 64>::value>
{
}; // class RNGMaxBits

template <typename RNGType>
class RNGBits : public std::integral_constant<int,
                    RNGMaxBits<RNGType>::value - RNGMinBits<RNGType>::value>
{
}; // class RNGBits

template <typename SeedSeq, typename U, typename V = U, typename W = V>
class is_seed_seq
    : public std::integral_constant<bool,
          !std::is_convertible<SeedSeq, U>::value &&
              !std::is_convertible<SeedSeq, V>::value &&
              !std::is_convertible<SeedSeq, W>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                                        U>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                                        V>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type, W>::value>
{
}; // class is_seed_seq

} // namespace vsmc::internal

/// \brief Generate random bits
/// \ingroup RNG
template <typename RNGType>
void rng_rand(RNGType &rng, std::size_t n, typename RNGType::result_type *r)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = rng();
}

/// \brief Generate random distribution variates
/// \ingroup RNG
template <typename RNGType, typename DistributionType>
void rng_rand(RNGType &rng, DistributionType &dist, std::size_t n,
    typename DistributionType::result_type *r)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = dist(rng);
}

template <typename>
class CounterEngine;

template <typename Generator>
inline void rng_rand(CounterEngine<Generator> &, std::size_t,
    typename CounterEngine<Generator>::result_type *);

template <typename = int>
class DiscreteDistribution;

template <typename = double>
class BetaDistribution;

template <typename = double>
class CauchyDistribution;

template <typename = double>
class ChiSquaredDistribution;

template <typename = double>
class ExponentialDistribution;

template <typename = double>
class ExtremeValueDistribution;

template <typename = double>
class FisherFDistribution;

template <typename = double>
class GammaDistribution;

template <typename = double>
class LaplaceDistribution;

template <typename = double>
class LevyDistribution;

template <typename = double>
class LogisticDistribution;

template <typename = double>
class LognormalDistribution;

template <typename = double>
class NormalDistribution;

template <typename = double>
class ParetoDistribution;

template <typename = double>
class RayleighDistribution;

template <typename = double>
class StudentTDistribution;

template <typename, typename, typename>
class U01LRDistribution;

template <typename, typename, typename>
class UniformRealLRDistribution;

template <typename = double>
class WeibullDistribution;

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, BetaDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, CauchyDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, ChiSquaredDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, DiscreteDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, ExponentialDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, ExtremeValueDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, FisherFDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, GammaDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, LaplaceDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, LevyDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, LogisticDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, LognormalDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, NormalDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, ParetoDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, RayleighDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, StudentTDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType, typename Left, typename Right>
inline void rng_rand(RNGType &, U01LRDistribution<RealType, Left, Right> &,
    std::size_t, RealType *);

template <typename RealType, typename RNGType, typename Left, typename Right>
inline void rng_rand(RNGType &,
    UniformRealLRDistribution<RealType, Left, Right> &, std::size_t,
    RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, WeibullDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void beta_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void cauchy_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void chi_squared_distribution(
    RNGType &, std::size_t, RealType *, RealType);

template <typename RealType, typename RNGType>
inline void exponential_distribution(
    RNGType &, std::size_t, RealType *, RealType);

template <typename RealType, typename RNGType>
inline void extreme_value_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void fisher_f_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void gamma_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void laplace_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void levy_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void logistic_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void lognormal_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void normal_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void pareto_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void rayleigh_distribution(
    RNGType &, std::size_t, RealType *, RealType);

template <typename RealType, typename RNGType>
inline void student_t_distribution(
    RNGType &, std::size_t, RealType *, RealType);

template <typename RealType, typename RNGType>
inline void u01_distribution(RNGType &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void u01_lr_distribution(RNGType &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void uniform_real_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void uniform_real_lr_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void weibull_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

#if VSMC_HAS_MKL

template <MKL_INT, std::size_t>
class MKLEngine;

template <MKL_INT BRNG, std::size_t Bits>
inline void rng_rand(MKLEngine<BRNG, Bits> &, std::size_t,
    typename MKLEngine<BRNG, Bits>::result_type *);

template <MKL_INT BRNG, std::size_t Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void u01_distribution(MKLEngine<BRNG, Bits> &, std::size_t, float *);

template <MKL_INT BRNG, std::size_t Bits>
inline void u01_distribution(MKLEngine<BRNG, Bits> &, std::size_t, double *);

template <MKL_INT BRNG, std::size_t Bits>
inline void normal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *r, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void normal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *r, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void laplace_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void laplace_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void gamma_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void gamma_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, std::size_t Bits>
inline void beta_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, std::size_t Bits>
inline void beta_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

#endif // VSMC_HAS_MKL

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
