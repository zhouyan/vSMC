//============================================================================
// vSMC/include/vsmc/rng/internal/common.hpp
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

#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/simd.hpp>
#if VSMC_HAS_MKL
#include <vsmc/utility/mkl.hpp>
#endif

#define VSMC_RUNTIME_ASSERT_RNG_DISTRIBUTION_PARAM(flag, Name)                \
    VSMC_RUNTIME_ASSERT((flag),                                               \
        "**" #Name "Distribution** CONSTRUCTED WITH INVALID PARAMETERS")

#define VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_0(Name, T, t, Type)           \
    public:                                                                   \
    class param_type                                                          \
    {                                                                         \
        static_assert(std::is_##t<T>::value,                                  \
            "**" #Name "Distribution::param_type** USED WITH " #T             \
            " OTHER THAN " #Type " INTEGER TYPES");                           \
                                                                              \
        public:                                                               \
        using result_type = T;                                                \
        using distribution_type = Name##Distribution<T>;                      \
                                                                              \
        friend bool operator==(const param_type &, const param_type &)        \
        {                                                                     \
            return true;                                                      \
        }                                                                     \
                                                                              \
        friend bool operator!=(const param_type &, const param_type &)        \
        {                                                                     \
            return false;                                                     \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_ostream<CharT, Traits> &operator<<(                 \
            std::basic_ostream<CharT, Traits> &os, const param_type &)        \
        {                                                                     \
            return os;                                                        \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_istream<CharT, Traits> &operator>>(                 \
            std::basic_istream<CharT, Traits> &is, param_type &)              \
        {                                                                     \
            return is;                                                        \
        }                                                                     \
                                                                              \
        private:                                                              \
        friend distribution_type;                                             \
    }; // class param_type

#define VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_1(Name, name, p1, v1)         \
    public:                                                                   \
    class param_type                                                          \
    {                                                                         \
        static_assert(std::is_floating_point<RealType>::value,                \
            "**" #Name "Distribution::param_type** USED WITH RealType OTHER " \
            "THAN FLOATING POINT TYPES");                                     \
                                                                              \
        public:                                                               \
        using result_type = RealType;                                         \
        using distribution_type = Name##Distribution<RealType>;               \
                                                                              \
        explicit param_type(result_type p1 = v1) : p1##_(p1)                  \
        {                                                                     \
            VSMC_RUNTIME_ASSERT_RNG_DISTRIBUTION_PARAM(                       \
                internal::name##_distribution_check_param(p1), Name);         \
        }                                                                     \
                                                                              \
        result_type p1() const { return p1##_; }                              \
                                                                              \
        friend bool operator==(                                               \
            const param_type &param1, const param_type &param2)               \
        {                                                                     \
            if (!internal::is_equal(param1.p1##_, param2.p1##_))              \
                return false;                                                 \
            return true;                                                      \
        }                                                                     \
                                                                              \
        friend bool operator!=(                                               \
            const param_type &param1, const param_type &param2)               \
        {                                                                     \
            return !(param1 == param2);                                       \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_ostream<CharT, Traits> &operator<<(                 \
            std::basic_ostream<CharT, Traits> &os, const param_type &param)   \
        {                                                                     \
            if (!os.good())                                                   \
                return os;                                                    \
                                                                              \
            os << param.p1##_;                                                \
                                                                              \
            return os;                                                        \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_istream<CharT, Traits> &operator>>(                 \
            std::basic_istream<CharT, Traits> &is, param_type &param)         \
        {                                                                     \
            if (!is.good())                                                   \
                return is;                                                    \
                                                                              \
            result_type p1 = 0;                                               \
            is >> std::ws >> p1;                                              \
                                                                              \
            if (is.good()) {                                                  \
                if (internal::name##_distribution_check_param(p1))            \
                    param.p1##_ = p1;                                         \
                else                                                          \
                    is.setstate(std::ios_base::failbit);                      \
            }                                                                 \
                                                                              \
            return is;                                                        \
        }                                                                     \
                                                                              \
        private:                                                              \
        result_type p1##_;                                                    \
                                                                              \
        friend distribution_type;                                             \
    }; // class param_type

#define VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_2(Name, name, p1, v1, p2, v2) \
    public:                                                                   \
    class param_type                                                          \
    {                                                                         \
        static_assert(std::is_floating_point<RealType>::value,                \
            "**" #Name "Distribution::param_type** USED WITH RealType OTHER " \
            "THAN FLOATING POINT TYPES");                                     \
                                                                              \
        public:                                                               \
        using result_type = RealType;                                         \
        using distribution_type = Name##Distribution<RealType>;               \
                                                                              \
        explicit param_type(result_type p1 = v1, result_type p2 = v2)         \
            : p1##_(p1), p2##_(p2)                                            \
        {                                                                     \
            VSMC_RUNTIME_ASSERT_RNG_DISTRIBUTION_PARAM(                       \
                internal::name##_distribution_check_param(p1, p2), Name);     \
        }                                                                     \
                                                                              \
        result_type p1() const { return p1##_; }                              \
        result_type p2() const { return p2##_; }                              \
                                                                              \
        friend bool operator==(                                               \
            const param_type &param1, const param_type &param2)               \
        {                                                                     \
            if (!internal::is_equal(param1.p1##_, param2.p1##_))              \
                return false;                                                 \
            if (!internal::is_equal(param1.p2##_, param2.p2##_))              \
                return false;                                                 \
            return true;                                                      \
        }                                                                     \
                                                                              \
        friend bool operator!=(                                               \
            const param_type &param1, const param_type &param2)               \
        {                                                                     \
            return !(param1 == param2);                                       \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_ostream<CharT, Traits> &operator<<(                 \
            std::basic_ostream<CharT, Traits> &os, const param_type &param)   \
        {                                                                     \
            if (!os.good())                                                   \
                return os;                                                    \
                                                                              \
            os << param.p1##_ << ' ';                                         \
            os << param.p2##_;                                                \
                                                                              \
            return os;                                                        \
        }                                                                     \
                                                                              \
        template <typename CharT, typename Traits>                            \
        friend std::basic_istream<CharT, Traits> &operator>>(                 \
            std::basic_istream<CharT, Traits> &is, param_type &param)         \
        {                                                                     \
            if (!is.good())                                                   \
                return is;                                                    \
                                                                              \
            result_type p1 = 0;                                               \
            result_type p2 = 0;                                               \
            is >> std::ws >> p1;                                              \
            is >> std::ws >> p2;                                              \
                                                                              \
            if (is.good()) {                                                  \
                if (internal::name##_distribution_check_param(p1, p2)) {      \
                    param.p1##_ = p1;                                         \
                    param.p2##_ = p2;                                         \
                } else {                                                      \
                    is.setstate(std::ios_base::failbit);                      \
                }                                                             \
            }                                                                 \
                                                                              \
            return is;                                                        \
        }                                                                     \
                                                                              \
        private:                                                              \
        result_type p1##_;                                                    \
        result_type p2##_;                                                    \
                                                                              \
        friend distribution_type;                                             \
    }; // class param_type

#define VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_0(Name, T)                   \
    public:                                                                   \
    using result_type = T;                                                    \
    using distribution_type = Name##Distribution<T>;                          \
                                                                              \
    Name##Distribution() = default;                                           \
    explicit Name##Distribution(const param_type &) {}

#define VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_1(Name, p1, v1)              \
    public:                                                                   \
    using result_type = RealType;                                             \
    using distribution_type = Name##Distribution<RealType>;                   \
                                                                              \
    explicit Name##Distribution(result_type p1 = v1) : param_(p1)             \
    {                                                                         \
        reset();                                                              \
    }                                                                         \
                                                                              \
    explicit Name##Distribution(const param_type &param) : param_(param)      \
    {                                                                         \
        reset();                                                              \
    }                                                                         \
                                                                              \
    result_type p1() const { return param_.p1(); }

#define VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_2(Name, p1, v1, p2, v2)      \
    public:                                                                   \
    using result_type = RealType;                                             \
    using distribution_type = Name##Distribution<RealType>;                   \
                                                                              \
    explicit Name##Distribution(result_type p1 = v1, result_type p2 = v2)     \
        : param_(p1, p2)                                                      \
    {                                                                         \
        reset();                                                              \
    }                                                                         \
                                                                              \
    explicit Name##Distribution(const param_type &param) : param_(param)      \
    {                                                                         \
        reset();                                                              \
    }                                                                         \
                                                                              \
    result_type p1() const { return param_.p1(); }                            \
    result_type p2() const { return param_.p2(); }

#define VSMC_DEFINE_RNG_DISTRIBUTION_OPERATOR(Name, name)                     \
    public:                                                                   \
    const param_type &param() const { return param_; }                        \
                                                                              \
    void param(const param_type &param)                                       \
    {                                                                         \
        param_ = param;                                                       \
        reset();                                                              \
    }                                                                         \
                                                                              \
    void pram(param_type &&param)                                             \
    {                                                                         \
        param_ = std::move(param);                                            \
        reset();                                                              \
    }                                                                         \
                                                                              \
    template <typename RNGType>                                               \
    result_type operator()(RNGType &rng)                                      \
    {                                                                         \
        return operator()(rng, param_);                                       \
    }                                                                         \
                                                                              \
    template <typename RNGType>                                               \
    result_type operator()(RNGType &rng, const param_type &param)             \
    {                                                                         \
        return generate(rng, param);                                          \
    }                                                                         \
                                                                              \
    template <typename RNGType>                                               \
    void operator()(RNGType &rng, std::size_t n, result_type *r)              \
    {                                                                         \
        operator()(rng, n, r, param_);                                        \
    }                                                                         \
                                                                              \
    template <typename RNGType>                                               \
    void operator()(                                                          \
        RNGType &rng, std::size_t n, result_type *r, const param_type &param) \
    {                                                                         \
        if (n < 100) {                                                        \
            for (std::size_t i = 0; i != n; ++i)                              \
                r[i] = operator()(rng, param);                                \
        } else {                                                              \
            name##_distribution(rng, n, r, param);                            \
        }                                                                     \
    }                                                                         \
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
        return !(dist1 == dist2);                                             \
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
        is >> std::ws >> dist.param_;                                         \
        if (is.good())                                                        \
            dist.reset();                                                     \
                                                                              \
        return is;                                                            \
    }                                                                         \
                                                                              \
    private:                                                                  \
    param_type param_;

#define VSMC_DEFINE_RNG_DISTRIBUTION_0(Name, name, T, t, Type)                \
    VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_0(Name, T, t, Type)               \
    VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_0(Name, T)                       \
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATOR(Name, name)

#define VSMC_DEFINE_RNG_DISTRIBUTION_1(Name, name, p1, v1)                    \
    VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_1(Name, name, p1, v1)             \
    VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_1(Name, p1, v1)                  \
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATOR(Name, name)

#define VSMC_DEFINE_RNG_DISTRIBUTION_2(Name, name, p1, v1, p2, v2)            \
    VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_2(Name, name, p1, v1, p2, v2)     \
    VSMC_DEFINE_RNG_DISTRIBUTION_CONSTRUCTOR_2(Name, p1, v1, p2, v2)          \
    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATOR(Name, name)

#define VSMC_DEFINE_RNG_DISTRIBUTION_RAND_0(Name, name, T)                    \
    template <typename T, typename RNGType>                                   \
    inline void name##_distribution(RNGType &rng, std::size_t n, T *r,        \
        const typename Name##Distribution<T>::param_type &)                   \
    {                                                                         \
        name##_distribution(rng, n, r);                                       \
    }                                                                         \
                                                                              \
    template <typename T, typename RNGType>                                   \
    inline void rng_rand(                                                     \
        RNGType &rng, Name##Distribution<T> &dist, std::size_t n, T *r)       \
    {                                                                         \
        dist(rng, n, r);                                                      \
    }

#define VSMC_DEFINE_RNG_DISTRIBUTION_RAND_1(Name, name, p1)                   \
    template <typename RealType, typename RNGType>                            \
    inline void name##_distribution(RNGType &rng, std::size_t n, RealType *r, \
        const typename Name##Distribution<RealType>::param_type &param)       \
    {                                                                         \
        name##_distribution(rng, n, r, param.p1());                           \
    }                                                                         \
                                                                              \
    template <typename RealType, typename RNGType>                            \
    inline void rng_rand(RNGType &rng, Name##Distribution<RealType> &dist,    \
        std::size_t n, RealType *r)                                           \
    {                                                                         \
        dist(rng, n, r);                                                      \
    }

#define VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(Name, name, p1, p2)               \
    template <typename RealType, typename RNGType>                            \
    inline void name##_distribution(RNGType &rng, std::size_t n, RealType *r, \
        const typename Name##Distribution<RealType>::param_type &param)       \
    {                                                                         \
        name##_distribution(rng, n, r, param.p1(), param.p2());               \
    }                                                                         \
                                                                              \
    template <typename RealType, typename RNGType>                            \
    inline void rng_rand(RNGType &rng, Name##Distribution<RealType> &dist,    \
        std::size_t n, RealType *r)                                           \
    {                                                                         \
        dist(rng, n, r);                                                      \
    }

namespace vsmc
{

namespace internal
{

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(KeyType, key_type, NullType)

template <typename UIntType, UIntType U,
    int N = std::numeric_limits<UIntType>::digits>
class RNGMinBitsImpl
{
    static constexpr int M = std::numeric_limits<UIntType>::digits - N;

    public:
    static constexpr int value =
        (U >> M) == 0 ? M : RNGMinBitsImpl<UIntType, U, N - 1>::value;
}; // class RNGMinBitsImpl

template <typename UIntType, UIntType U>
class RNGMinBitsImpl<UIntType, U, 0>
{
    public:
    static constexpr int value = std::numeric_limits<UIntType>::digits;
}; // class RNGMinBitsImpl

} // namespace vsmc::internal

/// \brief Find the smallest N such that `(RNGType::min() >> N) == 0`
/// \ingroup RNG
template <typename RNGType>
class RNGMinBits : public std::integral_constant<int,
                       internal::RNGMinBitsImpl<typename RNGType::result_type,
                                                     RNGType::min()>::value>
{
}; // class RNGMinBits

namespace internal
{

template <typename UIntType, UIntType U,
    int N = std::numeric_limits<UIntType>::digits>
class RNGMaxBitsImpl
{
    static constexpr UIntType bmax = std::numeric_limits<UIntType>::max() >>
        (std::numeric_limits<UIntType>::digits - N);

    public:
    static constexpr int value =
        U < bmax ? RNGMaxBitsImpl<UIntType, U, N - 1>::value : N;
}; // class RNGMaxBitsImpl

template <typename UIntType, UIntType U>
class RNGMaxBitsImpl<UIntType, U, 0>
{
    public:
    static constexpr int value = 0;
}; // class RNGMaxBitsImpl

} // namespace vsmc::internal

/// \brief Find the largest N such that
/// RNGType::max() >= (M >> (W - N)) where
/// M = std::numeric_limits<typename RNGType::result_type>::max()
/// W = std::numeric_limits<typename RNGType::result_type>::digits
/// \ingroup RNG
template <typename RNGType>
class RNGMaxBits : public std::integral_constant<int,
                       internal::RNGMaxBitsImpl<typename RNGType::result_type,
                                                     RNGType::max()>::value>
{
}; // class RNGMaxBits

/// \brief The value of
/// RNGMaxBits<RNGType>::value - RNGMinBits<RNGType>::value
/// \ingroup RNG
///
/// \details
/// Let R = RNGMinBits<RNGType>::value.
/// Let P = RNGMaxBits<RNGType>::vaue - RNGMinBits<RNGType>::value
/// Then given `u` is the unsigned random integer generated by `RNGType`,
/// `(u >> R)` is an unsigned random integer covering at least
/// \f$\{0,\dots,2^P - 1\}\f$.
template <typename RNGType>
class RNGBits : public std::integral_constant<int,
                    RNGMaxBits<RNGType>::value - RNGMinBits<RNGType>::value>
{
}; // class RNGBits

/// \brief Parameter type for open interval
/// \ingroup RNG
class Open;

/// \brief Parameter type for closed interval
/// \ingroup RNG
class Closed;

/// \brief Generate random bits
/// \ingroup RNG
template <typename RNGType>
void rng_rand(RNGType &rng, std::size_t n, typename RNGType::result_type *r)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = rng();
}

template <typename>
class CounterEngine;

template <typename = double, std::size_t = Dynamic>
class RandomWalk;

template <typename = double, std::size_t = Dynamic, std::size_t = Dynamic>
class RandomWalkG;

template <typename = double>
class NormalProposal;

template <typename = double, std::size_t = Dynamic>
class NormalMVProposal;

template <typename = double>
class BetaDistribution;

template <typename = double>
class CauchyDistribution;

template <typename = double>
class ChiSquaredDistribution;

template <typename = int>
class DiscreteDistribution;

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

template <typename = double, std::size_t = Dynamic>
class NormalMVDistribution;

template <typename = double>
class ParetoDistribution;

template <typename = double>
class RayleighDistribution;

template <typename = double>
class StudentTDistribution;

template <typename = double>
class U01Distribution;

template <typename = double, typename = Open, typename = Closed>
class U01LRDistribution;

template <typename = unsigned>
class UniformBitsDistribution;

template <typename = double>
class UniformRealDistribution;

template <typename = double>
class WeibullDistribution;

template <typename Generator>
inline void rng_rand(CounterEngine<Generator> &, std::size_t,
    typename CounterEngine<Generator>::result_type *);

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

template <typename RealType, std::size_t Dim, typename RNGType>
inline void rng_rand(
    RNGType &, NormalMVDistribution<RealType, Dim> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, ParetoDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, RayleighDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, StudentTDistribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, U01Distribution<RealType> &, std::size_t, RealType *);

template <typename RealType, typename RNGType, typename Left, typename Right>
inline void rng_rand(RNGType &, U01LRDistribution<RealType, Left, Right> &,
    std::size_t, RealType *);

template <typename UIntType, typename RNGType>
inline void rng_rand(
    RNGType &, UniformBitsDistribution<UIntType> &, std::size_t, UIntType *);

template <typename RealType, typename RNGType>
inline void rng_rand(
    RNGType &, UniformRealDistribution<RealType> &, std::size_t, RealType *);

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
inline void normal_mv_distribution(RNGType &, std::size_t, RealType *,
    std::size_t, const RealType *, const RealType *);

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

template <typename, typename, typename RealType, typename RNGType>
inline void u01_lr_distribution(RNGType &, std::size_t, RealType *);

template <typename UIntType, typename RNGType>
inline void uniform_bits_distribution(RNGType &, std::size_t, UIntType *);

template <typename RealType, typename RNGType>
inline void uniform_real_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

template <typename RealType, typename RNGType>
inline void weibull_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

#if VSMC_HAS_MKL

template <MKL_INT, int>
class MKLEngine;

template <MKL_INT BRNG, int Bits>
inline void rng_rand(MKLEngine<BRNG, Bits> &, std::size_t,
    typename MKLEngine<BRNG, Bits>::result_type *);

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float);

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double);

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *r, float, float);

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *r, double, double);

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &, std::size_t,
    float *, std::size_t, const float *, const float *);

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &, std::size_t,
    double *, std::size_t, const double *, const double *);

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float);

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double);

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(MKLEngine<BRNG, Bits> &, std::size_t, float *);

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(MKLEngine<BRNG, Bits> &, std::size_t, double *);

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, float *, float, float);

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &, std::size_t, double *, double, double);

#endif // VSMC_HAS_MKL

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
