//============================================================================
// vSMC/include/vsmc/rng/normal_distribution.hpp
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

#ifndef VSMC_RNG_NORMAL_DISTRIBUTION_HPP
#define VSMC_RNG_NORMAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>

namespace vsmc
{

namespace internal
{

template <typename RealType>
inline bool normal_distribution_check_param(RealType, RealType stddev)
{
    return stddev > 0;
}

} // namespace vsmc::internal

/// \brief Normal distribution
/// \ingroup Distribution
template <typename RealType>
class NormalDistribution
{
    VSMC_DEFINE_RNG_DISTRIBUTION_PARAM_TYPE_2(
        Normal, normal, mean, 0, stddev, 1)

    public:
    using result_type = RealType;
    using distribution_type = NormalDistribution<RealType>;

    explicit NormalDistribution(result_type mean = 0, result_type stddev = 1)
        : param_(mean, stddev), v_(0), saved_(false)
    {
        reset();
    }

    explicit NormalDistribution(const param_type &param)
        : param_(param), v_(0), saved_(false)
    {
        reset();
    }

    result_type mean() const { return param_.mean(); }

    result_type stddev() const { return param_.stddev(); }

    result_type min() const
    {
        return std::numeric_limits<result_type>::lowest();
    }

    result_type max() const { return std::numeric_limits<result_type>::max(); }

    void reset()
    {
        v_ = 0;
        saved_ = false;
    }

    const param_type &param() const { return param_; }

    void param(const param_type &param)
    {
        param_ = param;
        reset();
    }

    void pram(param_type &&param)
    {
        param_ = std::move(param);
        reset();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return operator()(rng, param_);
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng, const param_type &param)
    {
        return generate(rng, param);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        operator()(rng, n, r, param_);
    }

    template <typename RNGType>
    void operator()(
        RNGType &rng, std::size_t n, result_type *r, const param_type &param)
    {
        if (n < 100) {
            for (std::size_t i = 0; i != n; ++i)
                r[i] = operator()(rng, param);
        } else {
            normal_distribution(rng, n, r, param);
        }
    }

    friend bool operator==(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        if (dist1.param_ != dist2.param_)
            return false;
        if (!internal::is_equal(dist1.v_, dist2.v_))
            return false;
        if (dist1.saved_ && !dist2.saved_)
            return false;
        if (!dist1.saved_ && dist2.saved_)
            return false;
        return true;
    }

    friend bool operator!=(
        const distribution_type &dist1, const distribution_type &dist2)
    {
        return !(dist1 == dist2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist)
    {
        if (!os)
            return os;

        os << dist.param_ << ' ';
        os << dist.v_ << ' ';
        os << dist.saved_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)
    {
        if (!is)
            return is;

        param_type param;
        result_type v;
        bool saved;
        is >> std::ws >> param;
        is >> std::ws >> v;
        is >> std::ws >> saved;
        if (static_cast<bool>(is)) {
            dist.param_ = param;
            dist.v_ = v;
            dist.saved_ = saved;
        }

        return is;
    }

    private:
    param_type param_;
    result_type v_;
    bool saved_;

    template <typename RNGType>
    result_type generate(RNGType &rng, const param_type &param)
    {
        result_type z = 0;
        if (saved_) {
            z = v_;
            saved_ = false;
        } else {
            U01Distribution<RealType> u01;
            result_type u1 = std::sqrt(-2 * std::log(u01(rng)));
            result_type u2 = const_pi_2<result_type>() * u01(rng);
            z = u1 * std::cos(u2);
            v_ = u1 * std::sin(u2);
            saved_ = true;
        }

        return param.mean() + param.stddev() * z;
    }
}; // class NormalDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void normal_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType mean, RealType stddev)
{
    alignas(32) RealType s[K / 2];
    const std::size_t nu = n / 2;
    RealType *const u1 = r;
    RealType *const u2 = r + nu;
    u01_distribution(rng, n, r);
    log(nu, u1, s);
    mul(nu, static_cast<RealType>(-2), s, s);
    sqrt(nu, s, s);
    mul(nu, const_pi_2<RealType>(), u2, u2);
    sincos(nu, u2, u1, u2);
    mul(nu, stddev, s, s);
    fma(nu, s, u1, mean, u1);
    fma(nu, s, u2, mean, u2);
}

} // namespace vsmc::internal

/// \brief Generating Normal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void normal_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType mean, RealType stddev)
{
    static_assert(std::is_floating_point<RealType>::value,
        "**normal_distribution** USED WITH RealType OTHER THAN FLOATING POINT "
        "TYPES");

    const std::size_t k = internal::StaticBufferSize<RealType>::value;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i, r += k)
        internal::normal_distribution_impl<k>(rng, k, r, mean, stddev);
    internal::normal_distribution_impl<k>(rng, l, r, mean, stddev);
    if (n % 2 != 0) {
        U01Distribution<RealType> u01;
        RealType u = u01(rng);
        RealType v = u01(rng);
        r[l - 1] = mean +
            stddev * std::sqrt(-2 * std::log(u)) *
                std::cos(const_pi_2<RealType>() * v);
    }
}

VSMC_DEFINE_RNG_DISTRIBUTION_RAND_2(Normal, normal, mean, stddev)

} // namespace vsmc

#endif // VSMC_RNG_NORMAL_DISTRIBUTION_HPP
