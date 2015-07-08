//============================================================================
// vSMC/include/vsmc/rng/lognormal_distribution.hpp
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

#ifndef VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP
#define VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/normal_distribution.hpp>

namespace vsmc
{

template <typename RealType, typename RNGType>
inline void lognormal_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

/// \brief Lognormal distribution
/// \ingroup Distribution
template <typename RealType>
class LognormalDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = LognormalDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = LognormalDistribution<RealType>;

        explicit param_type(result_type m = 0, result_type s = 1)
            : normal_(m, s)
        {
            invariant();
        }

        result_type m() const { return normal_.mean(); }
        result_type s() const { return normal_.stddev(); }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            return param1.normal_ == param2.normal_;
        }

        friend bool operator!=(
            const param_type &param1, const param_type &param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.normal_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            NormalDistribution<RealType> normal;
            is >> std::ws >> normal;

            if (is.good())
                param.normal_ = std::move(normal);

            return is;
        }

        private:
        NormalDistribution<RealType> normal_;

        friend distribution_type;

        void invariant() {}

        void reset() { normal_.reset(); }
    }; // class param_type

    explicit LognormalDistribution(result_type m = 0, result_type s = 1)
        : param_(m, s)
    {
    }

    explicit LognormalDistribution(const param_type &param) : param_(param) {}

    result_type m() const { return param_.m(); }
    result_type s() const { return param_.s(); }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return std::exp(param_.normal_(rng));
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        lognormal_distribution(rng, n, r, m(), s());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class LognormalDistribution

/// \brief Generating lognormal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void lognormal_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType m, RealType s)
{
    normal_distribution(rng, n, r, m, s);
    exp(n, r, r);
}

} // namespace vsmc

#endif // VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP
