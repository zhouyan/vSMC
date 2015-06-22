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

#define VSMC_RUNTIME_ASSERT_RNG_LOGNORMAL_DISTRIBUTION_PARAM_CHECK(s)         \
    VSMC_RUNTIME_ASSERT((s > 0), "**LogNormalDistribution** CONSTRUCTED "     \
                                 "WITH INVALID SCALE PARAMETER VALUE")

namespace vsmc
{

/// \brief Generating log-normal random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
void lognormal_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType m = 0, RealType s = 1)
{
    normal_distribution(rng, n, r, m, s);
    math::vExp(n, r, r);
}

/// \brief LogNormal distribution
/// \ingroup Distribution
template <typename RealType>
class LogNormalDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = LogNormalDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = LogNormalDistribution<RealType>;

        explicit param_type(result_type m = 0, result_type s = 1)
            : m_(m), s_(s), u_(0), v_(0), saved_(false)
        {
            invariant();
        }

        result_type m() const { return m_; }
        result_type s() const { return s_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.m_, param2.m_))
                return false;
            if (!internal::is_equal(param1.s_, param2.s_))
                return false;
            if (!internal::is_equal(param1.u_, param2.u_))
                return false;
            if (!internal::is_equal(param1.v_, param2.v_))
                return false;
            if (param1.saved_ && !param2.saved)
                return false;
            if (!param1.saved_ && param2.saved)
                return false;
            return true;
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

            os << param.m_ << ' ';
            os << param.s_ << ' ';
            os << param.u_ << ' ';
            os << param.v_ << ' ';
            os << param.saved_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type m = 0;
            result_type s = 0;
            result_type u = 0;
            result_type v = 0;
            bool saved = false;
            is >> std::ws >> m;
            is >> std::ws >> s;
            is >> std::ws >> u;
            is >> std::ws >> v;
            is >> std::ws >> saved;

            if (is.good()) {
                if (s > 0) {
                    param = param_type(m, s);
                    param.u_ = u;
                    param.v_ = v;
                    param.saved_ = saved;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type m_;
        result_type s_;
        result_type u_;
        result_type v_;
        bool saved_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_NORMAL_DISTRIBUTION_PARAM_CHECK(s_);
            saved_ = false;
        }

        void reset() { saved_ = false; }
    }; // class param_type

    explicit LogNormalDistribution(result_type m = 0, result_type s = 1)
        : param_(m, s)
    {
    }

    explicit LogNormalDistribution(const param_type &param) : param_(param) {}

    result_type m() const { return param_.m_; }
    result_type s() const { return param_.s_; }

    result_type min VSMC_MNE() const { return 0; }

    result_type max VSMC_MNE() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        if (!param_.saved_) {
            UniformRealDistributionType<RNGType, RealType> runif(-1, 1);
            result_type u = 0;
            result_type v = 0;
            result_type s = 0;
            do {
                u = runif(rng);
                v = runif(rng);
                s = u * u + v * v;
            } while (s > 1 || s <= 0);
            s = std::sqrt(-2 * std::log(s) / s);
            param_.u_ = u * s;
            param_.v_ = v * s;
        }
        param_.saved_ = !param_.saved_;
        double z = param_.saved_ ? param_.u_ : param_.v_;

        return std::exp(param_.m_ + param_.s_ * z);
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class LogNormalDistribution

} // namespace vsmc

#endif // VSMC_RNG_LOGNORMAL_DISTRIBUTION_HPP
