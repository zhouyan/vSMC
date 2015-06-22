//============================================================================
// vSMC/include/vsmc/rng/normal_distribution.hpp
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

#ifndef VSMC_RNG_NORMAL_DISTRIBUTION_HPP
#define VSMC_RNG_NORMAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_NORMAL_DISTRIBUTION_PARAM_CHECK(sigma)        \
    VSMC_RUNTIME_ASSERT((sigma > 0), "**NormalDistribution** CONSTRUCTED "    \
                                     "WITH INVALID STANDARD DEVIATION "       \
                                     "PARAMETER VALUE")

namespace vsmc
{

/// \brief Normal distribution
/// \ingroup Distribution
template <typename RealType>
class NormalDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = NormalDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = NormalDistribution<RealType>;

        explicit param_type(result_type mean = 0, result_type sigma = 1)
            : mean_(mean), sigma_(sigma), u1_(0), u2_(0), saved_(false)
        {
            invariant();
        }

        result_type mean() const { return mean_; }
        result_type sigma() const { return sigma_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.mean_, param2.mean_))
                return false;
            if (!internal::is_equal(param1.sigma_, param2.sigma_))
                return false;
            if (!internal::is_equal(param1.u1_, param2.u1_))
                return false;
            if (!internal::is_equal(param1.u2_, param2.u2_))
                return false;
            if (param1.saved_ && !param2.saved)
                return false;
            if (!param1.saved_ && param2.saved)
                return false;
            return true;
        }

        friend bool operator!=(
            const param_type param1, const param_type param2)
        {
            return !(param1 == param2);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits> &operator<<(
            std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.mean_ << ' ';
            os << param.sigma_ << ' ';
            os << param.u1_ << ' ';
            os << param.u2_ << ' ';
            os << param.saved_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type mean = 0;
            result_type sigma = 0;
            result_type u1 = 0;
            result_type u2 = 0;
            bool saved = false;
            is >> std::ws >> mean;
            is >> std::ws >> sigma;
            is >> std::ws >> u1;
            is >> std::ws >> u2;
            is >> std::ws >> saved;

            if (is.good()) {
                if (sigma > 0) {
                    param = param_type(mean, sigma);
                    param.u1_ = u1;
                    param.u2_ = u2;
                    param.saved_ = saved;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type mean_;
        result_type sigma_;
        result_type u1_;
        result_type u2_;
        bool saved_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_NORMAL_DISTRIBUTION_PARAM_CHECK(sigma_);
            saved_ = false;
        }

        void reset() { saved_ = false; }
    }; // class param_type

    explicit NormalDistribution(result_type mean = 0, result_type sigma = 1)
        : param_(mean, sigma)
    {
    }

    explicit NormalDistribution(const param_type &param) : param_(param) {}

    result_type mean() const { return param_.mean_; }
    result_type sigma() const { return param_.sigma_; }

    result_type min VSMC_MNE() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max VSMC_MNE() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        if (!param_.saved_) {
            UniformRealDistributionType<RNGType, RealType> runif(0, 1);
            param_.u1_ = std::sqrt(-2 * std::log(runif(rng)));
            param_.u2_ = math::pi_2<result_type>() * runif(rng);
        }
        param_.saved_ = !param_.saved_;
        double z = param_.saved_ ? param_.u1_ * std::cos(param_.u2_) :
                                   param_.u1_ * std::sin(param_.u2_);

        return param_.mean_ + param_.sigma_ * z;
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class NormalDistribution

} // namespace vsmc

#endif // VSMC_RNG_NORMAL_DISTRIBUTION_HPP
