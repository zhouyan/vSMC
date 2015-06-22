//============================================================================
// vSMC/include/vsmc/rng/exponential_distribution.hpp
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

#ifndef VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP
#define VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01_distribution.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_EXPONENTIAL_DISTRIBUTION_PARAM_CHECK(lambda)  \
    VSMC_RUNTIME_ASSERT((lambda > 0), "**ExponentialDistribution** "          \
                                      "CONSTRUCTED WITH INVALID RATE "        \
                                      "PARAMETER VALUE")

namespace vsmc
{

/// \brief Generating exponential random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
void exponential_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType lambda = 1)
{
    U01DistributionType<RNGType, RealType> runif;
    for (std::size_t i = 0; i != n; ++i)
        r[i] = runif(rng);
    math::vLn(n, r, r);
    math::scal(n, -1 / lambda, r, 1);
}

/// \brief Exponential distribution
/// \ingroup Distribution
template <typename RealType>
class ExponentialDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = ExponentialDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = ExponentialDistribution<RealType>;

        explicit param_type(result_type lambda = 1) : lambda_(lambda)
        {
            invariant();
        }

        result_type lambda() const { return lambda_; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (!internal::is_equal(param1.lambda_, param2.lambda_))
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

            os << param.lambda_ << ' ';

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type lambda = 0;
            is >> std::ws >> lambda;

            if (is.good()) {
                if (lambda > 0) {
                    param = param_type(lambda);
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private:
        result_type lambda_;

        friend distribution_type;

        void invariant()
        {
            VSMC_RUNTIME_ASSERT_RNG_EXPONENTIAL_DISTRIBUTION_PARAM_CHECK(
                lambda);
        }

        void reset() {}
    }; // class param_type

    explicit ExponentialDistribution(result_type lambda = 0) : param_(lambda)
    {
    }

    explicit ExponentialDistribution(const param_type &param) : param_(param)
    {
    }

    result_type lambda() const { return param_.lambda_; }

    result_type min VSMC_MNE() const { return 0; }

    result_type max VSMC_MNE() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        U01DistributionType<RNGType, RealType> runif;

        return std::log(runif(rng)) / param_.lambda_;
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class ExponentialDistribution

} // namespace vsmc

#endif // VSMC_RNG_EXPONENTIAL_DISTRIBUTION_HPP
