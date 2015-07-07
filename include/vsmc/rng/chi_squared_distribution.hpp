//============================================================================
// vSMC/include/vsmc/rng/chi_squared_distribution.hpp
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

#ifndef VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP
#define VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/gamma_distribution.hpp>

namespace vsmc
{

/// \brief ChiSquared distribution
/// \ingroup Distribution
template <typename RealType>
class ChiSquaredDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = ChiSquaredDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = ChiSquaredDistribution<RealType>;

        explicit param_type(result_type n = 1)
            : gamma_(0.5 * n, static_cast<result_type>(0.5))
        {
        }

        result_type n() const { return gamma_.alpha() * 2; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            return param1.gamma_ == param2.gamma_;
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

            os << param.gamma_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            GammaDistribution<RealType> gamma;
            is >> std::ws >> gamma;

            if (is.good())
                param.gamma_ = gamma;

            return is;
        }

        private:
        GammaDistribution<RealType> gamma_;

        friend distribution_type;

        void invariant() {}

        void reset() { gamma_.reset(); }
    }; // class param_type

    explicit ChiSquaredDistribution(result_type n = 1) : param_(n) {}

    explicit ChiSquaredDistribution(const param_type &param) : param_(param) {}

    result_type n() const { return param_.n_; }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        return param_.gamma_(rng);
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class ChiSquaredDistribution

/// \brief Generating chi_squared random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void chi_squared_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df = 1)
{
    gamma_distribution(rng, n, r, 0.5 * df, static_cast<RealType>(0.5));
}

} // namespace vsmc

#endif // VSMC_RNG_CHI_SQUARED_DISTRIBUTION_HPP
