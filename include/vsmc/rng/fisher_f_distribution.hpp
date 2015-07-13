//============================================================================
// vSMC/include/vsmc/rng/fisher_f_distribution.hpp
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

#ifndef VSMC_RNG_FISHER_F_DISTRIBUTION_HPP
#define VSMC_RNG_FISHER_F_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/chi_squared_distribution.hpp>

namespace vsmc
{

template <typename RealType, typename RNGType>
inline void fisher_f_distribution(
    RNGType &, std::size_t, RealType *, RealType, RealType);

/// \brief Fisher-F distribution
/// \ingroup Distribution
template <typename RealType>
class FisherFDistribution
{
    public:
    using result_type = RealType;
    using distribution_type = FisherFDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = FisherFDistribution<RealType>;

        explicit param_type(result_type m = 1, result_type n = 1)
            : chi_squared_m_(m), chi_squared_n_(n)
        {
        }

        result_type m() const { return chi_squared_m_.n(); }
        result_type n() const { return chi_squared_n_.n(); }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.chi_squared_m_ != param2.chi_squared_m_)
                return false;
            if (param1.chi_squared_n_ != param2.chi_squared_n_)
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

            os << param.chi_squared_m_ << ' ';
            os << param.chi_squared_n_ << ' ';

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            ChiSquaredDistribution<RealType> chi_squared_m;
            ChiSquaredDistribution<RealType> chi_squared_n;
            is >> std::ws >> chi_squared_m;
            is >> std::ws >> chi_squared_n;

            if (is.good()) {
                param.chi_squared_m_ = std::move(chi_squared_m);
                param.chi_squared_n_ = std::move(chi_squared_n);
            }

            return is;
        }

        private:
        ChiSquaredDistribution<RealType> chi_squared_m_;
        ChiSquaredDistribution<RealType> chi_squared_n_;

        friend distribution_type;

        void invariant() {}

        void reset()
        {
            chi_squared_m_.reset();
            chi_squared_n_.reset();
        }
    }; // class param_type

    explicit FisherFDistribution(result_type m = 1, result_type n = 1)
        : param_(m, n)
    {
    }

    explicit FisherFDistribution(const param_type &param) : param_(param) {}

    result_type m() const { return param_.m(); }
    result_type n() const { return param_.n(); }

    result_type min() const { return 0; }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        result_type u1 = param_.chi_squared_m_(rng) / m();
        result_type u2 = param_.chi_squared_n_(rng) / n();

        return u1 / u2;
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        fisher_f_distribution(rng, n, r, m(), this->n());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class FisherFDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void fisher_f_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType df1, RealType df2)
{
    RealType s[K];
    chi_squared_distribution(rng, n, s, df1);
    chi_squared_distribution(rng, n, r, df2);
    mul(n, 1 / df1, s, s);
    mul(n, 1 / df2, r, r);
    div(n, s, r, r);
}

} // namespace vsmc::internal

/// \brief Generating Fisher-F random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void fisher_f_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df1, RealType df2)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::fisher_f_distribution_impl<k>(rng, k, r + i * k, df1, df2);
    internal::fisher_f_distribution_impl<k>(rng, l, r + m * k, df1, df2);
}

template <typename RealType, typename RNGType>
inline void rng_rand(RNGType &rng, FisherFDistribution<RealType> &dist,
    std::size_t n, RealType *r)
{
    fisher_f_distribution(rng, n, r, dist.m(), dist.n());
}

} // namespace vsmc

#endif // VSMC_RNG_FISHER_F_DISTRIBUTION_HPP
