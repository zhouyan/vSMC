//============================================================================
// vSMC/include/vsmc/rng/student_t_distribution.hpp
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

#ifndef VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP
#define VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/chi_squared_distribution.hpp>
#include <vsmc/rng/normal_distribution.hpp>

namespace vsmc
{

template <typename RealType, typename RNGType>
inline void student_t_distribution(
    RNGType &, std::size_t, RealType *, RealType);

/// \brief Student-t distribution
/// \ingroup Distribution
template <typename RealType>
class StudentTDistribution
{

    public:
    using result_type = RealType;
    using distribution_type = StudentTDistribution<RealType>;

    class param_type
    {
        public:
        using result_type = RealType;
        using distribution_type = StudentTDistribution<RealType>;

        explicit param_type(result_type n = 1) : chi_squared_(n), normal_(0, 1)
        {
        }

        result_type n() const { return chi_squared_.n() * 2; }

        friend bool operator==(
            const param_type &param1, const param_type &param2)
        {
            if (param1.chi_squared_ != param2.chi_squared_)
                return false;
            if (param1.normal_ != param2.normal_)
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

            os << param.chi_squared << ' ';
            os << param.normal_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits> &operator>>(
            std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            ChiSquaredDistribution<RealType> chi_squared;
            NormalDistribution<RealType> normal;
            is >> std::ws >> chi_squared;
            is >> std::ws >> normal;

            if (is.good()) {
                param.chi_squared = chi_squared;
                param.normal_ = normal;
            }

            return is;
        }

        private:
        ChiSquaredDistribution<RealType> chi_squared_;
        NormalDistribution<RealType> normal_;

        friend distribution_type;

        void invariant() {}

        void reset()
        {
            chi_squared_.reset();
            normal_.reset();
        }
    }; // class param_type

    explicit StudentTDistribution(result_type n = 1) : param_(n) {}

    explicit StudentTDistribution(const param_type &param) : param_(param) {}

    result_type n() const { return param_.n(); }

    result_type min() const
    {
        return -std::numeric_limits<result_type>::infinity();
    }

    result_type max() const
    {
        return std::numeric_limits<result_type>::infinity();
    }

    template <typename RNGType>
    result_type operator()(RNGType &rng)
    {
        result_type z = param_.normal_(rng);
        result_type u = n() / param_.chi_squared_(rng);

        return z * std::sqrt(u);
    }

    template <typename RNGType>
    void operator()(RNGType &rng, std::size_t n, result_type *r)
    {
        student_t_distribution(rng, n, r, this->n());
    }

    VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS

    private:
    param_type param_;
}; // class StudentTDistribution

namespace internal
{

template <std::size_t K, typename RealType, typename RNGType>
inline void student_t_distribution_impl(
    RNGType &rng, std::size_t n, RealType *r, RealType df)
{
    RealType s[K];
    chi_squared_distribution(rng, n, r, df);
    mul(n, 1 / df, r, r);
    sqrt(n, r, r);
    normal_distribution(
        rng, n, s, static_cast<RealType>(0), static_cast<RealType>(1));
    div(n, s, r, r);
}

} // namespace vsmc::internal

/// \brief Generating student-t random variates
/// \ingroup Distribution
template <typename RealType, typename RNGType>
inline void student_t_distribution(
    RNGType &rng, std::size_t n, RealType *r, RealType df)
{
    const std::size_t k = 1000;
    const std::size_t m = n / k;
    const std::size_t l = n % k;
    for (std::size_t i = 0; i != m; ++i)
        internal::student_t_distribution_impl<k>(rng, k, r + i * k, df);
    internal::student_t_distribution_impl<k>(rng, l, r + m * k, df);
}

} // namespace vsmc

#endif // VSMC_RNG_STUDENT_T_DISTRIBUTION_HPP
