//============================================================================
// vSMC/include/vsmc/integrate/nintegrate_newton_cotes.hpp
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

#ifndef VSMC_INTEGRATE_NINTEGRATE_NEWTON_COTES_HPP
#define VSMC_INTEGRATE_NINTEGRATE_NEWTON_COTES_HPP

#include <vsmc/integrate/nintegrate_base.hpp>

#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_NEWTON_COTES_DEGREE(degree)  \
    VSMC_STATIC_ASSERT((degree >= 1 && degree <= max_degree_),               \
        USE_NIntegrateNewtonCotes_WITH_A_DEGREE_LARGER_THAN_max_degree)

namespace vsmc
{

namespace internal
{

template <unsigned Index, typename EvalType>
struct NIntegrateNewtonCotesEval {
    static double result(
        const double *coeff, double a, double h, const EvalType &eval)
    {
        return coeff[Index] * eval(a + (Index - 1) * h) +
            NIntegrateNewtonCotesEval<Index - 1, EvalType>::result(
                   coeff, a, h, eval);
    }
}; // struct NIntegrateNewtonCotesEval

template <typename EvalType>
struct NIntegrateNewtonCotesEval<1, EvalType> {
    static double result(
        const double *coeff, double a, double, const EvalType &eval)
    {
        return coeff[1] * eval(a);
    }
}; // struct NIntegrateNewtonCotesEval

template <unsigned Degree>
class NIntegrateNewtonCotesCoeff
{
    public:
    static NIntegrateNewtonCotesCoeff<Degree> &instance()
    {
        static NIntegrateNewtonCotesCoeff<Degree> coeff;

        return coeff;
    }

    const double *coeff() const { return coeff_; }

    private:
    double coeff_[Degree + 2];

    NIntegrateNewtonCotesCoeff()
    {
        coeff_init(std::integral_constant<unsigned, Degree>());
    }

    NIntegrateNewtonCotesCoeff(const NIntegrateNewtonCotesCoeff<Degree> &);

    NIntegrateNewtonCotesCoeff<Degree> &operator=(
        const NIntegrateNewtonCotesCoeff<Degree> &);

    void coeff_init(std::integral_constant<unsigned, 1>)
    {
        coeff_[0] = 0.5;
        coeff_[1] = 1;
        coeff_[2] = 1;
    }

    void coeff_init(std::integral_constant<unsigned, 2>)
    {
        coeff_[0] = 1.0 / 6.0;
        coeff_[1] = 1;
        coeff_[2] = 4;
        coeff_[3] = 1;
    }

    void coeff_init(std::integral_constant<unsigned, 3>)
    {
        coeff_[0] = 0.125;
        coeff_[1] = 1;
        coeff_[2] = 3;
        coeff_[3] = 3;
        coeff_[4] = 1;
    }

    void coeff_init(std::integral_constant<unsigned, 4>)
    {
        coeff_[0] = 1.0 / 90.0;
        coeff_[1] = 7;
        coeff_[2] = 32;
        coeff_[3] = 12;
        coeff_[4] = 32;
        coeff_[5] = 7;
    }

    void coeff_init(std::integral_constant<unsigned, 5>)
    {
        coeff_[0] = 1.0 / 288.0;
        coeff_[1] = 19;
        coeff_[2] = 75;
        coeff_[3] = 50;
        coeff_[4] = 50;
        coeff_[5] = 75;
        coeff_[6] = 19;
    }

    void coeff_init(std::integral_constant<unsigned, 6>)
    {
        coeff_[0] = 1.0 / 840.0;
        coeff_[1] = 41;
        coeff_[2] = 216;
        coeff_[3] = 27;
        coeff_[4] = 272;
        coeff_[5] = 27;
        coeff_[6] = 216;
        coeff_[7] = 41;
    }

    void coeff_init(std::integral_constant<unsigned, 7>)
    {
        coeff_[0] = 1.0 / 17280.0;
        coeff_[1] = 751;
        coeff_[2] = 3577;
        coeff_[3] = 1323;
        coeff_[4] = 2989;
        coeff_[5] = 2989;
        coeff_[6] = 1323;
        coeff_[7] = 3577;
        coeff_[8] = 751;
    }

    void coeff_init(std::integral_constant<unsigned, 8>)
    {
        coeff_[0] = 1.0 / 28350.0;
        coeff_[1] = 989;
        coeff_[2] = 5888;
        coeff_[3] = -928;
        coeff_[4] = 10496;
        coeff_[5] = -4540;
        coeff_[6] = 10496;
        coeff_[7] = -928;
        coeff_[8] = 5888;
        coeff_[9] = 989;
    }

    void coeff_init(std::integral_constant<unsigned, 9>)
    {
        coeff_[0] = 1.0 / 89600.0;
        coeff_[1] = 2857;
        coeff_[2] = 15741;
        coeff_[3] = 1080;
        coeff_[4] = 19344;
        coeff_[5] = 5778;
        coeff_[6] = 5778;
        coeff_[7] = 19344;
        coeff_[8] = 1080;
        coeff_[9] = 15741;
        coeff_[10] = 2857;
    }

    void coeff_init(std::integral_constant<unsigned, 10>)
    {
        coeff_[0] = 1.0 / 598752.0;
        coeff_[1] = 16067;
        coeff_[2] = 106300;
        coeff_[3] = -48525;
        coeff_[4] = 272400;
        coeff_[5] = -260550;
        coeff_[6] = 427368;
        coeff_[7] = -260550;
        coeff_[8] = 272400;
        coeff_[9] = -48525;
        coeff_[10] = 106300;
        coeff_[11] = 16067;
    }
}; // NIntegrateNewtonCotesCoeff

} // namespace vsmc::internal

/// \brief Numerical integration with the (closed) Newton-Cotes formulae
/// \ingroup Integrate
template <unsigned Degree>
class NIntegrateNewtonCotes
    : public NIntegrateBase<NIntegrateNewtonCotes<Degree>>
{
    public:
    typedef typename NIntegrateBase<NIntegrateNewtonCotes<Degree>>::size_type
        size_type;
    typedef typename NIntegrateBase<NIntegrateNewtonCotes<Degree>>::eval_type
        eval_type;

    static double integrate_segment(double a, double b, const eval_type &eval)
    {
        VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_NEWTON_COTES_DEGREE(Degree);

        const double *const coeff =
            internal::NIntegrateNewtonCotesCoeff<Degree>::instance().coeff();

        double h = (b - a) / Degree;

        return coeff[0] * (b - a) *
            (internal::NIntegrateNewtonCotesEval<Degree, eval_type>::result(
                 coeff, a, h, eval) +
                   coeff[Degree + 1] * eval(b));
    }

    static constexpr unsigned max_degree() { return max_degree_; }

    private:
    static constexpr const unsigned max_degree_ = 10;
}; // class NIntegrateNewtonCotes

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_NEWTON_COTES_HPP
