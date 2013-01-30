#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_NEWTON_COTES
#define VSMC_UTILITY_INTEGRATE_NUMERIC_NEWTON_COTES

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>

#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <cstddef>
#include <vector>

namespace vsmc { namespace internal {

template <unsigned Index, typename EvalType>
struct NumericNewtonCotesEval
{
    static double result (const double *coeff, double a, double h,
            const EvalType &eval)
    {
        return coeff[Index] * eval(a + (Index - 1) * h) +
            NumericNewtonCotesEval<Index - 1, EvalType>::
            result(coeff, a, h, eval);
    }
};

template <typename EvalType>
struct NumericNewtonCotesEval<1, EvalType>
{
    static double result (const double *coeff, double a, double h,
            const EvalType &eval)
    {
        return coeff[1] * eval(a);
    }
};

template <unsigned Degree>
class NumericNewtonCotesCoeff
{
    public :

    static NumericNewtonCotesCoeff<Degree> &instance ()
    {
        static NumericNewtonCotesCoeff<Degree> coeff;

        return coeff;
    }

    const double *coeff() const
    {
        return coeff_;
    }

    private :

    double coeff_[Degree + 2];

    NumericNewtonCotesCoeff ()
    {coeff_init(cxx11::integral_constant<unsigned, Degree>());}

    NumericNewtonCotesCoeff
        (const NumericNewtonCotesCoeff<Degree> &);
    NumericNewtonCotesCoeff<Degree> &operator=
        (const NumericNewtonCotesCoeff<Degree> &);

    void coeff_init (cxx11::integral_constant<unsigned, 1>)
    {
        coeff_[0] = 0.5;
        coeff_[1] = 1;
        coeff_[2] = 1;
    }

    void coeff_init (cxx11::integral_constant<unsigned, 2>)
    {
        coeff_[0] = 1.0 / 6.0;
        coeff_[1] = 1;
        coeff_[2] = 4;
        coeff_[3] = 1;
    }

    void coeff_init (cxx11::integral_constant<unsigned, 3>)
    {
        coeff_[0] = 0.125;
        coeff_[1] = 1;
        coeff_[2] = 3;
        coeff_[3] = 3;
        coeff_[4] = 1;
    }

    void coeff_init (cxx11::integral_constant<unsigned, 4>)
    {
        coeff_[0] = 1.0 / 90.0;
        coeff_[1] = 7;
        coeff_[2] = 32;
        coeff_[3] = 12;
        coeff_[4] = 32;
        coeff_[5] = 7;
    }

    void coeff_init (cxx11::integral_constant<unsigned, 5>)
    {
        coeff_[0] = 1.0 / 288.0;
        coeff_[1] = 19;
        coeff_[2] = 75;
        coeff_[3] = 50;
        coeff_[4] = 50;
        coeff_[5] = 75;
        coeff_[6] = 19;
    }

    void coeff_init (cxx11::integral_constant<unsigned, 6>)
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

    void coeff_init (cxx11::integral_constant<unsigned, 7>)
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

    void coeff_init (cxx11::integral_constant<unsigned, 8>)
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

    void coeff_init (cxx11::integral_constant<unsigned, 9>)
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

    void coeff_init (cxx11::integral_constant<unsigned, 10>)
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
};

} } // namespace vsmc::internal

namespace vsmc { namespace integrate {

/// \brief Numerical integration with the (closed) Newton-Cotes formulae
/// \ingroup Integrate
template <unsigned Degree, template <typename> class NumericImpl>
class NumericNewtonCotes :
    public NumericImpl<NumericNewtonCotes<Degree, NumericImpl> >
{
    public :

    typedef NumericImpl<NumericNewtonCotes<Degree, NumericImpl> >
        integrate_impl_type;
    typedef typename integrate_impl_type::size_type size_type;
    typedef typename integrate_impl_type::eval_type eval_type;

    NumericNewtonCotes () :
        coeff_(internal::NumericNewtonCotesCoeff<Degree>::instance().coeff())
    {}

    double integrate_segment (double a, double b, const eval_type &eval) const
    {
        VSMC_STATIC_ASSERT_NUMERIC_NEWTON_COTES_DEGREE(Degree);

        double h = (b - a) / Degree;

        return coeff_[0] * (b - a) * (
                internal::NumericNewtonCotesEval<Degree, eval_type>::
                result(coeff_, a, h, eval) + coeff_[Degree + 1] * eval(b));
    }

    static VSMC_CONSTEXPR unsigned max_degree ()
    {
        return max_degree_;
    }

    private :

    static const unsigned max_degree_ = 10;
    const double *coeff_;
}; // class NumericNewtonCotes

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_NUMERIC_NEWTON_COTES
