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

    double integrate_segment (double a, double b, const eval_type &eval)
    {
        VSMC_STATIC_ASSERT_NUMERIC_NEWTON_COTES_DEGREE(Degree);
        return integrate_segment_newton_cotes(a, b, eval,
                cxx11::integral_constant<unsigned, Degree>());
    }

    private :

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 1>)
    {
        return 0.5 * (b - a) * (eval(a) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 2>)
    {
        const double coeff = 1.0 / 6.0;

        return coeff * (b - a) * (
                eval(a) + 4 * eval(a + 0.5 * (b - a)) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 3>)
    {
        double h = (b - a ) / 3;
        double x1 = a + h;
        double x2 = b - h;

        return 0.125 * (b - a) * (
                eval(a) + 3 * eval(x1) + 3 * eval(x2) + eval(b));
    }

    double integrate_segment_newton_cotes (double a, double b,
            const eval_type &eval,
            cxx11::integral_constant<unsigned, 4>)
    {
        const double coeff = 1.0 / 90.0;
        double h = 0.25 * (b - a);
        double x1 = a + h;
        double x2 = a + h * 2;
        double x3 = a + h * 3;

        return coeff * (b - a) * (
                7 * eval(a) + 32 * eval(x1) + 12 * eval(x2) +
                32 * eval(x3) + 7 * eval(b));
    }
}; // class NumericNewtonCotes

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_NUMERIC_NEWTON_COTES
