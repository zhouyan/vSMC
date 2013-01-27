#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_GCD_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_GCD_HPP

#include <vsmc/utility/integrate/numeric_base.hpp>
#include <vsmc/utility/dispatch.hpp>

namespace vsmc { namespace integrate {

/// \brief Numerical integration with Apple Grand Central Dispatch
/// \ingroup Integrate
template <typename Derived>
class NumericGCD : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename integrate_base_type::size_type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        for (size_type i = 1; i != N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval);

        return integral;
    }
}; // class NumericBase

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_GCD_HPP
