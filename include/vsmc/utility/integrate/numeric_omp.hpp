#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP

#include <vsmc/utility/integrate/numeric_base.hpp>
#include <omp.h>

namespace vsmc { namespace integrate {

/// \brief Numerical integration with OpenMP
/// \ingroup Integrate
template <typename Derived>
class NumericOMP : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename traits::OMPSizeTypeTrait<
        typename integrate_base_type::size_type>::type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        eval_type eval_op(eval);
#pragma omp parallel for reduction(+ : integral) default(none) \
        firstprivate (eval_op) shared(N, grid)
        for (size_type i = 1; i < N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval_op);

        return integral;
    }
}; // class NumericBase

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP
