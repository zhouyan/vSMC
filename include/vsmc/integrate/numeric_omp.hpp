#ifndef VSMC_INTEGRATE_NUMERIC_OMP_HPP
#define VSMC_INTEGRATE_NUMERIC_OMP_HPP

#include <vsmc/integrate/base.hpp>
#include <omp.h>

namespace vsmc {

/// \brief Numerical integration using OpenMP
/// \ingroup Integrate
template <typename Derived>
class NumericOMP : public NumericBase<Derived>
{
    public :

    typedef typename traits::OMPSizeTypeTrait<
        typename NumericBase<Derived>::size_type>::type size_type;
    typedef typename NumericBase<Derived>::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        eval_type eval_op(eval);
#ifndef __SUNPRO_CC
#pragma omp parallel for reduction(+ : integral) default(shared) \
        firstprivate (eval_op)
#endif
        for (size_type i = 1; i < N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval_op);

        return integral;
    }
}; // class NumericOMP

} // namespace vsmc

#endif // VSMC_INTEGRATE_NUMERIC_OMP_HPP
