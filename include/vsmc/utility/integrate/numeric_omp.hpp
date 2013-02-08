#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP

#include <vsmc/utility/integrate/base.hpp>
#include <omp.h>

namespace vsmc {

/// \brief Numerical integration using OpenMP
/// \ingroup Integrate
template <typename Derived>
class NumericOMP : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename internal::OMPSizeTypeTrait<
        typename integrate_base_type::size_type>::type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        eval_type eval_op(eval);
#if VSMC_OPENMP_COMPILER_GOOD
#pragma omp parallel for reduction(+ : integral) default(shared) \
        firstprivate (eval_op)
#endif
        for (size_type i = 1; i < N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval_op);

        return integral;
    }
}; // class NumericOMP

} // namespace vsmc

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_OMP_HPP
