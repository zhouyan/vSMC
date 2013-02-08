#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_CILK_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_CILK_HPP

#include <vsmc/utility/integrate/base.hpp>
#include <cilk/cilk.h>
#include <cilk/holder.h>
#include <cilk/reducer_opadd.h>

namespace vsmc {

/// \brief Numerical integration using Intel Cilk Plus
/// \ingroup Integrate
template <typename Derived>
class NumericCILK : public NumericBase<Derived>
{
    public :

    typedef typename NumericBase<Derived>::size_type size_type;
    typedef typename NumericBase<Derived>::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        cilk::reducer_opadd<double> integral;
        cilk::holder<eval_type> eval_op(eval);
        cilk_for (size_type i = 1; i != N; ++i) {
            integral += this->integrate_segment(
                    grid[i - 1], grid[i], eval_op());
        }

        return integral.get_value();
    }
}; // class NumericCILK

} // namespace vsmc

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_CILK_HPP
