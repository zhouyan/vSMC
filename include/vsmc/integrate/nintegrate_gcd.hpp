#ifndef VSMC_INTEGRATE_NINTEGRATE_GCD_HPP
#define VSMC_INTEGRATE_NINTEGRATE_GCD_HPP

#include <vsmc/integrate/nintegrate_base.hpp>
#include <vsmc/smp/internal/dispatch.hpp>

namespace vsmc {

/// \brief Numerical integration using Apple Grand Central Dispatch
/// \ingroup Integrate
template <typename Derived>
class NIntegrateGCD : public NIntegrateBase<Derived>
{
    public :

    typedef typename NIntegrateBase<Derived>::size_type size_type;
    typedef typename NIntegrateBase<Derived>::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        for (size_type i = 1; i != N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval);

        return integral;
    }
}; // class NIntegrateGCD

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_GCD_HPP
