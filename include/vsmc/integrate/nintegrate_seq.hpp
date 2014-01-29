#ifndef VSMC_INTEGRATE_NINTEGRATE_SEQ_HPP
#define VSMC_INTEGRATE_NINTEGRATE_SEQ_HPP

#include <vsmc/integrate/nintegrate_base.hpp>

namespace vsmc {

/// \brief Numerical integration using sequential implementation
/// \ingroup Integrate
template <typename Derived>
class NIntegrateSEQ : public NIntegrateBase<Derived>
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
}; // class NIntegrateSEQ

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_SEQ_HPP
