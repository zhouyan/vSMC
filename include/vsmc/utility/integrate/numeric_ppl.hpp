#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_PPL_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_PPL_HPP

#include <vsmc/utility/integrate/numeric_base.hpp>

namespace vsmc { namespace integrate {

/// \brief Numerical integration with Microsoft Parallel Pattern Library
/// \ingroup Integrate
template <typename Derived>
class NumericPPL : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename integrate_base_type::size_type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        integral_.resize(N - 1);
        ppl::parallel_for(static_cast<size_type>(1), N,
                work_(this, grid, eval, &integral_[0]));

        double integral = 0;
        for (std::size_t i = 0; i != integral_.size(); ++i)
            integral += integral_[i];

        return integral;
    }

    private :

    std::vector<double> integral_;

    class work_
    {
        public :

        work_ (NumericPPL<Derived> *numeric, const double *grid,
                const eval_type &eval, double *integral) :
            numeric_(numeric), grid_(grid), eval_(eval), integral_(integral) {}

        void operator() (size_type i) const
        {
            integral_[i - 1] = numeric_->integrate_segment(
                    grid_[i - 1], grid_[i], eval_);
        }

        private :

        NumericPPL<Derived> *const numeric_;
        const double *const grid_;
        const eval_type eval_;
        double *const integral_;
    }; // class work_
}; // class NumericBase

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_PPL_HPP
