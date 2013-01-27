#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_STD_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_STD_HPP

#include <vsmc/utility/integrate/numeric_base.hpp>
#include <vsmc/utility/stdtbb.hpp>

namespace vsmc { namespace integrate {

/// \brief Numerical integration with C++11 concurrency implementation
/// \ingroup Integrate
template <typename Derived>
class NumericSTD : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename integrate_base_type::size_type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        return thread::parallel_accumulate(
                thread::BlockedRange<size_type>(1, N), work_(this, grid, eval),
                static_cast<double>(0));
    }

    private :

    class work_
    {
        public :

        work_ (NumericSTD<Derived> *numeric, const double *grid,
                const eval_type &eval) :
            numeric_(numeric), grid_(grid), eval_(eval) {}

        void operator() (const thread::BlockedRange<size_type> &range,
                double &integral) const
        {
            double sum = 0;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                sum += numeric_->integrate_segment(
                        grid_[i - 1], grid_[i], eval_);
            }
            integral = sum;
        }

        private :

        NumericSTD<Derived> *const numeric_;
        const double *const grid_;
        const eval_type eval_;
    }; // class work_
}; // class NumericBase

} } // namespace vsmc::integrate

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_STD_HPP
