#ifndef VSMC_UTILITY_INTEGRATE_NUMERIC_TBB_HPP
#define VSMC_UTILITY_INTEGRATE_NUMERIC_TBB_HPP

#if defined(__clang__) && !defined(_LIBCPP_VERSION) && (__GLIBCXX__ < 20100429)
#ifndef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 1
#endif
#endif // __clang__

#include <vsmc/utility/integrate/numeric_base.hpp>
#include <tbb/tbb.h>

namespace vsmc {

/// \brief Numerical integration using Intel Threading Building Block
/// \ingroup Integrate
template <typename Derived>
class NumericTBB : public NumericBase<Derived>
{
    public :

    typedef NumericBase<Derived> integrate_base_type;
    typedef typename integrate_base_type::size_type size_type;
    typedef typename integrate_base_type::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        work_ work(this, grid, eval);
        tbb::parallel_reduce(tbb::blocked_range<size_type>(1, N), work);

        return work.integral();
    }

    private :

    class work_
    {
        public :

        work_ (NumericTBB<Derived> *numeric, const double *grid,
                const eval_type &eval) :
            numeric_(numeric), grid_(grid), eval_(eval), eval_copy_(eval),
            integral_(0) {}

        work_ (const work_ &other, tbb::split) :
            numeric_(other.numeric_), grid_(other.grid_),
            eval_(other.eval_copy_), eval_copy_(other.eval_copy_),
            integral_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            double sum = integral_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                sum += numeric_->integrate_segment(
                        grid_[i - 1], grid_[i], eval_);
            }
            integral_ = sum;
        }

        void join (const work_ &other)
        {
            integral_ += other.integral_;
        }

        double integral () const
        {
            return integral_;
        }

        private :

        NumericTBB<Derived> *const numeric_;
        const double *const grid_;
        const eval_type eval_;
        const eval_type eval_copy_;
        double integral_;
    }; // class work_
}; // class NumericTBB

} // namespace vsmc

#endif // VSMC_UTILITY_INTEGRATE_NUMERIC_TBB_HPP
