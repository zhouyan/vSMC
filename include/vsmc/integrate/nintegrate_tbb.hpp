#ifndef VSMC_INTEGRATE_NINTEGRATE_TBB_HPP
#define VSMC_INTEGRATE_NINTEGRATE_TBB_HPP

#include <vsmc/integrate/nintegrate_base.hpp>
#include <tbb/tbb.h>

namespace vsmc {

/// \brief Numerical integration using Intel Threading Building Block
/// \ingroup Integrate
template <typename Derived>
class NIntegrateTBB : public NIntegrateBase<Derived>
{
    public :

    typedef typename NIntegrateBase<Derived>::size_type size_type;
    typedef typename NIntegrateBase<Derived>::eval_type eval_type;

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

        typedef typename NIntegrateBase<Derived>::size_type size_type;

        work_ (NIntegrateTBB<Derived> *nintegrate, const double *grid,
                const eval_type &eval) :
            nintegrate_(nintegrate), grid_(grid),
            eval_(eval), eval_copy_(eval), integral_(0) {}

        work_ (const work_ &other, tbb::split) :
            nintegrate_(other.nintegrate_), grid_(other.grid_),
            eval_(other.eval_copy_), eval_copy_(other.eval_copy_),
            integral_(0) {}

        void operator() (const tbb::blocked_range<size_type> &range)
        {
            double sum = integral_;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                sum += nintegrate_->integrate_segment(
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

        NIntegrateTBB<Derived> *const nintegrate_;
        const double *const grid_;
        const eval_type eval_;
        const eval_type eval_copy_;
        double integral_;
    }; // class work_
}; // class NIntegrateTBB

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_TBB_HPP
