#ifndef VSMC_INTEGRATE_NINTEGRATE_STD_HPP
#define VSMC_INTEGRATE_NINTEGRATE_STD_HPP

#include <vsmc/integrate/nintegrate_base.hpp>
#include <vsmc/utility/stdtbb.hpp>

namespace vsmc {

/// \brief Numerical integration using C++11 concurrency
/// \ingroup Integrate
template <typename Derived>
class NIntegrateSTD : public NIntegrateBase<Derived>
{
    public :

    typedef typename NIntegrateBase<Derived>::size_type size_type;
    typedef typename NIntegrateBase<Derived>::eval_type eval_type;

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        return parallel_accumulate(BlockedRange<size_type>(1, N),
                work_(this, grid, eval), static_cast<double>(0));
    }

    private :

    class work_
    {
        public :

        typedef typename NIntegrateBase<Derived>::size_type size_type;

        work_ (NIntegrateSTD<Derived> *nintegrate, const double *grid,
                const eval_type &eval) :
            nintegrate_(nintegrate), grid_(grid), eval_(eval) {}

        void operator() (const BlockedRange<size_type> &range,
                double &integral) const
        {
            double sum = 0;
            for (size_type i = range.begin(); i != range.end(); ++i) {
                sum += nintegrate_->integrate_segment(
                        grid_[i - 1], grid_[i], eval_);
            }
            integral = sum;
        }

        private :

        NIntegrateSTD<Derived> *const nintegrate_;
        const double *const grid_;
        const eval_type eval_;
    }; // class work_
}; // class NIntegrateSTD

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_STD_HPP
