//============================================================================
// include/vsmc/integrate/nintegrate_base.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTEGRATE_NINTEGRATE_BASE_HPP
#define VSMC_INTEGRATE_NINTEGRATE_BASE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/functional.hpp>

#ifdef _MSC_VER
#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED
#else // _MSC_VER
#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_base_of<NIntegrateBase<Derived>, Derived>::value),    \
            USE_CRTP_NIntegrateBase_WITH_A_CLASS_NOT_DERIVED_FROM_THE_BASE)
#endif // _MSC_VER

#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_NO_IMPL(member) \
    VSMC_STATIC_ASSERT((cxx11::is_same<Derived, NullType>::value),           \
            DERIVED_FROM_NIntegrateBase_WITHOUT_IMPLEMENTATION_OF_##member##_IN_THE_Derived_TEMPLATE_PARAMETER)

#define VSMC_RUNTIME_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)),                     \
            ("DERIVED FROM  NIntegrateBase "                                 \
             "WITH INCORRECT **Derived** TEMPLATE PARAMTER"));

namespace vsmc {

/// \brief Numerical integration base dispatch class
/// \ingroup Integrate
template <typename Derived>
class NIntegrateBase
{
    protected :

    typedef std::size_t size_type;
    typedef cxx11::function<double (double)> eval_type;

    NIntegrateBase () {}
    NIntegrateBase (const NIntegrateBase<Derived> &) {}
    NIntegrateBase<Derived> &operator= (const NIntegrateBase<Derived> &)
    {return *this;}
    VSMC_CRTP_DESTRUCTOR_PREFIX ~NIntegrateBase () {}

    /// \brief Integrate a segment on the grid
    ///
    /// \param a The leftmost of the segment
    /// \param b The rightmost of the segment
    /// \param eval The functor used for evaluation
    double integrate_segment (double a, double b, const eval_type &eval)
    {
        return integrate_segment_dispatch(a, b, eval,
                &Derived::integrate_segment);
    }

    double operator() (size_type N, const double *grid, const eval_type &eval)
    {
        if (N < 2)
            return 0;

        double integral = 0;
        for (size_type i = 1; i != N; ++i)
            integral += this->integrate_segment(grid[i - 1], grid[i], eval);

        return integral;
    }

    private :

    template <typename D>
    double integrate_segment_dispatch (double a, double b,
            const eval_type &eval,
            double (D::*) (double, double, const eval_type &))
    {
        VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED;
        VSMC_RUNTIME_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED;
        return static_cast<Derived *>(this)->integrate_segment(a, b, eval);
    }

    template <typename D>
    double integrate_segment_dispatch (double a, double b,
            const eval_type &eval,
            double (D::*) (double, double, const eval_type &) const)
    {
        VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED;
        VSMC_RUNTIME_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED;
        return static_cast<Derived *>(this)->integrate_segment(a, b, eval);
    }

    double integrate_segment_dispatch (double a, double b,
            const eval_type &eval,
            double (*) (double, double, const eval_type &))
    {return Derived::integrate_segment(a, b, eval);}

    double integrate_segment_dispatch (double, double, const eval_type &,
            double (NIntegrateBase::*) (double, double, const eval_type &))
    {
        VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_NO_IMPL(
                integrate_segment);
        return 0;
    }
}; // class NIntegrateBase

/// \brief Numerical integration base dispatch class
/// \ingroup Integrate
template <>
class NIntegrateBase<Virtual>
{
    public :

    typedef std::size_t size_type;
    typedef cxx11::function<double (double)> eval_type;

    protected :

    NIntegrateBase () {}
    NIntegrateBase (const NIntegrateBase<Virtual> &) {}
    NIntegrateBase<Virtual> &operator= (const NIntegrateBase<Virtual> &)
    {return *this;}
    virtual ~NIntegrateBase () {}

    virtual double integrate_segment (double, double, const eval_type &) = 0;
}; // class NIntegrateBase<Virtual>

} // namespace vsmc

#endif // VSMC_INTEGRATE_NINTEGRATE_BASE_HPP
