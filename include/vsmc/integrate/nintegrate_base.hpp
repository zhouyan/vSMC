//============================================================================
// vSMC/include/vsmc/integrate/nintegrate_base.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTEGRATE_NINTEGRATE_BASE_HPP
#define VSMC_INTEGRATE_NINTEGRATE_BASE_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MSVC
#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED
#else // VSMC_MSVC
#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED \
    VSMC_STATIC_ASSERT(                                                      \
            (cxx11::is_base_of<NIntegrateBase<Derived>, Derived>::value),    \
            USE_CRTP_NIntegrateBase_WITH_A_CLASS_NOT_DERIVED_FROM_THE_BASE)
#endif // VSMC_MSVC

#define VSMC_STATIC_ASSERT_INTEGRATE_NINTEGRATE_BASE_NO_IMPL(member) \
    VSMC_STATIC_ASSERT((cxx11::is_same<Derived, NullType>::value),           \
            DERIVED_FROM_NIntegrateBase_WITHOUT_IMPLEMENTATION_OF_##member##_IN_THE_Derived_TEMPLATE_PARAMETER)

#define VSMC_RUNTIME_ASSERT_INTEGRATE_NINTEGRATE_BASE_DERIVED \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)),                     \
            ("DERIVED FROM NIntegrateBase "                                  \
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
