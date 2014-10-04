//============================================================================
// include/vsmc/gcd/dispatch_function.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_GCD_DISPATCH_FUNCTION_HPP
#define VSMC_GCD_DISPATCH_FUNCTION_HPP

#include <vsmc/internal/common.hpp>
#include <dispatch/dispatch.h>

namespace vsmc {

/// \brief Wrap a callable object into a `dispatch_function_t` type pointer
/// \ingroup Dispatch
///
/// \details
/// It is important that this object live at least after the completion of the
/// work. This is a thin wrapper around a regular C++ callable object
/// (including lambda expressions) that make it easier to interface them with
/// GCD. The memory management issue is the same as using raw GCD interface
/// with a context variable, which shall not be destroyed before the execution
/// of the function finishes. The original object does not need to exist after
/// the creation of DispatchFunction.
template <typename T>
class DispatchFunction
{
    public :

    DispatchFunction (const T &work) : work_(work) {}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchFunction (T &&work) VSMC_NOEXCEPT : work_(cxx11::move(work)) {}
#endif

    void *context () {return static_cast<void *>(this);}

    ::dispatch_function_t function () const {return function_;}

    private :

    T work_;

    static void function_ (void *ctx)
    {
        DispatchFunction<T> *df_ptr = static_cast<DispatchFunction<T> *>(ctx);
        df_ptr->work_();
    }
}; // class DispatchFunction

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Make a DispatchFunction object from an arbitrary callable object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(void)`
template <typename T>
inline DispatchFunction<
typename cxx11::remove_cv<typename cxx11::remove_reference<T>::type>::type>
*dispatch_function_new (T &&work) VSMC_NOEXCEPT
{
    typedef typename cxx11::remove_reference<T>::type U;
    typedef typename cxx11::remove_cv<U>::type V;
    return new DispatchFunction<V>(cxx11::forward<T>(work));
}
#else
template <typename T>
inline DispatchFunction<T> *dispatch_function_new (const T &work)
{return new DispatchFunction<T>(work);}
#endif

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_FUNCTION_HPP
