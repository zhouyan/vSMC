//============================================================================
// vSMC/include/vsmc/gcd/dispatch_function.hpp
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

#ifndef VSMC_GCD_DISPATCH_FUNCTION_HPP
#define VSMC_GCD_DISPATCH_FUNCTION_HPP

#include <vsmc/internal/common.hpp>
#include <dispatch/dispatch.h>

namespace vsmc
{

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
template <typename T> class DispatchFunction
{
    public:
    DispatchFunction(const T &work) : work_(work) {}

    DispatchFunction(T &&work) noexcept : work_(std::move(work)) {}

    void *context() { return static_cast<void *>(this); }

    ::dispatch_function_t function() const { return function_; }

    private:
    T work_;

    static void function_(void *ctx)
    {
        DispatchFunction<T> *df_ptr = static_cast<DispatchFunction<T> *>(ctx);
        df_ptr->work_();
    }
}; // class DispatchFunction

/// \brief Make a DispatchFunction object from an arbitrary callable object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(void)`
template <typename T>
inline DispatchFunction<
    typename std::remove_cv<typename std::remove_reference<T>::type>::type> *
    dispatch_function_new(T &&work) noexcept
{
    typedef typename std::remove_reference<T>::type U;
    typedef typename std::remove_cv<U>::type V;
    return new DispatchFunction<V>(std::forward<T>(work));
}

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_FUNCTION_HPP
