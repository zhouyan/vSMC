//============================================================================
// vSMC/include/vsmc/gcd/dispatch_object.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_GCD_DISPATCH_OBJECT_HPP
#define VSMC_GCD_DISPATCH_OBJECT_HPP

#include <vsmc/internal/common.hpp>
#include <dispatch/dispatch.h>

namespace vsmc {

/// \brief Base class of Dispatch objects
/// \ingroup Dispatch
///
/// \details All Dispatch objects are reference counting shared objects
template <typename DispatchType>
class DispatchObject
{
    public :

    /// \brief Create a DispatchObject from its C-type object
    ///
    /// \param object The raw object to be aquired
    /// \param retained Whether the object is already retained. Some objects
    /// are retained at least once when it is created. These objects shall not
    /// be retained again by this constructor. Passing `true` as this argument
    /// prevent the constructor to retain the object.
    DispatchObject (const DispatchType &object, bool retained) :
        object_(object) {if (!retained) retain();}

    DispatchObject (const DispatchObject<DispatchType> &other) :
        object_(other.object_) {retain();}

    DispatchObject<DispatchType> &operator= (
            const DispatchObject<DispatchType> &other)
    {
        if (this != &other && object_ != other.object_) {
            release();
            object_ = other.object_;
            retain();
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchObject (DispatchObject<DispatchType> &&other) :
        object_(cxx11::move(other.object_)) {other.object_ = VSMC_NULLPTR;}

    DispatchObject<DispatchType> &operator= (
            DispatchObject<DispatchType> &&other)
    {
        using std::swap;

        if (this != &other && object_ != other.object_)
            swap(object_, other.object_);

        return *this;
    }
#endif

    ~DispatchObject () {release();}

    /// \brief Return the underlying Dispatch object
    DispatchType object () const {return object_;}

    /// \brief Set the underlying Dispatch object and retain it
    void object (DispatchType obj)
    {
        if (object_ != obj) {
            release();
            object_ = obj;
            retain();
        }
    }

    void *get_context () const
    {return ::dispatch_get_context(object_);}

    void set_context (void *context) const
    {::dispatch_set_context(object_, context);}

    void set_finalizer_f (::dispatch_function_t finalizer) const
    {::dispatch_set_finalizer_f(object_, finalizer);}

    private :

    DispatchType object_;

    void retain ()
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);
    }

    void release ()
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
    }
}; // class DispatchObject

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_OBJECT_HPP
