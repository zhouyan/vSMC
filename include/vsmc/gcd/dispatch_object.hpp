//============================================================================
// include/vsmc/gcd/dispatch_object.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
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
    explicit DispatchObject (const DispatchType &object, bool retained) :
        object_(object)
    {
        if (!retained && object_ != VSMC_NULLPTR)
            ::dispatch_retain(object);
    }

    DispatchObject (const DispatchObject<DispatchType> &other) :
        object_(other.object_)
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);
    }

    DispatchObject<DispatchType> &operator= (
            const DispatchObject<DispatchType> &other)
    {
        if (this == &other)
            return *this;

        if (object_ == other.object_)
            return *this;

        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
        object_ = other.object_;
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchObject (DispatchObject<DispatchType> &&other)
    {
        object_ = other.object_;
        other.object_ = VSMC_NULLPTR;
    }

    DispatchObject<DispatchType> &operator= (
            DispatchObject<DispatchType> &&other)
    {
        if (this != &other) {
            DispatchType tmp = object_;
            object_ = other.object_;
            other.object_ = tmp;
        }

        return *this;
    }
#endif

    ~DispatchObject ()
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
    }

    /// \brief Return the underlying Dispatch object
    DispatchType object () const {return object_;}

    /// \brief Set the underlying Dispatch object and retain it
    void object (DispatchType obj)
    {
        if (object_ == obj)
            return;

        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
        object_ = obj;
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);
    }

    void *get_context () const
    {return ::dispatch_get_context(object_);}

    void set_context (void *context) const
    {::dispatch_set_context(object_, context);}

    void set_finalizer_f (::dispatch_function_t finalizer) const
    {::dispatch_set_finalizer_f(object_, finalizer);}

    private :

    DispatchType object_;
}; // class DispatchObject

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_OBJECT_HPP
