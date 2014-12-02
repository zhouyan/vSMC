//============================================================================
// vSMC/include/vsmc/opencl/cl_buffer.hpp
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

#ifndef VSMC_OPENCL_CL_BUFFER_HPP
#define VSMC_OPENCL_CL_BUFFER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_manager.hpp>

namespace vsmc {

/// \brief OpenCL buffer
/// \ingroup OpenCL
///
/// \details
/// Unlike `cl::Buffer`, this class performs deep copy instead of shallow copy.
/// Each CLBuffer object is tied to a specific CLManager, and thus its context
/// and command queue, which are used to create and copy the buffers.
template <typename T, typename ID = CLDefault>
class CLBuffer
{
    public :

    typedef T value_type;
    typedef cl_ulong size_type;
    typedef CLManager<ID> manager_type;

    CLBuffer () : size_(0) {}

    CLBuffer (size_type N) :
        size_(N),
        data_(manager().template create_buffer<value_type>(size_)) {}

    CLBuffer (const CLBuffer<T, ID> &other) :
        size_(other.size_),
        data_(manager().template create_buffer<value_type>(size_)) {}

    CLBuffer<T, ID> &operator= (const CLBuffer<T, ID> &other)
    {
        if (this != &other) {
            resize(other.size_);
            if (size_ != 0) {
                manager().template copy_buffer<value_type>(
                        other.data_, data_, size_);
            }
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    CLBuffer (CLBuffer<T, ID> &&other) :
        size_(other.size_), data_(cxx11::move(other.data_))
    {
        other.size_ = 0;
        other.data_ = ::cl::Buffer();
    }

    CLBuffer<T, ID> &operator= (CLBuffer<T, ID> &&other)
    {
        using std::swap;
        if (this != &other) {
            swap(size_, other.size_);
            swap(data_, other.data_);
        }

        return *this;
    }

#endif

    ~CLBuffer () {}

    size_type size () const {return size_;}

    static manager_type &manager () {return manager_type::instance();}

    /// \brief Read only access to the raw `cl::Buffer` object
    ///
    /// \details
    /// This is alike the `data` method of C++11 `std::vector` ect. It provides
    /// direct access to the raw buffer.
    const ::cl::Buffer &data () const {return data_;}

    void resize (size_type N)
    {
        if (N == size_)
            return;

        size_ = N;
        data_ = manager().template create_buffer<value_type>(N);
    }

    private :

    size_type size_;
    ::cl::Buffer data_;
}; // class CLBuffer

} // namespace vsmc

#endif // VSMC_OPENCL_CL_BUFFER_HPP
