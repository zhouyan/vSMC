//============================================================================
// include/vsmc/opencl/cl_manager.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_OPENCL_CL_BUFFER_HPP
#define VSMC_OPENCL_CL_BUFFER_HPP

#include <vsmc/opencl/cl_manager.hpp>
#include <utility>

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
    typedef std::size_t size_type;
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
