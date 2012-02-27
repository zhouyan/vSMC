#ifndef V_SMC_CORE_BUFFER_HPP
#define V_SMC_CORE_BUFFER_HPP

#include <stdexcept>
#include <cstddef>
#include <mkl_service.h>

#ifndef V_SMC_ALIGN_SIZE
#define V_SMC_ALIGN_SIZE 16
#endif // V_SMC_ALIGN_SIZE

namespace vSMC { namespace internal {

/// \brief Buffer resource manager.
///
/// The sole purpose of this class is to manage a temporary array used in
/// various places in vSMC This class manager the buffer used in various
/// places in vSMC. It can be safely used where a (T *) is required. It is a
/// little like a smart pointer, but not exactly the same. The purpose of this
/// class is to create and manage a buffer array. The buffer is copyable. It
/// is not a general purpose vector class, like one will find in Eigen or
/// Aramadillo. The size of the array cannot be changed once it is created.
template <typename T>
class Buffer
{
    public :

    /// \brief Type of the size
    typedef std::size_t size_type;

    /// \brief Allocate an array buffer of specified size
    ///
    /// \param n The size of the internal array of Buffer
    explicit Buffer (const size_type n = 0) : buffer_(NULL), size_(n)
    {
        if (size_)
            buffer_ = static_cast<T *>(
                    mkl_malloc(sizeof(T) * n, align_size()));
    }

    /// \brief Copy constructor
    ///
    /// \param buffer The Buffer to be copied
    Buffer (const Buffer<T> &buffer) : buffer_(NULL), size_(buffer.size_)
    {
        buffer_ = static_cast<T *>(
                mkl_malloc(sizeof(T) * size_, align_size()));
        for (size_type i = 0; i != size_; ++i)
            buffer_[i] = buffer[i];
    }

    /// \brief Assignment operator
    ///
    /// \param buffer The Buffer to be assigned
    /// \return The Buffer after assignemnt
    Buffer<T> & operator= (const Buffer<T> &buffer)
    {
        if (&buffer != this) {
            mkl_free(buffer_);
            buffer_ = static_cast<T *>(
                    mkl_malloc(sizeof(T) * size_, align_size()));
            size_ = buffer.size_;
            for (size_type i = 0; i != size_; ++i)
                buffer_[i] = buffer[i];
        }

        return *this;
    }

    ~Buffer ()
    {
        if (size_)
            mkl_free(buffer_);
    }

    /// \brief Get the size of the internal array
    size_type size () const
    {
        return size_;
    }

    /// \brief Check if it is empty
    ///
    /// \return True if empty, false otherwise
    bool empty () const
    {
        return size_;
    }

    /// \brief Resize the buffer
    ///
    /// \param n The new size of the buffer
    void resize (const size_type n)
    {
        if (size_ != n) {
            size_ = n;
            if (size_)
                mkl_free(buffer_);
            buffer_ = static_cast<T *>(
                    mkl_malloc(sizeof(T) * size_, align_size()));
        }
    }

    /// \brief Get the internal pointer of the allocated array
    T *get ()
    {
        return buffer_;
    }

    /// \brief Get the internal pointer of the allocated array
    const T *get () const
    {
        return buffer_;
    }

    /// \brief Implicit convert Buffer to pointer
    operator T* ()
    {
        return buffer_;
    }

    /// \brief Implicit convert Buffer to pointer
    operator const T* () const
    {
        return buffer_;
    }

    /// \brief Implicit convert Buffer to a void pointer, prevent \b delete
    operator void* ()
    {
        return buffer_;
    }

    /// \brief Implicit convert Buffer to a void pointer, prevent \b delete
    operator const void* () const
    {
        return buffer_;
    }

    /// \brief Read and write access to coefficient with bound check
    ///
    /// \param i The index number
    /// \return The \b lvalue of coefficient
    T & operator() (const size_type i)
    {
        if (i >= size_)
            throw std::out_of_range("Out of the range of the buffer");

        return buffer_[i];
    }

    /// \brief Read only access to coefficient with bound check
    ///
    /// \param i The index number
    /// \return The \b rvalue of coefficient
    const T & operator() (const size_type i) const
    {
        if (i >= size_)
            throw std::out_of_range("Out of the range of the buffer");

        return buffer_[i];
    }

    /// \brief Read and write access to coefficient without bound check
    ///
    /// \param i The index number
    /// \return The \b lvalue of coefficient
    T & operator[] (const size_type i)
    {
        return buffer_[i];
    }

    /// \brief Read only access to coefficient without bound check
    ///
    /// \param i The index number
    /// \return The \b rvalue of coefficient
    const T & operator[] (const size_type i) const
    {
        return buffer_[i];
    }

    private :

    T *buffer_;
    size_type size_;

    std::size_t align_size () const
    {
        if (sizeof(T) < V_SMC_ALIGN_SIZE && !(V_SMC_ALIGN_SIZE % sizeof(T)))
            return V_SMC_ALIGN_SIZE;
        else
            return sizeof(T);
    }
}; // class Buffer

} } // namespace vSMC::internal

#endif // V_SMC_CORE_BUFFER_HPP
