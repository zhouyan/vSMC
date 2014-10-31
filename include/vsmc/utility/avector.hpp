//============================================================================
// include/vsmc/utility/vector.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_AVECTOR_HPP
#define VSMC_UTILITY_AVECTOR_HPP

#include <vsmc/internal/common.hpp>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <utility>

#define VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T) \
    VSMC_STATIC_ASSERT((::vsmc::cxx11::is_arithmetic<T>::value),             \
            USE_AVector_WITH_A_TYPE_THAT_IS_NOT_ARITHMETIC)

#define VSMC_RUNTIME_ASSERT_UTILITY_AVECTOR_RANGE(i, N) \
    VSMC_RUNTIME_ASSERT((i < N),                                             \
            ("**AVector** USED WITH AN INDEX OUT OF RANGE"))

#if defined(__APPLE__) || defined(__MACOSX)
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_5)
#ifndef VSMC_HAS_POSIX_MEMALIGN
#define VSMC_HAS_POSIX_MEMALIGN 1
#endif
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_5)
#elif defined(__linux__)
#include <features.h>
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
#ifndef VSMC_HAS_POSIX_MEMALIGN
#define VSMC_HAS_POSIX_MEMALIGN 1
#endif
#endif // defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
#if defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600
#ifndef VSMC_HAS_POSIX_MEMALIGN
#define VSMC_HAS_POSIX_MEMALIGN 1
#endif
#endif // defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600
#endif

#ifndef VSMC_HAS_POSIX_MEMALIGN
#define VSMC_HAS_POSIX_MEMALIGN 0
#endif

namespace vsmc {

namespace internal {

#if VSMC_HAS_MKL

#include <mkl_service.h>

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{return mkl_malloc(n, static_cast<int>(alignment));}

inline void aligned_free (void *ptr) {mkl_free(ptr);}

#elif VSMC_HAS_POSIX_MEMALIGN

#include <stdlib.h>

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    void *ptr;
    if (posix_memalign(&ptr, alignment, n) != 0)
        throw std::bad_alloc();

    return ptr;
}

inline void aligned_free (void *ptr) {free(ptr);}

#elif defined(_WIN32)

#include <malloc.h>

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{return _aligned_malloc(n, alignment);}

inline void aligned_free (void *ptr) {_aligned_free(ptr);}

#else

#include <cstdlib>

inline void *aligned_malloc (std::size_t n, std::size_t)
{return std::malloc(n);}

inline void aligned_free (void *ptr) {std::free(ptr);}

#endif // VSMC_HAS_MKL

} // namespace vsmc::internal

/// \brief Dynamic vector
/// \ingroup AVector
///
/// \details
/// This is similiar to `std::vector`, but only accepts arithmetic types. It
/// does not have all the functionalities of the STL container. In particular,
/// except `resize`, all methods that can change the size of the vector is not
/// provided, such as `push_back`.
template <typename T>
class AVector
{
    public :

    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef value_type & reference;
    typedef const value_type & const_reference;
    typedef value_type * pointer;
    typedef const value_type * const_pointer;
    typedef pointer iterator;
    typedef const_pointer const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    AVector () : size_(0), data_(VSMC_NULLPTR)
    {VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T);}

    explicit AVector (size_type count) : size_(0), data_(VSMC_NULLPTR)
    {
        VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T);

        resize(count);
        std::memset(data_, 0, sizeof(T) * size_);
    }

    AVector (size_type count, const value_type &value) :
        size_(0), data_(VSMC_NULLPTR)
    {
        VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T);

        resize(count);
        for (size_type i = 0; i != count; ++i)
            data_[i] = value;
    }

    template <typename InputIter>
    AVector (InputIter first, InputIter last,
            typename cxx11::enable_if<!(
                (cxx11::is_same<InputIter, size_type>::value ||
                 cxx11::is_convertible<InputIter, size_type>::value) &&
                (cxx11::is_same<InputIter, value_type>::value ||
                 cxx11::is_convertible<InputIter, value_type>::value))>::
            type * = VSMC_NULLPTR) : size_(0), data_(VSMC_NULLPTR)
    {
        VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T);

        using std::distance;

        size_type count = distance(first, last);
        resize(count);
        for (size_type i = 0; i != size_; ++i, ++first)
            data_[i] = *first;
    }

    AVector (const AVector<T> &other) : size_(0), data_(VSMC_NULLPTR)
    {
        VSMC_STATIC_ASSERT_UTILITY_AVECTOR_TYPE(T);

        resize(other.size_);
        copy(other.data_);
    }

    AVector<T> &operator= (const AVector<T> &other)
    {
        if (this != &other) {
            resize(other.size_);
            copy(other.data_);
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    AVector (AVector<T> &&other) : size_(other.size_), data_(other.data_)
    {
        other.size_ = 0;
        other.data_ = VSMC_NULLPTR;
    }

    AVector<T> &operator= (AVector<T> &other)
    {swap(other); return *this;}
#endif

    ~AVector () {if (data_ != VSMC_NULLPTR) deallocate();}

    reference at (size_type i)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_AVECTOR_RANGE(i, size_);

        return data_[i];
    }

    const_reference at (size_type i) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_AVECTOR_RANGE(i, size_);

        return data_[i];
    }

    reference operator[] (size_type i) {return data_[i];}

    const_reference operator[] (size_type i) const {return data_[i];}

    reference front () {return data_[0];}

    const_reference front () const {return data_[0];}

    reference back () {return data_[size_ - 1];}

    const_reference back () const {return data_[size_ - 1];}

    pointer data () {return data_;}

    const_pointer data () const {return data_;}

    iterator begin () {return data_;}

    iterator end () {return data_ + size_;}

    const_iterator begin () const {return data_;}

    const_iterator end () const {return data_ + size_;}

    const_iterator cbegin () const {return data_;}

    const_iterator cend () const {return data_ + size_;}

    reverse_iterator rbegin () {return reverse_iterator(end());}

    reverse_iterator rend () {return reverse_iterator(begin());}

    const_reverse_iterator rbegin () const
    {return const_reverse_iterator(end());}

    const_reverse_iterator rend () const
    {return const_reverse_iterator(begin());}

    const_reverse_iterator crbegin () const
    {return const_reverse_iterator(cend());}

    const_reverse_iterator crend () const
    {return const_reverse_iterator(cbegin());}

    bool empty () const {return size_ == 0;}

    size_type size () const {return size_;}

    static VSMC_CONSTEXPR size_type max_size ()
    {return static_cast<size_type>(~static_cast<size_type>(0)) / sizeof(T);}

    size_type capacity () const {return size_;}

    void resize (size_type new_size)
    {
        if (new_size == size_)
            return;

        pointer new_data = allocate(new_size);
        if (data_ != VSMC_NULLPTR)
            deallocate();
        size_ = new_size;
        data_ = new_data;
    }

    void swap (AVector<T> &other)
    {
        using std::swap;

        if (this != &other) {
            swap(size_, other.size_);
            swap(data_, other.data_);
        }
    }

    friend inline bool operator== (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {
        if (vec1.size() != vec2.size())
            return false;
        for (size_type i = 0; i != vec1.size(); ++i)
            if (vec1[i] != vec2[i])
                return false;

        return true;
    }

    friend inline bool operator!= (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {return !(vec1 == vec2);}

    friend inline bool operator< (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {
        for (size_type i = 0; i != vec1.size() && i != vec2.size(); ++i) {
            if (vec1[i] < vec2[i])
                return true;
            if (vec2[i] < vec1[i])
                return false;
        }

        if (vec1.size() < vec2.size())
            return true;

        return false;
    }

    friend inline bool operator> (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {
        for (size_type i = 0; i != vec1.size() && i != vec2.size(); ++i) {
            if (vec1[i] > vec2[i])
                return true;
            if (vec2[i] > vec1[i])
                return false;
        }

        if (vec1.size() > vec2.size())
            return true;

        return false;
    }

    friend inline bool operator<= (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {return !(vec1 > vec2);}

    friend inline bool operator>= (
            const AVector<T> &vec1, const AVector<T> &vec2)
    {return !(vec1 < vec2);}

    private :

    size_type size_;
    pointer data_;

    pointer allocate (size_type new_size)
    {
        return static_cast<pointer>(internal::aligned_malloc(
                    sizeof(T) * new_size, 32));
    }

    void deallocate () {internal::aligned_free(data_);}

    void copy (const_pointer new_data)
    {
        if (data_ + size_ < new_data || new_data + size_ < data_)
            std::memcpy(data_, new_data, sizeof(T) * size_);
        else
            std::memmove(data_, new_data, sizeof(T) * size_);
    }
}; // class AVector

template <typename T>
inline void swap (AVector<T> &vec1, AVector<T> &vec2) {vec1.swap(vec2);}

} // namespace vsmc

#endif // VSMC_UTILITY_AVECTOR_HPP
