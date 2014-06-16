#ifndef VSMC_UTILITY_ALIGNED_ALLOCATOR
#define VSMC_UTILITY_ALIGNED_ALLOCATOR

#include <vsmc/internal/common.hpp>
#include <cstdlib>
#include <memory>

namespace vsmc {

namespace internal {

#if defined(__APPLE__) || defined(__MACOSX)
#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_5)
/// \brief Use POSIX posix_memalign
/// \ingroup Config
#ifndef VSMC_USE_POSIX_MEMALIGN
#define VSMC_USE_POSIX_MEMALIGN 1
#endif
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_5)
#elif defined(__linux__)
#include <features.h>
#if defined(_POSIX_C_SOURCE) &&  _POSIX_C_SOURCE >= 200112L
#ifndef VSMC_USE_POSIX_MEMALIGN
#define VSMC_USE_POSIX_MEMALIGN 1
#endif
#endif // defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
#if defined(_XOPEN_SOURCE) &&  _XOPEN_SOURCE >= 600
#ifndef VSMC_USE_POSIX_MEMALIGN
#define VSMC_USE_POSIX_MEMALIGN 1
#endif
#endif // defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600
#endif

#ifndef VSMC_USE_POSIX_MEMALIGN
#define VSMC_USE_POSIX_MEMALIGN 0
#endif

#if VSMC_USE_POSIX_MEMALIGN

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    void *ptr;
    if (posix_memalign(&ptr, alignment, n) != 0)
        throw std::bad_alloc();

    return ptr;
}

inline void aligned_free (void *ptr) {free(ptr);}

#elif VSMC_USE_MKL

#include <mkl_service.h>

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{return mkl_malloc(n, static_cast<int>(alignment));}

inline void aligned_free (void *ptr) {mkl_free(ptr);}

#elif defined(_WIN32)

#include <malloc.h>

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{return _aligned_malloc(n, alignment);}

inline void aligned_free (void *ptr) {aligned_free(ptr);}

#else

#error Aligned allocation not implemented

#endif // VSMC_USE_POSIX_MEMALIGN

} // namespace vsmc::internal

template <typename T, std::size_t Alignment>
class AlignedAllocator
{
    public :

    typedef T value_type;
    typedef T * pointer;
    typedef const T * const_pointer;
    typedef T & reference;
    typedef const T & const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef cxx11::true_type propagate_on_container_move_assignment;

    template <typename U> struct rebind
    {typedef AlignedAllocator<U, Alignment> other;};

    AlignedAllocator () {}

    AlignedAllocator (const AlignedAllocator<T, Alignment> &) {}

    template <typename U>
    AlignedAllocator (const AlignedAllocator<U, Alignment> &) {}

    ~AlignedAllocator () {}

    static VSMC_CONSTEXPR size_type max_size ()
    {return static_cast<size_type>(~static_cast<size_type>(0)) / sizeof(T);}

    static pointer address (reference obj)
    {
        std::allocator<T> alloc;
        return alloc.address(obj);
    }

    static const_pointer address (const_reference obj)
    {
        std::allocator<T> alloc;
        return alloc.address(obj);
    }

    static pointer allocate (size_type n, const void * = VSMC_NULLPTR)
    {
        if (n == 0)
            return VSMC_NULLPTR;

        return static_cast<pointer>(
                internal::aligned_malloc(sizeof(T) * n, Alignment));
    }

    static void deallocate (pointer ptr, size_type n)
    {
        if (n == 0)
            return;

        internal::aligned_free(ptr);
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES && VSMC_HAS_CXX11_VARIADIC_TEMPLATES
    template <typename U, typename... Args>
    static void construct (pointer ptr, Args &&... args)
    {
        ::new (const_cast<void *>(static_cast<const volatile void *>(ptr)))
            U(std::forward<Args>(args)...);
    }
#else
    static void construct (pointer ptr, const_reference val)
    {new (const_cast<void *>(static_cast<const volatile void *>(ptr))) T(val);}
#endif

    static void destory (pointer ptr) {ptr->~T();}

    friend bool operator== (
            const AlignedAllocator<T, Alignment> &,
            const AlignedAllocator<T, Alignment> &)
    {return true;}

    friend bool operator!= (
            const AlignedAllocator<T, Alignment> &alloc1,
            const AlignedAllocator<T, Alignment> &alloc2)
    {return !(alloc1 == alloc2);}
}; // class AlignedAllocator

} // namespace vsmc

#endif // VSMC_UTILITY_ALIGNED_ALLOCATOR
