//============================================================================
// include/vsmc/utility/aligned_allocator.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_ALIGNED_ALLOCATOR
#define VSMC_UTILITY_ALIGNED_ALLOCATOR

#include <vsmc/internal/common.hpp>

#if VSMC_HAS_POSIX
#include <stdlib.h>
#elif defined(_WIN32)
#include <malloc.h>
#elif VSMC_HAS_TBB
#include <tbb/scalable_allocator.h>
#elif VSMC_HAS_MKL
#include <mkl_service.h>
#endif

#define VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR_POWER_OF_TWO(Alignment) \
    VSMC_STATIC_ASSERT((Alignment != 0 && (Alignment & (Alignment - 1)) == 0),\
            USE_AlignedAllocator_WITH_ALIGNEMNT_NOT_A_POWER_OF_TWO)

#define VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR_SIZEOF_VOID(Alignemnt) \
    VSMC_STATIC_ASSERT((Alignment >= sizeof(void *)),                        \
            USE_AlignedAllocator_WITH_ALIGNMENT_LESS_THAN_SIZEOF_VOID_POINTER)

#define VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR \
    VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR_POWER_OF_TWO(Alignment);    \
    VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR_SIZEOF_VOID(Alignment);

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR_POWER_OF_TWO(alignment) \
    VSMC_RUNTIME_ASSERT((alignment != 0 && (alignment & (alignment - 1)) == 0),\
            "**aligned_malloc** USED WITH ALIGNMENT NOT A POWER OF TWO")

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR_SIZEOF_VOID(alignemnt) \
    VSMC_RUNTIME_ASSERT((alignment >= sizeof(void *)),                        \
            "**aligned_malloc** USED WITH ALIGNMENT LESS THAN sizeof(void *)")

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR \
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR_POWER_OF_TWO(alignment);    \
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR_SIZEOF_VOID(alignment);

namespace vsmc {

#if VSMC_HAS_POSIX

/// \brief Aligned malloc
/// \ingroup AlignedAllocator
inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR;

    if (n == 0)
        return VSMC_NULLPTR;

    void *ptr;
    if (posix_memalign(&ptr, alignment, n) != 0)
        throw std::bad_alloc();

    return ptr;
}

/// \brief Aligned free
/// \ingroup AlignedAllocator
inline void aligned_free (void *ptr) {free(ptr);}

#elif defined(_WIN32)

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR;

    if (n == 0)
        return VSMC_NULLPTR;

    void *ptr = _aligned_malloc(n, alignment);
    if (ptr == VSMC_NULLPTR)
        throw std::bad_alloc();

    return ptr;
}

inline void aligned_free (void *ptr) {_aligned_free(ptr);}

#elif VSMC_HAS_TBB

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR;

    if (n == 0)
        return VSMC_NULLPTR;

    void *ptr = scalable_aligned_malloc(n, alignment);
    if (ptr == VSMC_NULLPTR)
        throw std::bad_alloc();

    return ptr;
}

inline void aligned_free (void *ptr) {scalable_aligned_free(ptr);}

#elif VSMC_HAS_MKL

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR;

    if (n == 0)
        return VSMC_NULLPTR;

    void *ptr = mkl_malloc(n, static_cast<int>(alignment));
    if (ptr == VSMC_NULLPTR)
        throw std::bad_alloc();

    return ptr;
}

inline void aligned_free (void *ptr) {mkl_free(ptr);}

#else // VSMC_HAS_POSIX

inline void *aligned_malloc (std::size_t n, std::size_t alignment)
{
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_ALLOCATOR;

    if (n == 0)
        return VSMC_NULLPTR;

    void *orig_ptr = std::malloc(n + alignment + sizeof(void *));
    if (orig_ptr == VSMC_NULLPTR)
        throw std::bad_alloc();

    uintptr_t address = reinterpret_cast<uintptr_t>(orig_ptr);
    uintptr_t offset = alignment - (address + sizeof(void *)) % alignment;
    void *ptr = reinterpret_cast<void *>(address + offset + sizeof(void *));
    void **orig = reinterpret_cast<void **>(address + offset);
    *orig = orig_ptr;

    return ptr;
}

inline void aligned_free (void *ptr)
{
    std::free(*reinterpret_cast<void **>(
                reinterpret_cast<uintptr_t>(ptr) - sizeof(void *)));
}

#endif // VSMC_HAS_POSIX

/// \brief Aligned allocator
/// \ingroup AlignedAllocator
///
/// \details
/// This allocator can be used in place of `std::allocator`. The returned
/// pointer by `allocate` is aligned with `Alignment`, which must be power of
/// two and no less than `sizeof(void *)`.
template <typename T, std::size_t Alignment = 32>
class AlignedAllocator : public std::allocator<T>
{
    public :

    typedef typename std::allocator<T>::size_type size_type;
    typedef typename std::allocator<T>::pointer pointer;

    template <typename U> struct rebind
    {typedef AlignedAllocator<U, Alignment> other;};

    AlignedAllocator () {VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR;}

    AlignedAllocator (const AlignedAllocator<T, Alignment> &other) :
        std::allocator<T>(other)
    {VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR;}

    template <typename U>
    AlignedAllocator (const AlignedAllocator<U, Alignment> &other) :
        std::allocator<T>(static_cast<std::allocator<U> >(other))
    {VSMC_STATIC_ASSERT_UTILITY_ALIGNED_ALLOCATOR;}

    ~AlignedAllocator () {}

    pointer allocate (size_type n, const void * = VSMC_NULLPTR)
    {return static_cast<pointer>(aligned_malloc(sizeof(T) * n, Alignment));}

    void deallocate (pointer ptr, size_type) {aligned_free(ptr);}
}; // class AlignedAllocator

} // namespace vsmc

#endif // VSMC_UTILITY_ALIGNED_ALLOCATOR
