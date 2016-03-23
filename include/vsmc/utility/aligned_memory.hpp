//============================================================================
// vSMC/include/vsmc/utility/aligned_memory.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_UTILITY_ALIGNED_MEMORY
#define VSMC_UTILITY_ALIGNED_MEMORY

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/config.h>

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

#if VSMC_HAS_POSIX
#include <stdlib.h>
#elif defined(VSMC_MSVC)
#include <malloc.h>
#endif

#if VSMC_HAS_TBB_MALLOC
#include <tbb/scalable_allocator.h>
#endif

#if VSMC_HAS_MKL
#include <mkl_service.h>
#endif

/// \brief Default AlignedMemory type
/// \ingroup Config
#ifndef VSMC_ALIGNED_MEMORY_TYPE
#if VSMC_HAS_TBB_MALLOC
#define VSMC_ALIGNED_MEMORY_TYPE ::vsmc::AlignedMemoryTBB
#elif VSMC_HAS_MKL
#define VSMC_ALIGNED_MEMORY_TYPE ::vsmc::AlignedMemoryMKL
#elif VSMC_HAS_POSIX || defined(VSMC_MSVC)
#define VSMC_ALIGNED_MEMORY_TYPE ::vsmc::AlignedMemorySYS
#else
#define VSMC_ALIGNED_MEMORY_TYPE ::vsmc::AlignedMemorySTD
#endif
#endif

/// \brief Defualt alignment
/// \ingroup Config
#ifndef VSMC_ALIGNMENT
#define VSMC_ALIGNMENT 32
#endif

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY_POWER_OF_TWO(alignment)    \
    VSMC_RUNTIME_ASSERT(                                                      \
        (alignment != 0 && (alignment & (alignment - 1)) == 0),               \
        "**aligned_malloc** USED WITH ALIGNMENT NOT A POWER OF TWO")

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY_SIZEOF_VOID(alignemnt)     \
    VSMC_RUNTIME_ASSERT((alignment >= sizeof(void *)),                        \
        "**aligned_malloc** USED WITH ALIGNMENT LESS THAN sizeof(void *)")

#define VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY                            \
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY_POWER_OF_TWO(alignment);       \
    VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY_SIZEOF_VOID(alignment);

namespace vsmc
{

/// \brief Aligned memory using `std::malloc` and `std::free`
/// \ingroup AlignedMemory
///
/// \details
/// Memory allocated through this class is aligned but some bytes might be
/// wasted in each allocation.
class AlignedMemorySTD
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY;

        if (n == 0)
            return nullptr;

        void *orig_ptr = std::malloc(n + alignment + sizeof(void *));
        if (orig_ptr == nullptr)
            throw std::bad_alloc();

        uintptr_t address = reinterpret_cast<uintptr_t>(orig_ptr);
        uintptr_t offset = alignment - (address + sizeof(void *)) % alignment;
        void *ptr =
            reinterpret_cast<void *>(address + offset + sizeof(void *));
        void **orig = reinterpret_cast<void **>(address + offset);
        *orig = orig_ptr;

        return ptr;
    }

    static void aligned_free(void *ptr)
    {
        std::free(*reinterpret_cast<void **>(
            reinterpret_cast<uintptr_t>(ptr) - sizeof(void *)));
    }
}; // class AlignedMemmorySTD

#if VSMC_HAS_POSIX

/// \brief Aligned memory using native system aligned memory allocation
/// \ingroup AlignedMemory
///
/// \details
/// This class use `posix_memalign` and `free` on POSIX systems (Mac OS X,
/// Linux, etc.) and `_aligned_malloc` and `_aligned_free` on Windows.
class AlignedMemorySYS
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY;

        if (n == 0)
            return nullptr;

        void *ptr;
        if (posix_memalign(&ptr, alignment, n) != 0)
            throw std::bad_alloc();

        return ptr;
    }

    static void aligned_free(void *ptr) { free(ptr); }
}; // class AlignedMallocSYS

#elif defined(VSMC_MSVC)

class AlignedMemorySYS
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY;

        if (n == 0)
            return nullptr;

        void *ptr = _aligned_malloc(n, alignment);
        if (ptr == nullptr)
            throw std::bad_alloc();

        return ptr;
    }

    static void aligned_free(void *ptr) { _aligned_free(ptr); }
}; // class AlignedMemorySYS

#endif // VSMC_HAS_POSIX

#if VSMC_HAS_TBB_MALLOC

/// \brief Aligned memory using Intel TBB `scalable_aligned_malloc` and
/// `scalable_aligned_free`.
/// \ingroup AlignedMemory
class AlignedMemoryTBB
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY;

        if (n == 0)
            return nullptr;

        void *ptr = scalable_aligned_malloc(n, alignment);
        if (ptr == nullptr)
            throw std::bad_alloc();

        return ptr;
    }

    static void aligned_free(void *ptr) { scalable_aligned_free(ptr); }
}; // class AlignedMemoryTBB

#endif // VSMC_HAS_TBB_MALLOC

#if VSMC_HAS_MKL

/// \brief Aligned memory using Intel MKL `mkl_malloc` and `mkl_free`
/// \ingroup AlignedMemory
class AlignedMemoryMKL
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ALIGNED_MEMORY;

        if (n == 0)
            return nullptr;

        void *ptr = mkl_malloc(n, static_cast<int>(alignment));
        if (ptr == nullptr)
            throw std::bad_alloc();

        return ptr;
    }

    static void aligned_free(void *ptr) { mkl_free(ptr); }
}; // class AlignedMemoryMKL

#endif // VSMC_HAS_MKL

/// \brief Default AlignedMemory type
/// \ingroup AlignedMemory
using AlignedMemory = VSMC_ALIGNED_MEMORY_TYPE;

/// \brief Aligned allocator
/// \ingroup AlignedMemory
///
/// \tparam T The value type
/// \tparam Alignment The alignment requirement of memory, must be a power of
/// two and no less than `sizeof(void *)`.
/// \tparam Memory The memory management class. Must provides two static member
/// functions, `aligned_malloc` and `aligned_free`. The member function
/// `aligned_malloc` shall behave similar to `std::malloc` but take an
/// additional arguments for alignment. The member function `aligned_free`
/// shall behave just like `std::free`.
template <typename T, std::size_t Alignment = VSMC_ALIGNMENT,
    typename Memory = AlignedMemory>
class AlignedAllocator : public std::allocator<T>
{
    static_assert(Alignment != 0 && (Alignment & (Alignment - 1)) == 0,
        "**AlignedAllocator** USED WITH Alignment OTHER THAN A POWER OF TWO "
        "POSITIVE INTEGER");

    static_assert(Alignment >= sizeof(void *),
        "**AlignedAllocator** USED WITH Alignment LESS THAN sizeof(void *)");

    public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using pointer = T *;
    using const_pointer = const T *;
    using reference = typename std::add_lvalue_reference<T>::type;
    using const_reference = typename std::add_lvalue_reference<const T>::type;
    using is_always_equal = std::true_type;

    template <typename U>
    class rebind
    {
        public:
        using other = AlignedAllocator<U, Alignment, Memory>;
    }; // class rebind

    AlignedAllocator() = default;

    AlignedAllocator(const AlignedAllocator<T, Alignment, Memory> &) = default;

    template <typename U>
    AlignedAllocator(const AlignedAllocator<U, Alignment, Memory> &other)
        : std::allocator<T>(static_cast<std::allocator<U>>(other))
    {
    }

    static pointer allocate(size_type n, const void * = nullptr)
    {
        if (n == 0)
            return nullptr;

        return static_cast<pointer>(
            Memory::aligned_malloc(sizeof(T) * n, Alignment));
    }

    static void deallocate(pointer ptr, size_type)
    {
        if (ptr != nullptr)
            Memory::aligned_free(ptr);
    }
}; // class AlignedAllocator

template <std::size_t Alignment, typename Memory>
class AlignedAllocator<void, Alignment, Memory>
{
    using value_type = void;
    using pointer = void *;
    using const_pointer = const void *;

    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment, Memory>;
    };
}; // class AlignedAllocator

template <std::size_t Alignment, typename Memory>
class AlignedAllocator<const void, Alignment, Memory>
{
    using value_type = const void;
    using pointer = const void *;
    using const_pointer = const void *;

    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment, Memory>;
    };
}; // class AlignedAllocator

template <typename T1, typename T2, std::size_t Alignment, typename Memory>
inline bool operator==(const AlignedAllocator<T1, Alignment, Memory> &,
    const AlignedAllocator<T2, Alignment, Memory> &)
{
    return true;
}

template <typename T1, typename T2, std::size_t Alignment, typename Memory>
inline bool operator!=(const AlignedAllocator<T1, Alignment, Memory> &,
    const AlignedAllocator<T2, Alignment, Memory> &)
{
    return false;
}

/// \brief AlignedAllocator with proper alignement detected for type
/// \ingroup AlignedMemory
template <typename T>
using Allocator = AlignedAllocator<T,
    std::is_scalar<T>::value ? 32 : (alignof(T) > 16 ? alignof(T) : 16)>;

/// \brief Vector type using Allocator
/// \ingroup AlignedMemory
template <typename T>
using Vector = std::vector<T, Allocator<T>>;

} // namespace vsmc

#endif // VSMC_UTILITY_ALIGNED_MEMORY
