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

#ifndef VSMC_UTILITY_ALIGNED_MEMORY_HPP
#define VSMC_UTILITY_ALIGNED_MEMORY_HPP

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

/// \brief The default alignment for scalar type
/// \ingroup Config
#ifndef VSMC_ALIGNMENT
#define VSMC_ALIGNMENT 32
#endif

/// \brief The minimum alignment for any type
/// \ingroup Config
#ifndef VSMC_ALIGNMENT_MIN
#define VSMC_ALIGNMENT_MIN 16
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

/// \brief Allocator::construct default behavior for scalar type
#ifndef VSMC_CONSTRUCT_SCALAR
#define VSMC_CONSTRUCT_SCALAR 0
#endif

/// \brief Define class member `new` and `delete` using Allocator
/// \ingroup AlignedMemory
///
/// \details
/// The behavior of the custom `operator new` is sligtly different than
/// the standard ones. There will be no `new` handler called in the case of
/// failure to allocate memroy. Another way to view this is that the class tha
/// uses these custom `operator new` has its own `new` handler which does
/// exactly nothing.
#define VSMC_DEFINE_NEW_DELETE(Class)                                         \
    public:                                                                   \
    static void *operator new(std::size_t n)                                  \
    {                                                                         \
        void *ptr = ::vsmc::AlignedMemory::aligned_malloc(                    \
            n, ::vsmc::AlignmentTrait<Class>::value);                         \
        if (ptr == nullptr)                                                   \
            throw std::bad_alloc();                                           \
                                                                              \
        return ptr;                                                           \
    }                                                                         \
                                                                              \
    static void *operator new[](std::size_t n)                                \
    {                                                                         \
        void *ptr = ::vsmc::AlignedMemory::aligned_malloc(                    \
            n, ::vsmc::AlignmentTrait<Class>::value);                         \
        if (ptr == nullptr)                                                   \
            throw std::bad_alloc();                                           \
                                                                              \
        return ptr;                                                           \
    }                                                                         \
                                                                              \
    static void *operator new(std::size_t, void *ptr) { return ptr; }         \
                                                                              \
    static void *operator new[](std::size_t, void *ptr) { return ptr; }       \
                                                                              \
    static void operator delete(void *ptr)                                    \
    {                                                                         \
        ::vsmc::AlignedMemory::aligned_free(ptr);                             \
    }                                                                         \
                                                                              \
    static void operator delete[](void *ptr)                                  \
    {                                                                         \
        ::vsmc::AlignedMemory::aligned_free(ptr);                             \
    }                                                                         \
                                                                              \
    static void operator delete(void *, void *) {}                            \
                                                                              \
    static void operator delete[](void *, void *) {}

namespace vsmc
{

namespace internal
{

template <typename T, bool = std::is_scalar<T>::value>
class AlignmentTraitImpl
{
    public:
    static constexpr std::size_t value =
        alignof(T) > VSMC_ALIGNMENT_MIN ? alignof(T) : VSMC_ALIGNMENT_MIN;
}; // class AlignmentTraitImpl

template <typename T>
class AlignmentTraitImpl<T, false>
{
    public:
    static constexpr std::size_t value = VSMC_ALIGNMENT;
}; // class AlignmentTraitImpl

} // namespace vsmc::internal

/// \brief Alignment of a given type
///
/// \details
/// For scalar type, such as `double`, `value` is VSMC_ALIGNMENT, which shall
/// be big enough for SIME aligned operations. For other types, it return
/// `VSMC_ALIGNMENT_MIN` if `alignof(T)` is smaller, otherwise `alignof(T)`
template <typename T>
class AlignmentTrait : public internal::AlignmentTraitImpl<T>
{
};

/// \brief Aligned memory using `std::malloc` and `std::free`
/// \ingroup AlignedMemory
///
/// \details
/// Memory allocated through this class is aligned but some bytes might be
/// wasted in each allocation.
class AlignedMemorySTD
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment) noexcept
    {
        void *orig_ptr =
            std::malloc((n > 0 ? n : 1) + alignment + sizeof(void *));
        if (orig_ptr == nullptr)
            return nullptr;

        uintptr_t address = reinterpret_cast<uintptr_t>(orig_ptr);
        uintptr_t offset = alignment - (address + sizeof(void *)) % alignment;
        void *ptr =
            reinterpret_cast<void *>(address + offset + sizeof(void *));
        void **orig = reinterpret_cast<void **>(address + offset);
        *orig = orig_ptr;

        return ptr;
    }

    static void aligned_free(void *ptr) noexcept
    {
        if (ptr != nullptr) {
            std::free(*reinterpret_cast<void **>(
                reinterpret_cast<uintptr_t>(ptr) - sizeof(void *)));
        }
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
    static void *aligned_malloc(std::size_t n, std::size_t alignment) noexcept
    {
        void *ptr;
        if (posix_memalign(&ptr, alignment, n > 0 ? n : 1) != 0)
            return nullptr;
        return ptr;
    }

    static void aligned_free(void *ptr) noexcept
    {
        if (ptr != nullptr)
            free(ptr);
    }
}; // class AlignedMallocSYS

#elif defined(VSMC_MSVC)

class AlignedMemorySYS
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment) noexcept
    {
        return _aligned_malloc(n > 0 ? n : 1, alignment);
    }

    static void aligned_free(void *ptr) noexcept
    {
        if (ptr != nullptr)
            _aligned_free(ptr);
    }
}; // class AlignedMemorySYS

#endif // VSMC_HAS_POSIX

#if VSMC_HAS_TBB_MALLOC

/// \brief Aligned memory using Intel TBB `scalable_aligned_malloc` and
/// `scalable_aligned_free`.
/// \ingroup AlignedMemory
class AlignedMemoryTBB
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment) noexcept
    {
        return scalable_aligned_malloc(n > 0 ? n : 1, alignment);
    }

    static void aligned_free(void *ptr) noexcept
    {
        if (ptr != nullptr)
            scalable_aligned_free(ptr);
    }
}; // class AlignedMemoryTBB

#endif // VSMC_HAS_TBB_MALLOC

#if VSMC_HAS_MKL

/// \brief Aligned memory using Intel MKL `mkl_malloc` and `mkl_free`
/// \ingroup AlignedMemory
class AlignedMemoryMKL
{
    public:
    static void *aligned_malloc(std::size_t n, std::size_t alignment) noexcept
    {
        return mkl_malloc(n > 0 ? n : 1, static_cast<int>(alignment));
    }

    static void aligned_free(void *ptr) noexcept
    {
        if (ptr != nullptr)
            mkl_free(ptr);
    }
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
/// \tparam ConstructScalar Should construct(ptr) initialize a scalar type with
/// zero or left it uninitialized.
/// \tparam Memory The memory management class. Must provides two static member
/// functions, `aligned_malloc` and `aligned_free`. The member function
/// `aligned_malloc` shall behave similar to `std::malloc` but take an
/// additional argument as alignment requirement. It need to be able to handle
/// any alignment that is a power of two, other than zero. It need to return a
/// non-null pointer even if the size is zero. It shall only return a null
/// pointer if it fails to allocated the memory. It shall not throw any
/// exceptions. The member function `aligned_free` shall behave just like
/// `std::free`. It shall be able to handle a null pointer as its input.
template <typename T, bool ConstructScalar = VSMC_CONSTRUCT_SCALAR != 0,
    std::size_t Alignment = AlignmentTrait<T>::value,
    typename Memory = AlignedMemory>
class Allocator : public std::allocator<T>
{
    static_assert(Alignment != 0 && (Alignment & (Alignment - 1)) == 0,
        "**Allocator** USED WITH Alignment OTHER THAN A POWER OF TWO "
        "POSITIVE INTEGER");

    static_assert(Alignment >= sizeof(void *),
        "**Allocator** USED WITH Alignment LESS THAN sizeof(void *)");

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
        using other = Allocator<U, ConstructScalar, Alignment, Memory>;
    }; // class rebind

    Allocator() = default;

    Allocator(
        const Allocator<T, ConstructScalar, Alignment, Memory> &) = default;

    Allocator(Allocator<T, ConstructScalar, Alignment, Memory> &&) = default;

    Allocator<T, ConstructScalar, Alignment, Memory> &operator=(
        const Allocator<T, ConstructScalar, Alignment, Memory> &) = default;

    Allocator<T, ConstructScalar, Alignment, Memory> &operator=(
        Allocator<T, ConstructScalar, Alignment, Memory> &&) = default;

    template <typename U>
    Allocator(const Allocator<U, ConstructScalar, Alignment, Memory> &other)
        : std::allocator<T>(static_cast<std::allocator<U>>(other))
    {
    }

    template <typename U>
    Allocator(Allocator<U, ConstructScalar, Alignment, Memory> &&other)
        : std::allocator<T>(std::move(static_cast<std::allocator<U>>(other)))
    {
    }

    pointer allocate(size_type n, const void * = nullptr)
    {
        pointer ptr = static_cast<pointer>(Memory::aligned_malloc(
            (n > 0 ? n : 1) * sizeof(value_type), Alignment));
        if (ptr == nullptr)
            throw std::bad_alloc();

        return ptr;
    }

    void deallocate(pointer ptr, size_type = 0) noexcept
    {
        if (ptr != nullptr)
            Memory::aligned_free(ptr);
    }

    template <typename U>
    void construct(U *ptr)
    {
        construct_dispatch(
            ptr, std::integral_constant<bool,
                     (ConstructScalar || !std::is_scalar<U>::value)>());
    }

    template <typename U, typename Arg, typename... Args>
    void constrct(U *ptr, Arg &&arg, Args &&... args)
    {
        std::allocator<T>::construct(
            ptr, std::forward<Arg>(arg), std::forward<Args>(args)...);
    }

    private:
    template <typename U>
    void construct_dispatch(U *ptr, std::true_type)
    {
        std::allocator<T>::construct(ptr);
    }

    template <typename U>
    void construct_dispatch(U *, std::false_type)
    {
    }
}; // class Allocator

template <bool ConstructScalar, std::size_t Alignment, typename Memory>
class Allocator<void, ConstructScalar, Alignment, Memory>
{
    using value_type = void;
    using pointer = void *;
    using const_pointer = const void *;

    template <class U>
    struct rebind {
        using other = Allocator<U, ConstructScalar, Alignment, Memory>;
    };
}; // class Allocator

template <bool ConstructScalar, std::size_t Alignment, typename Memory>
class Allocator<const void, ConstructScalar, Alignment, Memory>
{
    using value_type = const void;
    using pointer = const void *;
    using const_pointer = const void *;

    template <class U>
    struct rebind {
        using other = Allocator<U, ConstructScalar, Alignment, Memory>;
    };
}; // class Allocator

template <typename T1, typename T2, bool ConstructScalar,
    std::size_t Alignment, typename Memory>
inline bool operator==(
    const Allocator<T1, ConstructScalar, Alignment, Memory> &,
    const Allocator<T2, ConstructScalar, Alignment, Memory> &)
{
    return true;
}

template <typename T1, typename T2, bool ConstructScalar,
    std::size_t Alignment, typename Memory>
inline bool operator!=(
    const Allocator<T1, ConstructScalar, Alignment, Memory> &,
    const Allocator<T2, ConstructScalar, Alignment, Memory> &)
{
    return false;
}

/// \brief Vector type using Allocator
/// \ingroup AlignedMemory
template <typename T>
using Vector = std::vector<T, Allocator<T>>;

} // namespace vsmc

#endif // VSMC_UTILITY_ALIGNED_MEMORY_HPP
