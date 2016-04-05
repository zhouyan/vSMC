//============================================================================
// vSMC/include/vsmc/utility/static_vector.hpp
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

#ifndef VSMC_UTILITY_STATIC_VECTOR_HPP
#define VSMC_UTILITY_STATIC_VECTOR_HPP

#include <vsmc/internal/basic.hpp>
#include <vsmc/utility/aligned_memory.hpp>

#if VSMC_HAS_TBB
#include <tbb/combinable.h>
#endif

#define VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_AT(pos)                     \
    VSMC_RUNTIME_ASSERT(                                                      \
        (pos < size()), "**StaticVectorN::at INDEX OUT OF RANGE")

#if VSMC_HAS_TBB
#define VSMC_BUFFER(var, T, n)                                                \
    static ::tbb::combinable<Vector<T, AllocatorCache<T>>> var##_tls;         \
    auto &var = var##_tls.local();                                            \
    var.resize(static_cast<std::size_t>(n));
#else // VSMC_HAS_TBB
#define VSMC_BUFFER(var, T, n)                                                \
    StaticVector<T, internal::BufferSize<T>::value> var(                      \
        static_cast<std::size_t>(n));
#endif // VSMC_HAS_TBB

namespace vsmc
{

#if VSMC_HAS_TBB
namespace internal
{

template <typename T, int = 0>
inline Vector<T> &buffer(std::size_t n)
{
    static tbb::combinable<Vector<T>> tls;

    Vector<T> &local = tls.local();
    local.resize(n);

    return local;
}

} // namespace internal
#endif // VSMC_HAS_TBB

/// \brief Vector with statically allocated space
/// \ingroup StaticVector
///
/// \details
/// This is sort of a combination of `std::array` and `std::vector`. If the
/// template parameter is bigger than zero, then a static array is allocated.
/// The size of the vector can be specified at construction but cannot be
/// changed later. If the construction size n is bigger than N, dynamic memory
/// will be used. The vector size is the maixmum of N and n.
template <typename T, std::size_t N, typename Alloc = Allocator<T>>
class StaticVectorN
{
    static_assert(std::is_same<T, typename Alloc::value_type>::value,
        "**StaticVectorN** USED WITH Alloc HAVING DIFFERENT value_type");

    public:
    using value_type = T;
    using allocator_type = Alloc;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = typename std::allocator_traits<Alloc>::pointer;
    using const_pointer = typename std::allocator_traits<Alloc>::const_pointer;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    StaticVectorN() noexcept(
        std::is_nothrow_default_constructible<allocator_type>::value)
        : data_(array_.data())
    {
    }

    explicit StaticVectorN(const allocator_type &alloc) noexcept(
        std::is_nothrow_copy_constructible<allocator_type>::vlaue)
        : vector_(alloc), data_(array_.data())
    {
    }

    explicit StaticVectorN(size_type count)
        : vector_(count <= N ? 0 : count)
        , data_(count <= N ? array_.data() : vector_.data())
    {
    }

    explicit StaticVectorN(size_type count, const allocator_type &alloc)
        : vector_(count <= N ? 0 : count, alloc)
        , data_(count <= N ? array_.data() : vector_.data())
    {
    }

    explicit StaticVectorN(size_type count, const_reference value)
        : vector_(count <= N ? 0 : count, value)
        , data_(count <= N ? array_.data() : vector_.data())
    {
        if (count <= N)
            std::fill_n(array_.begin(), array_.begin() + count, value);
    }

    explicit StaticVectorN(
        size_type count, const_reference value, const allocator_type &alloc)
        : vector_(count <= N ? 0 : count, value, alloc)
        , data_(count <= N ? array_.data() : vector_.data())
    {
        if (count <= N)
            std::fill_n(array_.begin(), array_.begin() + count, value);
    }

    StaticVectorN(const StaticVectorN<T, N, Alloc> &other)
        : array_(other.array_)
        , vector_(other.vector_)
        , data_(vector_.empty() ? array_.data() : vector_.data())
    {
    }

    StaticVectorN(
        const StaticVectorN<T, N, Alloc> &other, const allocator_type &alloc)
        : array_(other.array_)
        , vector_(other.vector_, alloc)
        , data_(vector_.empty() ? array_.data() : vector_.data())
    {
    }

    StaticVectorN(StaticVectorN<T, N, Alloc> &&other)
        : array_(std::move(other.array_))
        , vector_(std::move(other.vector_))
        , data_(vector_.empty() ? array_.data() : vector_.data())
    {
    }

    StaticVectorN(
        StaticVectorN<T, N, Alloc> &&other, const allocator_type &alloc)
        : array_(std::move(other.array_))
        , vector_(std::move(other.vector_), alloc)
        , data_(vector_.empty() ? array_.data() : vector_.data())
    {
    }

    StaticVectorN<T, N, Alloc> &operator=(
        const StaticVectorN<T, N, Alloc> &other)
    {
        if (this != other) {
            array_ = other.array_;
            vector_ = other.vector_;
            data_ = vector_.empty() ? array_.data() : vector_.data();
        }

        return *this;
    }

    StaticVectorN<T, N, Alloc> &operator=(StaticVectorN<T, N, Alloc> &&other)
    {
        if (this != other) {
            array_ = std::move(other.array_);
            vector_ = std::move(other.vector_);
            data_ = vector_.empty() ? array_.data() : vector_.data();
        }

        return *this;
    }

    allocator_type get_allocator() const { return vector_.get_allocator(); }

    reference at(size_type pos)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_AT(pos);

        return data_[pos];
    }

    const_reference at(size_type pos) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_AT(pos);

        return data_[pos];
    }

    reference operator[](size_type pos) { return data_[pos]; }

    const_reference operator[](size_type pos) const { return data_[pos]; }

    reference front() { return data_[0]; }

    const_reference front() const { return data_[0]; }

    reference back() { return data_[size() - 1]; }

    const_reference back() const { return data_[size() - 1]; }

    pointer data() { return data_; }

    const_pointer data() const { return data_; }

    iterator begin() { return data_; }

    const_iterator begin() const { return data_; }

    const_iterator cbegin() { return data_; }

    const_iterator cbegin() const { return data_; }

    iterator end() { return data_ + size(); }

    const_iterator end() const { return data_ + size(); }

    const_iterator cend() { return data_ + size(); }

    const_iterator cend() const { return data_ + size(); }

    reverse_iterator rbegin() { return reverse_iterator(end()); }

    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator crbegin() { return const_reverse_iterator(cend()); }

    const_reverse_iterator crbegin() const
    {
        return const_reverse_iterator(cend());
    }

    reverse_iterator rend() { return reverse_iterator(begin()); }

    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    const_reverse_iterator crend() { return const_reverse_iterator(cbegin()); }

    const_reverse_iterator crend() const
    {
        return const_reverse_iterator(cbegin());
    }

    bool empty() const { return array_.empty() && vector_.size() == 0; }

    size_type size() const { return vector_.empty() ? N : vector_.size(); }

    size_type max_size() const { return N; }

    size_type capacity() const { return N; }

    void swap(StaticVectorN<T, N, Alloc> &other)
    {
        array_.swap(other.array_);
        vector_.swap(other.vector_);
        std::swap(data_, other.data_);
    }

    private:
    Array<T, N, internal::AllocatorAlignment<Alloc>::value> array_;
    Vector<T, Alloc> vector_;
    pointer data_;
}; // class StaticVectorN

/// \brief Swap two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline void swap(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    sv1.swap(sv2);
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator==(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    if (sv1.size() != sv2.size())
        return false;
    for (std::size_t i = 0; i != sv1.size(); ++i)
        if (sv1[i] != sv2[i])
            return false;
    return true;
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator!=(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    return !(sv1 == sv2);
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator<(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    return std::lexicographical_compare(
        sv1.begin(), sv1.end(), sv2.begin(), sv2.end());
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator<=(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    return sv1 < sv2 || sv1 == sv2;
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator>(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    return !(sv1 <= sv2);
}

/// \brief Compare two StaticVectorN objects
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc>
inline bool operator>=(const StaticVectorN<T, N, Alloc> &sv1,
    const StaticVectorN<T, N, Alloc> &sv2)
{
    return !(sv1 < sv2);
}

/// \brief Output an StaticVectorN objects
/// \ingroup StaticVector
template <typename CharT, typename Traits, typename T, std::size_t N,
    typename Alloc>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os,
    const StaticVectorN<T, N, Alloc> &sv)
{
    if (!os)
        return os;

    os << sv.size();
    if (!os)
        return os;

    for (const auto &v : sv)
        os << ' ' << v;

    return os;
}

/// \brief Input an StaticVectorN objects
/// \ingroup StaticVector
template <typename CharT, typename Traits, typename T, std::size_t N,
    typename Alloc>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, StaticVectorN<T, N, Alloc> &sv)
{
    if (!is)
        return is;

    std::size_t n = 0;
    is >> n;
    if (!is)
        return is;

    StaticVectorN<T, N, Alloc> tmp(n);
    for (std::size_t i = 0; i != n; ++i)
        is >> std::ws >> tmp[i];
    if (static_cast<bool>(is))
        sv = std::move(tmp);

    return is;
}

/// \brief Alias to StaticVectorN if N > 0 other wise Vector
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Alloc = Allocator<T>>
using StaticVector = typename std::conditional<N == Dynamic, Vector<T, Alloc>,
    StaticVectorN<T, N, Alloc>>::type;

} // namespace vsmc

#endif // VSMC_UTILITY_STATIC_VECTOR_HPP
