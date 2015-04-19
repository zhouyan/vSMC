//============================================================================
// vSMC/include/vsmc/utility/array.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_UTILITY_ARRAY_HPP
#define VSMC_UTILITY_ARRAY_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MSVC
#pragma warning(push)
#pragma warning(disable : 4351)
#endif

#define VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N)                       \
    VSMC_STATIC_ASSERT((Pos < N), USE_Array_WITH_AN_INDEX_OUT_OF_RANGE)

#define VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N)                        \
    VSMC_RUNTIME_ASSERT((i < N),                                             \
                        ("**Array** USED WITH AN INDEX OUT OF RANGE"))

namespace vsmc
{

/// \brief Static array
/// \ingroup Array
///
/// \details
/// The `Array` container is almost identical to `std::array` with a few minor
/// differences.
/// - The `size()`, `max_size()` and `empty()` member functions are `static`
/// and can be used as constant expression if C++11 `constexpr` is supported
/// - There are additional `at<Pos>()`, `at(Position<Pos>)` and
/// `operator[](Position<Pos>)` member functions, which perform static
/// assertions of the index instead of runtime assertion. They can also be
/// used
/// to write loop unrolling functions for the container. They are preferred
/// way
/// to access elements since they provide bounds checking without performance
/// penalty.
/// - The non-member function `get` and `swap` are defined in the namespace
/// `vsmc`. Use the swap idiom,
/// ~~~{.cpp}
/// using std::swap;
/// swap(obj1, obj2);
/// ~~~
/// instead of calling `std::swap(obj1, obj2)`. See "Effective C++". The
/// library does not pollute the `std` namespace in any way.
/// - The helper classes `std::tuple_size` and `std::tuple_element` are not
/// defined for Array. Same reason as above. Instead, use `vsmc::TupleSize`
/// and
/// `vsmc::TupleElement`.
///
/// It also provides a few other member functions not presented in
/// `std::array`, see the documents.
template <typename T, std::size_t N> class Array
{
    public:
    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef value_type &reference;
    typedef const value_type &const_reference;
    typedef value_type *pointer;
    typedef const value_type *const_pointer;
    typedef pointer iterator;
    typedef const_pointer const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    template <typename Archive> void serialize(Archive &ar, const unsigned)
    {
        ar &data_;
    }

    template <typename Archive>
    void serialize(Archive &ar, const unsigned) const
    {
        ar &data_;
    }

    reference at(size_type pos)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(pos, N);

        return data_[pos];
    }

    const_reference at(size_type pos) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(pos, N);

        return data_[pos];
    }

    reference operator[](size_type pos) { return data_[pos]; }

    const_reference operator[](size_type pos) const { return data_[pos]; }

    reference front() { return at<0>(); }

    const_reference front() const { return at<0>(); }

    reference back() { return at<N - 1>(); }

    const_reference back() const { return at<N - 1>(); }

    pointer data() { return data_; }

    const_pointer data() const { return data_; }

    iterator begin() { return data_; }

    iterator end() { return data_ + N; }

    const_iterator begin() const { return data_; }

    const_iterator end() const { return data_ + N; }

    const_iterator cbegin() const { return data_; }

    const_iterator cend() const { return data_ + N; }

    reverse_iterator rbegin() { return reverse_iterator(end()); }

    reverse_iterator rend() { return reverse_iterator(begin()); }

    const_reverse_iterator rbegin() const
    {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator rend() const
    {
        return const_reverse_iterator(begin());
    }

    const_reverse_iterator crbegin() const
    {
        return const_reverse_iterator(cend());
    }

    const_reverse_iterator crend() const
    {
        return const_reverse_iterator(cbegin());
    }

    static VSMC_CONSTEXPR bool empty() { return N == 0; }

    static VSMC_CONSTEXPR size_type size() { return N; }

    static VSMC_CONSTEXPR size_type max_size()
    {
        return static_cast<size_type>(~static_cast<size_type>(0)) / sizeof(T);
    }

    void fill(const T &value)
    {
        fill(value,
             std::integral_constant < bool,
             std::is_integral<T>::value &&N >= 100 > ());
    }

    void swap(Array<T, N> &other)
    {
        using std::swap;

        for (size_type i = 0; i != N; ++i)
            swap(data_[i], other.data_[i]);
    }

    friend inline bool operator==(const Array<T, N> &ary1,
                                  const Array<T, N> &ary2)
    {
        for (size_type i = 0; i != N; ++i)
            if (ary1[i] != ary2[i])
                return false;

        return true;
    }

    friend inline bool operator!=(const Array<T, N> &ary1,
                                  const Array<T, N> &ary2)
    {
        return !(ary1 == ary2);
    }

    friend inline bool operator<(const Array<T, N> &ary1,
                                 const Array<T, N> &ary2)
    {
        for (size_type i = 0; i != N; ++i) {
            if (ary1[i] < ary2[i])
                return true;
            if (ary2[i] < ary1[i])
                return false;
        }

        return false;
    }

    friend inline bool operator>(const Array<T, N> &ary1,
                                 const Array<T, N> &ary2)
    {
        for (size_type i = 0; i != N; ++i) {
            if (ary1[i] > ary2[i])
                return true;
            if (ary2[i] > ary1[i])
                return false;
        }

        return false;
    }

    friend inline bool operator<=(const Array<T, N> &ary1,
                                  const Array<T, N> &ary2)
    {
        return !(ary1 > ary2);
    }

    friend inline bool operator>=(const Array<T, N> &ary1,
                                  const Array<T, N> &ary2)
    {
        return !(ary1 < ary2);
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &
        operator<<(std::basic_ostream<CharT, Traits> &os,
                   const Array<T, N> &ary)
    {
        if (!os.good())
            return os;

        for (size_type i = 0; i < N - 1; ++i)
            os << ary[i] << ' ';
        os << ary[N - 1];

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &
        operator>>(std::basic_istream<CharT, Traits> &is, Array<T, N> &ary)
    {
        if (!is.good())
            return is;

        Array<T, N> ary_tmp;
        for (size_type i = 0; i != N; ++i)
            is >> std::ws >> ary_tmp[i];

        if (is.good()) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            ary = std::move(ary_tmp);
#else
            ary = ary_tmp;
#endif
        }

        return is;
    }

    template <size_type Pos> reference at()
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos> const_reference at() const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos> reference at(Position<Pos>)
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos> const_reference at(Position<Pos>) const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos> reference operator[](Position<Pos>)
    {
        return at<Pos>();
    }

    template <size_type Pos> const_reference operator[](Position<Pos>) const
    {
        return at<Pos>();
    }

    private:
    value_type data_[N];

    void fill(const T &value, std::true_type)
    {
        if (value == 0)
            std::memset(data_, 0, sizeof(T) * N);
        else
            fill(value, std::false_type());
    }

    void fill(const T &value, std::false_type)
    {
        for (size_type i = 0; i != N; ++i)
            data_[i] = value;
    }
};  // class Array

/// \brief Array ADL of swap
/// \ingroup Array
template <typename T, std::size_t N>
inline void swap(Array<T, N> &ary1, Array<T, N> &ary2)
{
    ary1.swap(ary2);
}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline T &get(Array<T, N> &ary)
{
    return ary.template at<I>();
}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline const T &get(const Array<T, N> &ary)
{
    return ary.template at<I>();
}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline T &&get(Array<T, N> &&ary)
{
    return std::move(ary.template at<I>());
}
#endif

template <typename> struct TupleSize;

/// \brief The size of Array
/// \ingroup Array
template <typename T, std::size_t N>
struct TupleSize<Array<T, N>>
    : public std::integral_constant<std::size_t, N> {
};

template <std::size_t, typename> struct TupleElement;

/// \brief The type of Array
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
struct TupleElement<I, Array<T, N>> {
    typedef T type;
};

}  // namespace vsmc

#ifdef VSMC_MSVC
#pragma warning(pop)
#endif

#endif  // VSMC_UTILITY_ARRAY_HPP
