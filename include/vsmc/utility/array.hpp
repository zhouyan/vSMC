//============================================================================
// include/vsmc/utility/array.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_ARRAY_HPP
#define VSMC_UTILITY_ARRAY_HPP

#include <vsmc/internal/common.hpp>
#include <cstring>
#include <iomanip>
#include <iostream>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4351)
#endif

#define VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N) \
    VSMC_STATIC_ASSERT((Pos < N), USE_Array_WITH_AN_INDEX_OUT_OF_RANGE)

#define VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N) \
    VSMC_RUNTIME_ASSERT((i < N), ("**Array** USED WITH AN INDEX OUT OF RANGE"))

namespace vsmc {

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
/// assertions of the index instead of runtime assertion. They can also be used
/// to write loop unrolling functions for the container. They are preferred way
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
/// defined for Array. Same reason as above. Instead, use `vsmc::TupleSize` and
/// `vsmc::TupleElement`.
///
/// It also provides a few other member functions not presented in
/// `std::array`, see the documents.
template <typename T, std::size_t N>
class Array
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

    reference at (size_type i)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N);
        return data_[i];
    }

    const_reference at (size_type i) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N);
        return data_[i];
    }

    reference operator[] (size_type i) {return data_[i];}

    const_reference operator[] (size_type i) const {return data_[i];}

    reference front () {return at<0>();}

    const_reference front () const {return at<0>();}

    reference back () {return at<N - 1>();}

    const_reference back () const {return at<N - 1>();}

    pointer data () {return data_;}

    const_pointer data () const {return data_;}

    iterator begin () {return data_;}

    iterator end () {return data_ + N;}

    const_iterator begin () const {return data_;}

    const_iterator end () const {return data_ + N;}

    const_iterator cbegin () const {return data_;}

    const_iterator cend () const {return data_ + N;}

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

    static VSMC_CONSTEXPR bool empty () {return N == 0;}

    static VSMC_CONSTEXPR size_type size () {return N;}

    static VSMC_CONSTEXPR size_type max_size ()
    {return static_cast<size_type>(~static_cast<size_type>(0)) / sizeof(T);}

    void fill (const T &value) {fill(value, cxx11::is_integral<T>());}

    void swap (Array<T, N> &other)
    {
        using std::swap;

        for (size_type i = 0; i != N; ++i)
            swap(data_[i], other.data_[i]);
    }

    friend inline bool operator== (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {
        for (size_type i = 0; i != N; ++i)
            if (sv1[i] != sv2[i])
                return false;

        return true;
    }

    friend inline bool operator!= (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {return !(sv1 == sv2);}

    friend inline bool operator< (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {
        for (size_type i = 0; i != N; ++i) {
            if (sv1[i] < sv2[i])
                return true;
            if (sv2[i] < sv1[i])
                return false;
        }

        return false;
    }

    friend inline bool operator<= (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {
        for (size_type i = 0; i != N; ++i) {
            if (sv1[i] < sv2[i])
                return true;
            if (sv2[i] < sv1[i])
                return false;
        }

        return true;
    }

    friend inline bool operator> (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {return !(sv1 <= sv2);}

    friend inline bool operator>= (
            const Array<T, N> &sv1, const Array<T, N> &sv2)
    {return !(sv1 < sv2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os, const Array<T, N> &sv)
    {
        if (!os.good())
            return os;

        for (size_type i = 0; i < N - 1; ++i)
            os << sv[i] << ' ';
        os << sv[N - 1];

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is, Array<T, N> &sv)
    {
        if (!is.good())
            return is;

        Array<T, N> sv_tmp;
        for (size_type i = 0; i != N; ++i)
            is >> std::ws >> sv_tmp[i];

        if (is.good()) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            sv = cxx11::move(sv_tmp);
#else
            sv = sv_tmp;
#endif
        }
    }

    template <size_type Pos>
    reference at ()
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos>
    const_reference at () const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos>
    reference at (Position<Pos>)
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos>
    const_reference at (Position<Pos>) const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return data_[Pos];
    }

    template <size_type Pos>
    reference operator[] (Position<Pos>) {return at<Pos>();}

    template <size_type Pos>
    const_reference operator[] (Position<Pos>) const {return at<Pos>();}

    private :

    value_type data_[N];

    void fill (const T &value, cxx11::true_type)
    {
        if (value == 0)
            std::memset(data_, 0, sizeof(T) * N);
        else
            fill(value, cxx11::false_type());
    }

    void fill (const T &value, cxx11::false_type)
    {for (size_type i = 0; i != N; ++i) data_[i] = value;}
}; // class Array

/// \brief Array ADL of swap
/// \ingroup Array
template <typename T, std::size_t N>
inline void swap (Array<T, N> &sv1, Array<T, N> &sv2) {sv1.swap(sv2);}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline T &get (Array<T, N> &sv) {return sv.template at<I>();}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline const T &get (const Array<T, N> &sv) {return sv.template at<I>();}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
inline T &&get (Array<T, N> &&sv) {return cxx11::move(sv.template at<I>());}
#endif

template <typename> struct TupleSize;

/// \brief The size of Array
/// \ingroup Array
template <typename T, std::size_t N>
struct TupleSize<Array<T, N> > :
public cxx11::integral_constant<std::size_t, N> {};

template <std::size_t, typename> struct TupleElement;

/// \brief The type of Array
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N>
struct TupleElement<I, Array<T, N> > {typedef T type;};

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_UTILITY_ARRAY_HPP
