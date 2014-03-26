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
    VSMC_STATIC_ASSERT((Pos < N),                                            \
            USE_Array_WITH_AN_INDEX_OUT_OF_RANGE)

#define VSMC_STATIC_ASSERT_UTILITY_ARRAY_SLICE_BEGIN(Begin, N) \
    VSMC_STATIC_ASSERT((Begin < N),                                          \
            USE_Array_SLICE_WITH_FIRST_INDEX_TOO_LARGE)

#define VSMC_STATIC_ASSERT_UTILITY_ARRAY_SLICE_END(End, N) \
    VSMC_STATIC_ASSERT((End < N),                                            \
            USE_Array_WITH_SLICE_WITH_SECOND_INDEX_TOO_LARGE)

#define VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N) \
    VSMC_RUNTIME_ASSERT((i < N), ("**Array** USED WITH AN INDEX OUT OF RANGE"))

namespace vsmc {

namespace internal {

template <typename, std::size_t, bool> class ArrayStorage;

template <typename T, std::size_t N>
class ArrayStorage<T, N, true>
{
    public :

    T &operator[] (std::size_t i) {return data_[i];}

    const T &operator[] (std::size_t i) const {return data_[i];}

    T *data () {return data_;}

    const T *data () const {return data_;}

    void swap_data (ArrayStorage<T, N, true> &other)
    {
        using std::swap;

        for (std::size_t i = 0; i != N; ++i)
            swap(data_[i], other.data_[i]);
    }

    private :

    T data_[N];
}; // class ArrayStorage

template <typename T, std::size_t N>
class ArrayStorage<T, N, false>
{
    public :

    ArrayStorage () : data_(new T[N]()) {}

    ArrayStorage (const ArrayStorage<T, N, false> &other) :
        data_(new T[N]())
    {
        for (std::size_t i = 0; i != N; ++i)
            data_[i] = other.data_[i];
    }

    ArrayStorage<T, N, false> &operator= (
            const ArrayStorage<T, N, false> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != N; ++i)
                data_[i] = other.data_[i];
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    ArrayStorage (ArrayStorage<T, N, false> &&other) :
        data_(other.data_) {other.data_ = VSMC_NULLPTR;}

    ArrayStorage<T, N, false> &operator= (
            ArrayStorage<T, N, false> &&other)
    {
        if (this != &other) {
            if (data_ != VSMC_NULLPTR)
                delete [] data_;
            data_ = other.data_;
            other.data_ = VSMC_NULLPTR;
        }

        return *this;
    }
#endif

    ~ArrayStorage ()
    {
        if (data_ != VSMC_NULLPTR)
            delete [] data_;
    }

    T &operator[] (std::size_t i) {return data_[i];}

    const T &operator[] (std::size_t i) const {return data_[i];}

    T *data () {return data_;}

    const T *data () const {return data_;}

    void swap_data (ArrayStorage<T, N, false> &other)
    {std::swap(data_, other.data_);}

    private :

    T *data_;
}; // class ArrayStorage

} // namespace vsmc::internal

namespace traits {

/// \brief Default trait of Array
/// \ingroup Traits
template <typename T> struct ArrayTrait
{
    static VSMC_CONSTEXPR const std::size_t max_static_size =
        static_cast<std::size_t>(~(static_cast<std::size_t>(0)));
}; // struct ArrayTrait

} // namespace vsmc::traits

/// \brief Container class with static size but possible dynamic memory
/// allocation
/// \ingroup Array
///
/// \details
/// Array and C++11 `std::array` are efficient when the size is known at
/// compile time. However, when the size is large, the benefits can be
/// negligible. And in the worst case, it can even cause stack overflow or
/// degenerate the performance. There are other undesired effects of
/// `std::array`. For example, the `swap` operation has a complexity linear to
/// the size while with `std::vector`, it has a constant complexity.
///
/// The Array container is sort of a hybrid of `std::array` and `std::vector`.
/// The default behavior is almost identical to `std::array` with a few minor
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
/// library does not pollute the `std` namespace in anyway.
/// - The helper classes `std::tuple_size` and `std::tuple_element` are not
/// defined for Array. Same reason as above. Instead, use `vsmc::ArraySize` and
/// `vsmc::ArrayElement`.
///
/// It also provides a few other member functions not presented in
/// `std::array`, see the documents.
///
/// The class has an additional template parameter, `Traits`. The class check
/// the static constant member data `max_static_size`, which by default is the
/// maximum of `std::size_t`. If the size of the array, `sizeof(T) * N`, is
/// smaller than or equal to `max_static_size`, then the memory is allocated
/// with array. Otherwise, it is allocated on the heap. Though the memory is
/// allocated dynamically, the size still has to be known at compile time. In
/// addition, the statical bounds checking can still be used.
template <typename T, std::size_t N,
         typename Traits = traits::ArrayTrait<T> >
class Array : public internal::ArrayStorage<T, N,
    (sizeof(T) * N <= Traits::max_static_size)>
{
    typedef internal::ArrayStorage<T, N,
    (sizeof(T) * N <= Traits::max_static_size)> storage_type;

    public :

    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    typedef value_type & reference;
    typedef const value_type & const_reference;
    typedef T * pointer;
    typedef const T * const_pointer;

    typedef pointer iterator;
    typedef const_pointer const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    iterator begin () {return this->data();}

    iterator end () {return this->data() + N;}

    const_iterator begin () const {return this->data();}

    const_iterator end () const {return this->data() + N;}

    const_iterator cbegin () const {return this->data();}

    const_iterator cend () const {return this->data() + N;}

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

    reference at (size_type i)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N);
        return storage_type::operator[](i);
    }

    const_reference at (size_type i) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_ARRAY_RANGE(i, N);
        return storage_type::operator[](i);
    }

    template <size_type Pos>
    reference at ()
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    const_reference at () const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    reference at (Position<Pos>)
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    const_reference at (Position<Pos>) const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    reference operator[] (size_type i)
    {return storage_type::operator[](i);}

    const_reference operator[] (size_type i) const
    {return storage_type::operator[](i);}

    template <size_type Pos>
    reference operator[] (Position<Pos>) {return at<Pos>();}

    template <size_type Pos>
    const_reference operator[] (Position<Pos>) const {return at<Pos>();}

    reference front () {return at<0>();}

    const_reference front () const {return at<0>();}

    reference back () {return at<N - 1>();}

    const_reference back () const {return at<N - 1>();}

    static VSMC_CONSTEXPR bool empty () {return N == 0;}

    static VSMC_CONSTEXPR size_type size () {return N;}

    static VSMC_CONSTEXPR size_type max_size ()
    {return static_cast<size_type>(~static_cast<size_type>(0)) / sizeof(T);}

    void fill (const T &value) {fill(value, cxx11::is_unsigned<T>());}

    void swap (Array<T, N, Traits> &other) {this->swap_data(other);}

    /// \brief Get a new vector with a different size
    template <std::size_t Size>
    Array<T, Size, Traits> resize () const
    {return resize<Size>(cxx11::integral_constant<bool, Size == N>());}

    /// \brief Get a slice of the vector, [Begin, End)
    template <std::size_t Begin, std::size_t End>
    Array<T, End - Begin, Traits> slice () const
    {
        return slice<Begin, End>(
                cxx11::integral_constant<bool, Begin == 0 && End == N>());
    }

    /// \brief Expand the vector, starting with Begin and size N
    template <std::size_t Begin, std::size_t Size>
    Array<T, Size, Traits> expand () const
    {return expand<Begin, Size>(cxx11::integral_constant<bool, Begin == 0>());}

    friend inline bool operator== (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {
        for (std::size_t i = 0; i != N; ++i)
            if (sv1[i] != sv2[i])
                return false;

        return true;
    }

    friend inline bool operator!= (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {return !(sv1 == sv2);}

    friend inline bool operator< (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {
        for (std::size_t i = 0; i != N; ++i) {
            if (sv1[i] < sv2[i])
                return true;
            if (sv2[i] < sv1[i])
                return false;
        }

        return false;
    }

    friend inline bool operator<= (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {
        for (std::size_t i = 0; i != N; ++i) {
            if (sv1[i] < sv2[i])
                return true;
            if (sv2[i] < sv1[i])
                return false;
        }

        return true;
    }

    friend inline bool operator> (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {return !(sv1 <= sv2);}

    friend inline bool operator>= (
            const Array<T, N, Traits> &sv1,
            const Array<T, N, Traits> &sv2)
    {return !(sv1 < sv2);}

    template <typename CharT, typename CharTraits>
    friend inline std::basic_ostream<CharT, CharTraits> &operator<< (
            std::basic_ostream<CharT, CharTraits> &os,
            const Array<T, N, Traits> &sv)
    {
        if (!os.good())
            return os;

        for (std::size_t i = 0; i < N - 1; ++i)
            os << sv[i] << ' ';
        os << sv[N - 1];

        return os;
    }

    template <typename CharT, typename CharTraits>
    friend inline std::basic_istream<CharT, CharTraits> &operator>> (
            std::basic_istream<CharT, CharTraits> &is,
            Array<T, N, Traits> &sv)
    {
        if (!is.good())
            return is;

        Array<T, N, Traits> sv_tmp;
        for (std::size_t i = 0; i != N; ++i)
            is >> std::ws >> sv_tmp[i];

        if (is.good()) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            sv = cxx11::move(sv_tmp);
#else
            sv = sv_tmp;
#endif
        }
    }

    private :

    template <std::size_t Size>
    Array<T, Size, Traits> resize (cxx11::true_type) const
    {return *this;}

    template <std::size_t Size>
    Array<T, Size, Traits> resize (cxx11::false_type) const
    {
        Array<T, Size, Traits> res;
        for (std::size_t i = 0, j = 0; i != N && j != Size; ++i, ++j)
            res[j] = storage_type::operator[](i);

        return res;
    }

    template <std::size_t Begin, std::size_t End>
    Array<T, End - Begin, Traits> slice (cxx11::true_type) const
    {return *this;}

    template <std::size_t Begin, std::size_t End>
    Array<T, End - Begin, Traits> slice (cxx11::false_type) const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_SLICE_BEGIN(Begin, N);
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_SLICE_END(End, N);
        Array<T, End - Begin, Traits> res;
        for (std::size_t i = Begin, j = 0; i != End; ++i, ++j)
            res[j] = storage_type::operator[](i);

        return res;
    }

    template <std::size_t Begin, std::size_t Size>
    Array<T, Size, Traits> expand (cxx11::true_type) const
    {return resize<Size>();}

    template <std::size_t Begin, std::size_t Size>
    Array<T, Size, Traits> expand (cxx11::false_type) const
    {
        VSMC_STATIC_ASSERT_UTILITY_ARRAY_SLICE_BEGIN(Begin, N);
        Array<T, Size, Traits> res;
        for (std::size_t i = Begin, j = 0; i != N && j != Size; ++i, ++j)
            res[j] = storage_type::operator[](i);

        return res;
    }

    void fill (const T &value, cxx11::true_type)
    {
        if (value == 0)
            std::memset(this->data(), 0, sizeof(T) * N);
        else
            fill(value, cxx11::false_type());
    }

    void fill (const T &value, cxx11::false_type)
    {
        for (size_type i = 0; i != N; ++i)
            storage_type::operator[](i) = value;
    }
}; // class Array

/// \brief Array ADL of swap
/// \ingroup Array
template <typename T, std::size_t N, typename Traits>
inline void swap (
        Array<T, N, Traits> &sv1,
        Array<T, N, Traits> &sv2)
{sv1.swap(sv2);}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline T &get (Array<T, N, Traits> &sv)
{return sv.template at<I>();}

/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline const T &get (const Array<T, N, Traits> &sv)
{return sv.template at<I>();}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Array ADL of get
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline T &&get (Array<T, N, Traits> &&sv)
{return cxx11::move(sv.template at<I>());}
#endif

template <typename> struct ArraySize;

/// \brief The size of Array
/// \ingroup Array
template <typename T, std::size_t N, typename Traits>
struct ArraySize<Array<T, N, Traits> > :
public cxx11::integral_constant<std::size_t, N> {};

template <std::size_t, typename> struct ArrayElement;

/// \brief The type of Array
/// \ingroup Array
template <std::size_t I, typename T, std::size_t N, typename Traits>
struct ArrayElement<I, Array<T, N, Traits> > {typedef T type;};

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_UTILITY_ARRAY_HPP
