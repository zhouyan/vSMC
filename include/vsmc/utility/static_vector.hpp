#ifndef VSMC_UTILITY_STATIC_VECTOR_HPP
#define VSMC_UTILITY_STATIC_VECTOR_HPP

#include <vsmc/internal/common.hpp>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4351)
#endif

#define VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N) \
    VSMC_STATIC_ASSERT((Pos < N),                                            \
            USE_StaticVector_WITH_AN_INDEX_OUT_OF_RANGE)

#define VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_SLICE_BEGIN(Begin, N) \
    VSMC_STATIC_ASSERT((Begin < N),                                          \
            USE_StaticVector_SLICE_WITH_FIRST_INDEX_TOO_LARGE)

#define VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_SLICE_END(End, N) \
    VSMC_STATIC_ASSERT((End < N),                                            \
            USE_StaticVector_WITH_SLICE_WITH_SECOND_INDEX_TOO_LARGE)

#define VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_RANGE(i, N) \
    VSMC_RUNTIME_ASSERT((i < N),                                             \
            ("**StaticVector** USED WITH AN INDEX OUT OF RANGE"))

namespace vsmc {

namespace internal {

template <typename, std::size_t, bool> class StaticVectorStorage;

template <typename T, std::size_t N>
class StaticVectorStorage<T, N, true>
{
    public :

    T &operator[] (std::size_t i) {return data_[i];}

    const T &operator[] (std::size_t i) const {return data_[i];}

    protected :

    StaticVectorStorage () : data_() {}

    StaticVectorStorage (const StaticVectorStorage<T, N, true> &other) :
        data_()
    {
        for (std::size_t i = 0; i != N; ++i)
            data_[i] = other.data_[i];
    }

    StaticVectorStorage<T, N, true> &operator= (
            const StaticVectorStorage<T, N, true> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != N; ++i)
                data_[i] = other.data_[i];
        }

        return *this;
    }

    T *ptr () {return data_;}

    const T *ptr () const {return data_;}

    void swap_data (StaticVectorStorage<T, N, true> &other)
    {
        using std::swap;

        for (std::size_t i = 0; i != N; ++i)
            swap(data_[i], other.data_[i]);
    }

    private :

    T data_[N];
}; // class StaticVectorStorage

template <typename T, std::size_t N>
class StaticVectorStorage<T, N, false>
{
    public :

    T &operator[] (std::size_t i) {return data_[i];}

    const T &operator[] (std::size_t i) const {return data_[i];}

    protected :

    StaticVectorStorage () : data_(new T[N]()) {}

    StaticVectorStorage (const StaticVectorStorage<T, N, false> &other) :
        data_(new T[N]())
    {
        for (std::size_t i = 0; i != N; ++i)
            data_[i] = other.data_[i];
    }

    StaticVectorStorage<T, N, false> &operator= (
            const StaticVectorStorage<T, N, false> &other)
    {
        if (this != &other) {
            for (std::size_t i = 0; i != N; ++i)
                data_[i] = other.data_[i];
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    StaticVectorStorage (StaticVectorStorage<T, N, false> &&other) :
        data_(other.data_) {other.data_ = VSMC_NULLPTR;}

    StaticVectorStorage<T, N, false> &operator= (
            StaticVectorStorage<T, N, false> &&other)
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

    ~StaticVectorStorage ()
    {
        if (data_ != VSMC_NULLPTR)
            delete [] data_;
    }

    T *ptr () {return data_;}

    const T *ptr () const {return data_;}

    void swap_data (StaticVectorStorage<T, N, false> &other)
    {std::swap(data_, other.data_);}

    private :

    T *data_;
}; // class StaticVectorStorage

} // namespace vsmc::internal

namespace traits {

/// \brief Default traits of StaticVector
/// \ingroup Traits
template <typename T> struct StaticVectorTrait
{static VSMC_CONSTEXPR const std::size_t max_static_size = 1024;};

} // namespace vsmc::traits

/// \brief StaticVector iterator
/// \ingroup StaticVector
template <typename T>
class StaticVectorIterator :
    public std::iterator<std::random_access_iterator_tag, T>
{
    typedef std::iterator<std::random_access_iterator_tag, T>
        base_iterator_type;

    public :

    typedef typename base_iterator_type::value_type value_type;
    typedef typename base_iterator_type::difference_type difference_type;
    typedef typename base_iterator_type::pointer pointer;
    typedef typename base_iterator_type::reference reference;
    typedef std::random_access_iterator_tag iterator_category;

    StaticVectorIterator () : ptr_(VSMC_NULLPTR) {}

    StaticVectorIterator (pointer ptr) : ptr_(ptr) {}

    reference operator* () const {return *ptr_;}

    pointer operator-> () const {return ptr_;}

    value_type operator[] (difference_type diff) const {return *(ptr_ + diff);}

    StaticVectorIterator<T> &operator++ () {++ptr_; return *this;}

    StaticVectorIterator<T> operator++ (int) const
    {StaticVectorIterator<T> iter(*this); return ++iter;}

    StaticVectorIterator<T> &operator-- () {--ptr_; return *this;}

    StaticVectorIterator<T> operator-- (int) const
    {StaticVectorIterator<T> iter(*this); return --iter;}

    StaticVectorIterator<T> &operator+= (difference_type diff)
    {ptr_ += diff; return *this;}

    StaticVectorIterator<T> &operator-= (difference_type diff)
    {ptr_ -= diff; return *this;}

    friend inline bool operator== (
            const StaticVectorIterator<T> &iter1,
            const StaticVectorIterator<T> &iter2)
    {return iter1.ptr_ == iter2.ptr_;}

    friend inline bool operator!= (
            const StaticVectorIterator<T> &iter1,
            const StaticVectorIterator<T> &iter2)
    {return iter1.ptr_ != iter2.ptr_;}

    friend inline difference_type operator- (
            const StaticVectorIterator<T> &iter1,
            const StaticVectorIterator<T> &iter2)
    {return iter1.ptr_ - iter2.ptr_;}

    friend inline StaticVectorIterator<T> operator+ (
            const StaticVectorIterator<T> &iter,
            difference_type diff)
    {return StaticVectorIterator<T>(iter.ptr_ + diff);}

    friend inline StaticVectorIterator<T> operator+ (
            difference_type diff,
            const StaticVectorIterator<T> &iter)
    {return StaticVectorIterator<T>(iter.ptr_ + diff);}

    friend inline StaticVectorIterator<T> operator- (
            const StaticVectorIterator<T> &iter,
            difference_type diff)
    {return StaticVectorIterator<T>(iter.ptr_ - diff);}

    private :

    pointer ptr_;
}; // class StaticVectorIterator

/// \brief Container class with static size but possible dynamic memory
/// allocation
/// \ingroup StaticVector
///
/// \details
/// Array and C++11 `std::array` are efficient when the size is known at
/// compile time. However, whent the size is large, the benefits can be
/// negligible. And in the worst case, it can even cause stack overflow or
/// degenerate the performance. There are other undesired effects of
/// `std::array`. For example, the `swap` operation has a complexity linear to
/// the size while with `std::vector`, it has a constant complexity.
///
/// The StaticVector container is sort of a hybrid of `std::array` and
/// `std::vector`.  When the size is small, it uses array to allocate the
/// memory and therefore if the object is allocated on the stack, then no
/// dynamic memory allocation happens at all. And if a sequence of StaticVector
/// is allocated, the internal storage is allocated next to each other. When
/// the size is large, it uses dynamic memory allocation and it still behaves
/// much like a `std::array` except one does not need to worry that very large
/// stack allocation will happen.
///
/// The interface is almost identical to that of `std::array` with a few
/// differences.
/// - There is no `max_size()` member function
/// - The `size()` and `empty()` member functions are `static` and can be used
/// as constant expression if `constexpr` is supported
/// - There are additional `at<Pos>()` and `at(Position<Pos>())` member
/// functions, which perform static assertions of the index instead of runtime
/// assertion. They can also be used to write loop unrolling functions for the
/// container. They are preferred way to access elements. They are (slightly)
/// more efficient than `operator[]` and also safer.
/// - The non-member function `get` and `swap` are defined in the namespace
/// `vsmc`. Use the swap idiom,
/// ~~~{.cpp}
/// using std::swap;
/// swap(obj1, obj2);
/// ~~~
/// instead of calling `std::swap(obj1, obj2)`. See "Effective C++".
/// - The helper classes `std::tuple_size` and `std::tuple_element` are not
/// defined for StaticVector
template <typename T, std::size_t N,
         typename Traits = traits::StaticVectorTrait<T> >
class StaticVector : public internal::StaticVectorStorage<T, N,
    (sizeof(T) * N <= Traits::max_static_size)>
{
    typedef internal::StaticVectorStorage<T, N,
    (sizeof(T) * N <= Traits::max_static_size)> storage_type;

    public :

    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    typedef value_type & reference;
    typedef const value_type & const_reference;
    typedef T * pointer;
    typedef const T * const_pointer;

    typedef StaticVectorIterator<T> iterator;
    typedef StaticVectorIterator<const T> const_iterator;
    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

    iterator begin () {return iterator(this->ptr());}

    iterator end () {return iterator(this->ptr() + N);}

    const_iterator begin () const {return cosnt_iterator(this->ptr());}

    const_iterator end () const {return const_iterator(this->ptr() + N);}

    const_iterator cbegin () const {return const_iterator(this->ptr());}

    const_iterator cend () const {return const_iterator(this->ptr() + N);}

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
        VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_RANGE(i, N);
        return storage_type::operator[](i);
    }

    const_reference at (size_type i) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_RANGE(i, N);
        return storage_type::operator[](i);
    }

    template <size_type Pos>
    reference at ()
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    const_reference at () const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    reference at (Position<Pos>)
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return storage_type::operator[](Pos);
    }

    template <size_type Pos>
    const_reference at (Position<Pos>) const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
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

    pointer data () {return this->ptr();}

    const_pointer data () const {return this->ptr();}

    static VSMC_CONSTEXPR bool empty () {return N == 0;}

    static VSMC_CONSTEXPR size_type size () {return N;}

    void fill (const T &value)
    {
        for (size_type i = 0; i != N; ++i)
            this->operator[](i) = value;
    }

    void swap (StaticVector<T, N, Traits> &other) {this->swap_data(other);}

    /// \brief Get a new vector with a different size
    template <std::size_t Size>
    StaticVector<T, Size, Traits> resize () const
    {return resize<Size>(cxx11::integral_constant<bool, Size == N>());}

    /// \brief Get a slice of the vector, [Begin, End)
    template <std::size_t Begin, std::size_t End>
    StaticVector<T, End - Begin, Traits> slice () const
    {
        return slice<Begin, End>(
                cxx11::integral_constant<bool, Begin == 0 && End == N>());
    }

    /// \brief Expand the vector, starting with Begin and size N
    template <std::size_t Begin, std::size_t Size>
    StaticVector<T, Size, Traits> expand () const
    {return expand<Begin, Size>(cxx11::integral_constant<bool, Begin == 0>());}

    private :

    template <std::size_t Size>
    StaticVector<T, Size, Traits> resize (cxx11::true_type) const
    {return *this;}

    template <std::size_t Size>
    StaticVector<T, Size, Traits> resize (cxx11::false_type) const
    {
        StaticVector<T, Size, Traits> res;
        for (std::size_t i = 0, j = 0; i != N && j != Size; ++i, ++j)
            res[j] = this->operator[](i);

        return res;
    }

    template <std::size_t Begin, std::size_t End>
    StaticVector<T, End - Begin, Traits> slice (cxx11::true_type) const
    {return *this;}

    template <std::size_t Begin, std::size_t End>
    StaticVector<T, End - Begin, Traits> slice (cxx11::false_type) const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_SLICE_BEGIN(Begin, N);
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_SLICE_END(End, N);
        StaticVector<T, End - Begin, Traits> res;
        for (std::size_t i = Begin, j = 0; i != End; ++i, ++j)
            res[j] = this->operator[](i);

        return res;
    }

    template <std::size_t Begin, std::size_t Size>
    StaticVector<T, Size, Traits> expand (cxx11::true_type) const
    {return resize<Size>();}

    template <std::size_t Begin, std::size_t Size>
    StaticVector<T, Size, Traits> expand (cxx11::false_type) const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_SLICE_BEGIN(Begin, N);
        StaticVector<T, Size, Traits> res;
        for (std::size_t i = Begin, j = 0; i != N && j != Size; ++i, ++j)
            res[j] = this->operator[](i);

        return res;
    }
}; // class StaticVector

/// \brief StaticVector ADL of swap
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline void swap (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{sv1.swap(sv2);}

/// \brief StaticVector ADL of get
/// \ingroup StaticVector
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline T &get (StaticVector<T, N, Traits> &sv)
{return sv.template at<I>();}

/// \brief StaticVector ADL of get
/// \ingroup StaticVector
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline const T &get (const StaticVector<T, N, Traits> &sv)
{return sv.template at<I>();}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief StaticVector ADL of get
/// \ingroup StaticVector
template <std::size_t I, typename T, std::size_t N, typename Traits>
inline T &&get (StaticVector<T, N, Traits> &&sv)
{return cxx11::move(sv.template at<I>());}
#endif

/// \brief StaticVector operator==
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator== (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{
    for (std::size_t i = 0; i != N; ++i)
        if (sv1[i] != sv2[i])
            return false;

    return true;
}

/// \brief StaticVector operator!=
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator!= (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 == sv2);}

/// \brief StaticVector operator<
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator< (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{
    for (std::size_t i = 0; i != N; ++i) {
        if (sv1[i] < sv2[i])
            return true;
        if (sv2[i] < sv1[i])
            return false;
    }

    return false;
}

/// \brief StaticVector operator<=
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator<= (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{
    for (std::size_t i = 0; i != N; ++i) {
        if (sv1[i] < sv2[i])
            return true;
        if (sv2[i] < sv1[i])
            return false;
    }

    return true;
}

/// \brief StaticVector operator>
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator> (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 <= sv2);}

/// \brief StaticVector operator>=
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits>
inline bool operator>= (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 < sv2);}

/// \brief StaticVector operator<<
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits,
         typename CharT, typename CharTraits>
inline std::basic_ostream<CharT, CharTraits> &operator<< (
        std::basic_ostream<CharT, CharTraits> &os,
        const StaticVector<T, N, Traits> &sv)
{
    if (!os)
        return os;

    for (std::size_t i = 0; i < N - 1; ++i)
        if (os) os << sv[i] << ' ';
    if (os) os << sv[N - 1];

    return os;
}

/// \brief StaticVector operator<<
/// \ingroup StaticVector
template <typename T, std::size_t N, typename Traits,
         typename CharT, typename CharTraits>
inline std::basic_istream<CharT, CharTraits> &operator>> (
        std::basic_istream<CharT, CharTraits> &is,
        const StaticVector<T, N, Traits> &sv)
{
    if (!is)
        return is;

    StaticVector<T, N, Traits> tmp;
    for (std::size_t i = 0; i != N; ++i) {
        if (!is) return is;
        is >> std::ws >> tmp[i];
    }
    if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
        sv = cxx11::move(tmp);
#else
        sv = tmp;
#endif
    }
}

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_UTILITY_STATIC_VECTOR_HPP
