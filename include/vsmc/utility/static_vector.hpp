#ifndef VSMC_UTILITY_STATIC_VECTOR_HPP
#define VSMC_UTILITY_STATIC_VECTOR_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_RANGE(i, N) \
    VSMC_RUNTIME_ASSERT((i < N),                                             \
            ("**StaticVector** USED WITH AN INDEX OUT OF RANGE"))

#define VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N) \
    VSMC_STATIC_ASSERT((Pos < N),                                            \
            USE_StaticVector_WITH_AN_INDEX_OUT_OF_RANGE)

namespace vsmc {

namespace internal {

template <typename, std::size_t, bool> class StaticVectorStorage;

template <typename T, std::size_t N>
class StaticVectorStorage<T, N, true>
{
    protected :

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
    protected :

    StaticVectorStorage () : data_(new T[N]) {}

    StaticVectorStorage (const StaticVectorStorage<T, N, false> &other) :
        data_(new T[N])
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

/// \brief A container class with static size but possible dynamic memory
/// allocation
/// \ingroup StaticVector
///
/// \details
/// Array and C++11 `std::array` are efficient for when the size is known at
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
template <typename T, std::size_t N,
         typename Traits = traits::StaticVectorTrait<T> >
class StaticVector : public internal::StaticVectorStorage<T, N,
    sizeof(T) * N <= Traits::max_static_size>
{
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
        return this->ptr()[i];
    }

    const_reference at (size_type i) const
    {
        VSMC_RUNTIME_ASSERT_UTILITY_STATIC_VECTOR_RANGE(i, N);
        return this->ptr()[i];
    }

    template <size_type Pos>
    reference at ()
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return this->ptr()[Pos];
    }

    template <size_type Pos>
    const_reference at () const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return this->ptr()[Pos];
    }

    template <size_type Pos>
    reference at (Position<Pos>)
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return this->ptr()[Pos];
    }

    template <size_type Pos>
    const_reference at (Position<Pos>) const
    {
        VSMC_STATIC_ASSERT_UTILITY_STATIC_VECTOR_RANGE(Pos, N);
        return this->ptr()[Pos];
    }

    reference operator[] (size_type i) {return this->ptr()[i];}

    const_reference operator[] (size_type i) const {return this->ptr()[i];}

    reference front () {return this->ptr()[0];}

    const_reference front () const {return this->ptr()[0];}

    reference back () {return this->ptr()[N - 1];}

    const_reference back () const {return this->ptr()[N - 1];}

    pointer data () {return this->ptr();}

    const_pointer data () const {return this->ptr();}

    static VSMC_CONSTEXPR bool empty () {return N == 0;}

    static VSMC_CONSTEXPR size_type size () {return N;}

    void fill (const T &value)
    {
        T *ptr = data();
        for (size_type i = 0; i != N; ++i)
            ptr[i] = value;
    }

    void swap (StaticVector<T, N, Traits> &other) {this->swap_data(other);}
}; // class StaticVector

template <typename T, std::size_t N, typename Traits>
inline void swap (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{sv1.swap(sv2);}

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

template <typename T, std::size_t N, typename Traits>
inline bool operator!= (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 == sv2);}

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

template <typename T, std::size_t N, typename Traits>
inline bool operator> (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 <= sv2);}

template <typename T, std::size_t N, typename Traits>
inline bool operator>= (
        StaticVector<T, N, Traits> &sv1,
        StaticVector<T, N, Traits> &sv2)
{return !(sv1 < sv2);}

} // namespace vsmc

#endif // VSMC_UTILITY_STATIC_VECTOR_HPP
