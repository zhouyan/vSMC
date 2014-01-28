#ifndef VSMC_SMP_INTERNAL_ITERATOR_HPP
#define VSMC_SMP_INTERNAL_ITERATOR_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_SMP_INTERNAL_ITERATOR_BINARY_OP \
    VSMC_RUNTIME_ASSERT((iter1->inc() == iter2->inc()),                      \
            ("BINARY OPERATION ON TWO **StateMatrixRCIteraotr** WITH"        \
            "TWO DIFFERNT INCREMENT"))

namespace vsmc {

template <typename RandomIter>
class StepRandomIterator :
    public std::iterator<
    typename std::iterator_traits<RandomIter>::iterator_category,
    typename std::iterator_traits<RandomIter>::value_type,
    typename std::iterator_traits<RandomIter>::difference_type,
    typename std::iterator_traits<RandomIter>::pointer,
    typename std::iterator_traits<RandomIter>::reference>
{
    public :

    typedef typename std::iterator_traits<RandomIter>::value_type
        value_type;
    typedef typename std::iterator_traits<RandomIter>::difference_type
        difference_type;
    typedef typename std::iterator_traits<RandomIter>::pointer
        pointer;
    typedef typename std::iterator_traits<RandomIter>::reference
        reference;

    StepRandomIterator (RandomIter iter, std::size_t inc) :
        iter_(iter), inc_(inc) {}

    template <typename OtherRandomIter>
    StepRandomIterator (
            const StepRandomIterator<OtherRandomIter> &other) :
        iter_(other.iterator()), inc_(other.increment()) {}

    template <typename OtherRandomIter>
    StepRandomIterator<RandomIter> &operator= (
            const StepRandomIterator<OtherRandomIter> &other)
    {
        iter_ = other.iterator();
        inc_ = other.increment();

        return *this;
    }

    RandomIter iterator () const {return iter_;}

    std::size_t increment () const {return inc_;}

    reference operator* () const {return *iter_;}

    pointer operator-> () const {return iter_;}

    value_type operator[] (difference_type diff) const
    {return *(iter_ + diff * inc_);}

    StepRandomIterator<RandomIter> &operator++ ()
    {iter_ += inc_ ; return *this;}

    StepRandomIterator<RandomIter> operator++ (int)
    {StepRandomIterator<RandomIter> iter(*this); return ++iter;}

    StepRandomIterator<RandomIter> &operator-- ()
    {iter_ -= inc_; return *this;}

    StepRandomIterator<RandomIter> operator-- (int)
    {StepRandomIterator<RandomIter> iter(*this); return --iter;}

    StepRandomIterator<RandomIter> &operator+= (difference_type diff)
    {iter_ += diff * inc_; return *this;}

    StepRandomIterator<RandomIter> &operator-= (difference_type diff)
    {iter_ -= diff * inc_; return *this;}

    private :

    RandomIter iter_;
    std::size_t inc_;
}; // class StepRandomIterator

template <typename RandomIter1, typename RandomIter2>
inline bool operator== (
        const StepRandomIterator<RandomIter1> &iter1,
        const StepRandomIterator<RandomIter2> &iter2)
{
    return
        (iter1.iterator() == iter2.iterator()) &&
        (iter1.increment() == iter2.increment());
}

template <typename RandomIter1, typename RandomIter2>
inline bool operator!= (
        const StepRandomIterator<RandomIter1> &iter1,
        const StepRandomIterator<RandomIter2> &iter2)
{
    return
        (iter1.iterator() != iter2.iterator()) ||
        (iter1.increment() != iter2.increment());
}

template <typename RandomIter1, typename RandomIter2>
inline std::ptrdiff_t operator- (
        const StepRandomIterator<RandomIter1> &iter1,
        const StepRandomIterator<RandomIter2> &iter2)
{
    VSMC_RUNTIME_ASSERT_SMP_INTERNAL_ITERATOR_BINARY_OP;

    return (iter1.iterator() - iter2.iterator()) / iter1.increment();
}

template <typename RandomIter>
inline StepRandomIterator<RandomIter> operator+ (
        const StepRandomIterator<RandomIter> &iter,
        typename StepRandomIterator<RandomIter>::difference_type diff)
{
    StepRandomIterator<RandomIter> new_iter(iter);
    new_iter += diff;

    return new_iter;
}

template <typename RandomIter>
inline StepRandomIterator<RandomIter> operator+ (
        typename StepRandomIterator<RandomIter>::difference_type diff,
        const StepRandomIterator<RandomIter> &iter) {return iter + diff;}

template <typename RandomIter>
inline StepRandomIterator<RandomIter> operator- (
        const StepRandomIterator<RandomIter> &iter,
        typename StepRandomIterator<RandomIter>::difference_type diff)
{
    StepRandomIterator<RandomIter> new_iter(iter);
    new_iter -= diff;

    return new_iter;
}

} // namespace vsmc

#endif // VSMC_SMP_INTERNAL_ITERATOR_HPP
