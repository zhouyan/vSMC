#ifndef VSMC_UTILITY_META_HPP
#define VSMC_UTILITY_META_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

template <unsigned Dim>
struct StaticFor
{
    template <typename Iter, typename Op>
    static void apply (Iter first, Op op)
    {
	op(first);
	StaticFor<Dim - 1>::apply(++first, op);
    }

    template <typename Iter1, typename Iter2, typename Op>
    static void apply (Iter1 first1, Iter2 first2, Op op)
    {
	op(first1, first2);
	StaticFor<Dim - 1>::apply(++first1, ++first2, op);
    }
}; // struct StaticFor

template <>
struct StaticFor<1U>
{
    template <typename Iter, typename UnaryOp>
    static void apply (Iter first, UnaryOp op)
    {
	op(first);
    }

    template <typename Iter1, typename Iter2, typename Op>
    static void apply (Iter1 first1, Iter2 first2, Op op)
    {
	op(first1, first2);
    }
}; // struct StaticFor

} // namespace vsmc
#endif // VSMC_UTILITY_META_HPP
