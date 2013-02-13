#ifndef VSMC_UTILITY_TUPLE_MANIP_HPP
#define VSMC_UTILITY_TUPLE_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <tuple>

namespace vsmc {

/// \brief Push a type to the front of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePushFront<T, std::tuple<Types...> >
{
    typedef std::tuple<T, Types...> type;
};

/// \brief Push a type to the back of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePushBack<T, std::tuple<Types...> >
{
    typedef std::tuple<Types..., T> type;
};

/// \brief Remove a type from the front of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePopFront<std::tuple<T, Types...> >
{
    typedef std::tuple<Types...> type;
};

/// \brief Remove a type from the front of a std::tuple type
/// \ingroup Utility
template <>
struct TuplePopFront<std::tuple<> >
{
    typedef std::tuple<> type;
};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePopBack<std::tuple<T, Types...> >
{
    private :

    typedef typename TuplePopBack<std::tuple<Types...> >::type tail_type;

    public :

    typedef typename TuplePushFront<T, tail_type>::type type;
};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Utility
template <typename T>
struct TuplePopBack<std::tuple<T> >
{
    typedef std::tuple<> type;
};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Utility
template <>
struct TuplePopBack<std::tuple<> >
{
    typedef std::tuple<> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <std::size_t N, typename T, typename... Types>
struct TuplePopFrontN<std::tuple<T, Types...>, N>
{
    typedef typename TuplePopFrontN<std::tuple<Types...>, N - 1>::type type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePopFrontN<std::tuple<T, Types...>, 0>
{
    typedef std::tuple<T, Types...> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <std::size_t N>
struct TuplePopFrontN<std::tuple<>, N>
{
    typedef std::tuple<> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <>
struct TuplePopFrontN<std::tuple<>, 0>
{
    typedef std::tuple<> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <std::size_t N, typename T, typename... Types>
struct TuplePopBackN<std::tuple<T, Types...>, N>
{
    typedef typename TuplePopBackN<
        typename TuplePopBack<std::tuple<T, Types...> >::type,
                 N - 1>::type type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <typename T, typename... Types>
struct TuplePopBackN<std::tuple<T, Types...>, 0>
{
    typedef std::tuple<T, Types...> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <std::size_t N>
struct TuplePopBackN<std::tuple<>, N>
{
    typedef std::tuple<> type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Utility
template <>
struct TuplePopBackN<std::tuple<>, 0>
{
    typedef std::tuple<> type;
};

/// \brief Merge two std::tuple types
/// \ingroup Utility
template <typename T1, typename T2>
struct TupleMerge
{
    private :

    typedef typename TuplePushBack<
        typename std::tuple_element<0, T2>::type, T1>::type head_type;
    typedef typename TuplePopFront<T2>::type tail_type;

    public :

    typedef typename TupleMerge<head_type, tail_type>::type type;
};

/// \brief Merge two std::tuple types
/// \ingroup Utility
template <typename T, typename... Types>
struct TupleMerge<std::tuple<>, std::tuple<T, Types...> >
{
    typedef std::tuple<T, Types...> type;
};

/// \brief Merge two std::tuple types
/// \ingroup Utility
template <typename T, typename... Types>
struct TupleMerge<std::tuple<T, Types...>, std::tuple<> >
{
    typedef std::tuple<T, Types...> type;
};

/// \brief Merge two std::tuple types
/// \ingroup Utility
template <>
struct TupleMerge<std::tuple<>, std::tuple<> >
{
    typedef std::tuple<> type;
};

/// \brief Generate a new std::tuple types by apply a class template
/// \ingroup Utility
///
/// \details
/// Give a `std::tuple<T1, T2, ...>` and a class template `C`, the member type
/// is `std::tuple<C<T1>, C<T2>, ...>`.
template <template <typename> class C, typename T, typename... Types>
struct TupleApply<C, std::tuple<T, Types...> >
{
    typedef typename TuplePushFront<C<T>,
            typename TupleApply<C, std::tuple<Types...> >::type>::type type;
};

/// \brief Generate a new std::tuple types by apply a class template
/// \ingroup Utility
template <template <typename> class C>
struct TupleApply<C, std::tuple<> >
{
    typedef std::tuple<> type;
};

}; // namespace vsmc

/// \brief Define a TupleApply class
/// \ingroup Utility
///
/// \details
/// Define a class template in namespace vsmc::tuple
/// \code
/// struct TupleApplyOuter<typename T>;
/// \endcode
/// where `T` is a `std::tuple` type, say `std::tuple<T1, T2>`. The struct has
/// memeber type
/// - `type`: std::tuple<Inner<T1>, Inner<T2>, ...>
/// Note that the class template `Inner` can have more than one template
/// parameter. However, all but the first must have default arguments
///
/// **Example**
/// \code
/// VSMC_DEFINE_TUPLE_APPLY(Vector, std::vector);
///
/// typedef vsmc::tuple::TupleApplyVector<
///     std::tuple<char, short, int> >::type t1;
/// typedef std::tuple<
///     std::vector<char>, std::vector<short>, std::vector<int> >::type t2;
/// vsmc::traits::is_same<t1, t2>::value; // true
/// \endcode
///
/// For class template `Inner` which has only one template parameter the macro
/// definition is not needed. One can simply use `vsmc::TupleApply`
/// \code
/// template <typename T> class Vector; 
/// typedef vsmc::TupleApply<Vector, std::tuple<char, short, int> >::type t1;
/// typedef std::tuple<Vector<char>, Vector<short>, Vector<int> >::type t2;
/// vsmc::traits::is_same<t1, t2>::value; // true
/// \endcode
/// Ideally the more general case, where the class template has more than one
/// template parameter can be handled by `TupleApply` through variadic
/// template. However currently compiler support for variadic template is not
/// very robust. The general `TupleApply` at the time of writing only works
/// with GCC 4.7 and Clang 3.2
///
/// `TupleApply` for all C++03 containers but std::map are defined
#define VSMC_DEFINE_TUPLE_APPLY(Outer, Inner)                                 \
namespace vsmc { namespace tuple {                                            \
                                                                              \
template <typename T, typename... Types>                                      \
struct TupleApply##Outer<std::tuple<T, Types...> >                            \
{                                                                             \
    typedef typename TuplePushFront<Inner<T>, typename TupleApply##Outer<     \
        std::tuple<Types...> >::type>::type type;                             \
};                                                                            \
                                                                              \
template <>                                                                   \
struct TupleApply##Outer<std::tuple<> >                                       \
{                                                                             \
    typedef std::tuple<> type;                                                \
};                                                                            \
                                                                              \
} } // namespace vsmc::tuple

VSMC_DEFINE_TUPLE_APPLY(Deque,         std::deque)
VSMC_DEFINE_TUPLE_APPLY(List,          std::list)
VSMC_DEFINE_TUPLE_APPLY(PriorityQueue, std::priority_queue)
VSMC_DEFINE_TUPLE_APPLY(Queue,         std::queue)
VSMC_DEFINE_TUPLE_APPLY(Set,           std::set)
VSMC_DEFINE_TUPLE_APPLY(Stack,         std::stack)
VSMC_DEFINE_TUPLE_APPLY(Vector,        std::vector)

#endif // VSMC_UTILITY_TUPLE_MANIP_HPP
