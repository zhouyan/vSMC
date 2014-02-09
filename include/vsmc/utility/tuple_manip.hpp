#ifndef VSMC_UTILITY_TUPLE_MANIP_HPP
#define VSMC_UTILITY_TUPLE_MANIP_HPP

#include <vsmc/internal/common.hpp>
#include <tuple>

namespace vsmc {

template <typename, typename> struct TuplePushFront;
template <typename, typename> struct TuplePushBack;
template <typename> struct TuplePopFront;
template <typename> struct TuplePopBack;
template <typename, std::size_t> struct TuplePopFrontN;
template <typename, std::size_t> struct TuplePopBackN;
template <typename, typename> struct TupleMerge;
template <typename...> struct TupleCat;

} // namespace vsmc

namespace vsmc {

/// \brief Push a type to the front of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePushFront<std::tuple<Types...>, T>
{typedef std::tuple<T, Types...> type;};

/// \brief Push a type to the back of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePushBack<std::tuple<Types...>, T>
{typedef std::tuple<Types..., T> type;};

/// \brief Remove a type from the front of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePopFront<std::tuple<T, Types...> >
{typedef std::tuple<Types...> type;};

/// \brief Remove a type from the front of a std::tuple type
/// \ingroup Tuple
template <>
struct TuplePopFront<std::tuple<> >
{typedef std::tuple<> type;};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePopBack<std::tuple<T, Types...> >
{
    private :

    typedef typename TuplePopBack<std::tuple<Types...> >::type tail_type;

    public :

    typedef typename TuplePushFront<tail_type, T>::type type;
};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Tuple
template <typename T>
struct TuplePopBack<std::tuple<T> >
{typedef std::tuple<> type;};

/// \brief Remove a type from the back of a std::tuple type
/// \ingroup Tuple
template <>
struct TuplePopBack<std::tuple<> >
{typedef std::tuple<> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <std::size_t N, typename T, typename... Types>
struct TuplePopFrontN<std::tuple<T, Types...>, N>
{typedef typename TuplePopFrontN<std::tuple<Types...>, N - 1>::type type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePopFrontN<std::tuple<T, Types...>, 0>
{typedef std::tuple<T, Types...> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <std::size_t N>
struct TuplePopFrontN<std::tuple<>, N>
{typedef std::tuple<> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <>
struct TuplePopFrontN<std::tuple<>, 0>
{typedef std::tuple<> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <std::size_t N, typename T, typename... Types>
struct TuplePopBackN<std::tuple<T, Types...>, N>
{
    typedef typename TuplePopBackN<
        typename TuplePopBack<std::tuple<T, Types...> >::type,
                 N - 1>::type type;
};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <typename T, typename... Types>
struct TuplePopBackN<std::tuple<T, Types...>, 0>
{typedef std::tuple<T, Types...> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <std::size_t N>
struct TuplePopBackN<std::tuple<>, N>
{typedef std::tuple<> type;};

/// \brief Remove N types from the front of a std::tuple type
/// \ingroup Tuple
template <>
struct TuplePopBackN<std::tuple<>, 0>
{typedef std::tuple<> type;};

/// \brief Merge two std::tuple types
/// \ingroup Tuple
template <typename T1, typename T2>
struct TupleMerge
{
    private :

    typedef typename TuplePushBack<T1,
        typename std::tuple_element<0, T2>::type>::type head_type;
    typedef typename TuplePopFront<T2>::type tail_type;

    public :

    typedef typename TupleMerge<head_type, tail_type>::type type;
};

/// \brief Merge two std::tuple types
/// \ingroup Tuple
template <typename T, typename... Types>
struct TupleMerge<std::tuple<>, std::tuple<T, Types...> >
{typedef std::tuple<T, Types...> type;};

/// \brief Merge two std::tuple types
/// \ingroup Tuple
template <typename T, typename... Types>
struct TupleMerge<std::tuple<T, Types...>, std::tuple<> >
{typedef std::tuple<T, Types...> type;};

/// \brief Merge two std::tuple types
/// \ingroup Tuple
template <>
struct TupleMerge<std::tuple<>, std::tuple<> >
{typedef std::tuple<> type;};

/// \brief Concatenate multiple std::tuple types
/// \ingroup Tuple
template <typename T1, typename T2, typename... Types>
struct TupleCat<T1, T2, Types...>
{
    private :

    typedef typename TupleMerge<T1, T2>::type head_type;

    public :

    typedef typename TupleCat<head_type, Types...>::type type;
};

/// \brief Concatenate multiple std::tuple types
/// \ingroup Tuple
template <typename... Types>
struct TupleCat<std::tuple<Types...> >
{typedef std::tuple<Types...> type;};

} // namespace vsmc

#endif // VSMC_UTILITY_TUPLE_MANIP_HPP
