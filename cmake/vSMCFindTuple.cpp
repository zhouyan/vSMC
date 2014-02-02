#include <vsmc/utility/tuple_manip.hpp>

template <typename VecType, typename TpType>
inline void assign_vec (VecType &vec, const TpType &tp, vsmc::Position<0>)
{
    std::get<0>(vec).push_back(std::get<0>(tp));
}

template <typename VecType, typename TpType, std::size_t Pos>
inline void assign_vec (VecType &vec, const TpType &tp, vsmc::Position<Pos>)
{
    std::get<Pos>(vec).push_back(std::get<Pos>(tp));
    assign_vec(vec, tp, vsmc::Position<Pos - 1>());
}

template <typename... Types>
inline std::tuple<std::vector<Types>...> create_vec (
        const std::tuple<Types...> &tp)
{
    std::tuple<std::vector<Types>...> vec;
    assign_vec(vec, tp, vsmc::Position<sizeof...(Types) - 1>());

    return vec;
}

int main ()
{
    assert(VSMC_HAS_CXX11_VARIADIC_TEMPLATES);

    using vsmc::cxx11::is_same;

    typedef std::tuple<> empty;
    typedef std::tuple<char, short, int, long> full;
    typedef std::tuple<char, short> tp1;
    typedef std::tuple<int, long> tp2;

    {
        typedef vsmc::TuplePushFront<empty, int>::type t1;
        assert((is_same<t1, std::tuple<int> >::value));

        typedef vsmc::TuplePushFront<t1, char>::type t2;
        assert((is_same<t2, std::tuple<char, int> >::value));

        typedef vsmc::TuplePushFront<t2, long>::type t3;
        assert((is_same<t3, std::tuple<long, char, int> >::value));
    }

    {
        typedef vsmc::TuplePushBack<empty, int>::type t1;
        assert((is_same<t1, std::tuple<int> >::value));

        typedef vsmc::TuplePushBack<t1, char>::type t2;
        assert((is_same<t2, std::tuple<int, char> >::value));

        typedef vsmc::TuplePushBack<t2, long>::type t3;
        assert((is_same<t3, std::tuple<int, char, long> >::value));
    }

    {
        typedef vsmc::TuplePopFront<full>::type t1;
        assert((is_same<t1, std::tuple<short, int, long> >::value));

        typedef vsmc::TuplePopFront<t1>::type t2;
        assert((is_same<t2, std::tuple<int, long> >::value));

        typedef vsmc::TuplePopFront<t2>::type t3;
        assert((is_same<t3, std::tuple<long> >::value));

        typedef vsmc::TuplePopFront<t3>::type t4;
        assert((is_same<t4, std::tuple<> >::value));
    }

    {
        typedef vsmc::TuplePopBack<full>::type t1;
        assert((is_same<t1, std::tuple<char, short, int> >::value));

        typedef vsmc::TuplePopBack<t1>::type t2;
        assert((is_same<t2, std::tuple<char, short> >::value));

        typedef vsmc::TuplePopBack<t2>::type t3;
        assert((is_same<t3, std::tuple<char> >::value));

        typedef vsmc::TuplePopBack<t3>::type t4;
        assert((is_same<t4, std::tuple<> >::value));
    }

    {
        typedef vsmc::TuplePopFrontN<full, 0>::type t0;
        assert((is_same<t0, std::tuple<char, short, int, long> >::value));

        typedef vsmc::TuplePopFrontN<full, 1>::type t1;
        assert((is_same<t1, std::tuple<short, int, long> >::value));

        typedef vsmc::TuplePopFrontN<full, 2>::type t2;
        assert((is_same<t2, std::tuple<int, long> >::value));

        typedef vsmc::TuplePopFrontN<full, 3>::type t3;
        assert((is_same<t3, std::tuple<long> >::value));

        typedef vsmc::TuplePopFrontN<full, 4>::type t4;
        assert((is_same<t4, std::tuple<> >::value));

        typedef vsmc::TuplePopFrontN<full, 5>::type t5;
        assert((is_same<t5, std::tuple<> >::value));

        typedef vsmc::TuplePopFrontN<empty, 1>::type t6;
        assert((is_same<t6, std::tuple<> >::value));
    }

    {
        typedef vsmc::TuplePopBackN<full, 0>::type t0;
        assert((is_same<t0, std::tuple<char, short, int, long> >::value));

        typedef vsmc::TuplePopBackN<full, 1>::type t1;
        assert((is_same<t1, std::tuple<char, short, int> >::value));

        typedef vsmc::TuplePopBackN<full, 2>::type t2;
        assert((is_same<t2, std::tuple<char, short> >::value));

        typedef vsmc::TuplePopBackN<full, 3>::type t3;
        assert((is_same<t3, std::tuple<char> >::value));

        typedef vsmc::TuplePopBackN<full, 4>::type t4;
        assert((is_same<t4, std::tuple<> >::value));

        typedef vsmc::TuplePopBackN<full, 5>::type t5;
        assert((is_same<t5, std::tuple<> >::value));

        typedef vsmc::TuplePopBackN<empty, 1>::type t6;
        assert((is_same<t6, std::tuple<> >::value));
    }

    {
        typedef vsmc::TupleMerge<tp1, tp2>::type t1;
        assert((is_same<t1, std::tuple<char, short, int, long> >::value));

        typedef vsmc::TupleMerge<tp2, tp1>::type t2;
        assert((is_same<t2, std::tuple<int, long, char, short> >::value));

        typedef vsmc::TupleMerge<tp1, empty>::type t3;
        assert((is_same<t3, tp1>::value));

        typedef vsmc::TupleMerge<empty, tp1>::type t4;
        assert((is_same<t4, tp1>::value));

        typedef vsmc::TupleMerge<empty, empty>::type t5;
        assert((is_same<t5, empty>::value));
    }

    {
        typedef vsmc::TupleCat<
            empty, full, empty, tp1, empty, tp2, empty>::type t1;
        assert((is_same<t1, std::tuple<
                    char, short, int, long, char, short, int, long> >::value));
    }

    {
        full tp;
        std::get<0>(tp) = 0;
        std::get<1>(tp) = 1;
        std::get<2>(tp) = 2;
        std::get<3>(tp) = 3;
        std::tuple<
            std::vector<char>,
            std::vector<short>,
            std::vector<int>,
            std::vector<long>
                > vec = create_vec(tp);
        assert((std::get<0>(tp) == std::get<0>(vec)[0]));
        assert((std::get<1>(tp) == std::get<1>(vec)[0]));
        assert((std::get<2>(tp) == std::get<2>(vec)[0]));
        assert((std::get<3>(tp) == std::get<3>(vec)[0]));
    }

    return 0;
}
