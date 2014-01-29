#include <iostream>
#include <vsmc/cxx11/random.hpp>

#define NO_EXPANSION

template<uint64_t, uint64_t>
struct IntegerRangeTypeTrait {typedef int type;};

template<>
struct IntegerRangeTypeTrait<0, ~((uint32_t)0)> {typedef uint32_t type;};

template<>
struct IntegerRangeTypeTrait<0, ~((uint64_t)0)> {typedef uint64_t type;};

int main ()
{
    std::cout << sizeof(IntegerRangeTypeTrait<
            vsmc::cxx11::mt19937::min NO_EXPANSION (),
            vsmc::cxx11::mt19937::max NO_EXPANSION ()>::type) << std::endl;

    return 0;
}
