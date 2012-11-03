#include <iostream>
#include <vsmc/utility/stdtbb.hpp>

class say
{
    public :

    template <typename IntType>
    void operator() (const vsmc::thread::BlockedRange<IntType> &block)
    {
        IntType sum = 0;
        for (IntType i = block.begin(); i != block.end(); ++i)
            sum += i;
        std::cout << sum << std::endl;
    }
};

class sum
{
    public :

    template <typename IntType>
    void operator() (const vsmc::thread::BlockedRange<IntType> &block,
            int &res)
    {
        res = 0;
        for (IntType i = block.begin(); i != block.end(); ++i)
            res += i;
    }
};

int main ()
{
    int r;
    vsmc::thread::parallel_for(vsmc::thread::BlockedRange<int>(0, 100), say());
    vsmc::thread::parallel_sum(vsmc::thread::BlockedRange<int>(0, 100), sum(), r);
    assert(r == 4950);
    return 0;
}
