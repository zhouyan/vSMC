#include <iostream>
#include <cassert>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

class say
{
    public :

    template <typename IntType>
    void operator() (const tbb::blocked_range<IntType> &block) const
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

    sum () : res_(0) {}

    sum (const sum &other, tbb::split) : res_(0) {}

    void join (const sum &other)
    {
        res_ += other.res_;
    }

    template <typename IntType>
    void operator() (const tbb::blocked_range<IntType> &block)
    {
        long r = res_;
        for (IntType i = block.begin(); i != block.end(); ++i)
            r += i;
        res_ = r;
    }

    long res () const
    {
        return res_;
    }

    private :
    long res_;
};

int main ()
{
    tbb::parallel_for(tbb::blocked_range<int>(0, 100), say());
    sum s;
    tbb::parallel_reduce(tbb::blocked_range<int>(0, 100), s);
    assert(s.res() == 4950);
    return 0;
}
