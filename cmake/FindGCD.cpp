//============================================================================
// cmake/FindGCD.cpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#include <dispatch/dispatch.h>
#include <cstdlib>
#include <cassert>

template <typename IntType>
class TestGCD
{
    public :

    IntType norm (IntType begin, IntType end)
    {
        std::size_t num = static_cast<std::size_t>(end - begin);
        begin_ = begin;
        dispatch_apply_f(num,
                dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                static_cast<void *>(this), norm_work);
        IntType sum = 0;
        for (IntType i = 0; i != end - begin; ++i)
            sum += square_[i];

        return sum;
    }

    private :

    static void norm_work (void *test, std::size_t i)
    {
        TestGCD<IntType> *pt = static_cast<TestGCD<IntType> *>(test);
        IntType n = static_cast<IntType>(i) + pt->begin_;
        pt->square_[i] = n * n;
    }

    IntType square_[100];
    IntType begin_;
};

int main ()
{
    TestGCD<int> test;
    assert(test.norm(1, 11) == 385);

    return 0;
}
