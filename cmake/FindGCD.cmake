# Find Apple GCD support
#
# The following variable is set
#
# GCD_FOUND - TRUE if Apple GCD is found and work correctly. Currently only
#             supported on Mac OS X

SET (GCD_TEST_SOURCE "
#include <dispatch/dispatch.h>
#include <cassert>
#include <cstdlib>

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
                (void *) this, norm_work);
        IntType sum = 0;
        for (IntType i = 0; i != num; ++i)
            sum += square_[i];

        return sum;
    }

    private :

    static void norm_work (void *test, std::size_t i)
    {
        TestGCD<IntType> *pt = reinterpret_cast<TestGCD<IntType> *>(test);
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
}")

IF (NOT GCD_FOUND)
    UNSET (GCD_FOUND CACHE)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${GCD_TEST_SOURCE}" GCD_FOUND)
    IF (GCD_FOUND)
        MESSAGE (STATUS "Find Apple GCD support")
    ELSE (GCD_FOUND)
        MESSAGE (STATUS "NOT Find GCD Plus support")
    ENDIF (GCD_FOUND)
ENDIF (NOT GCD_FOUND)
