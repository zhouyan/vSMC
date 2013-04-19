SET (VSMC_GCD_TEST_SOURCE "
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
                (void *) this, norm_work);
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
")

INCLUDE (FindGCD)
IF (GCD_FOUND AND NOT DEFINED VSMC_GCD_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${GCD_INCLUDE_DIR})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${GCD_LINK_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS ("${VSMC_GCD_TEST_SOURCE}" VSMC_GCD_FOUND)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ENDIF (GCD_FOUND AND NOT DEFINED VSMC_GCD_FOUND)
