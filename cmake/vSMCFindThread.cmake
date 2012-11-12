SET (VSMC_THREAD_TEST_SOURCE "
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
")

IF (NOT VSMC_THREAD_FOUND)
    UNSET (VSMC_THREAD_FOUND CACHE)
    INCLUDE (FindThreads)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRAREIS})
    SET (CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT})
    CHECK_CXX_SOURCE_RUNS ("${VSMC_THREAD_TEST_SOURCE}" VSMC_THREAD_FOUND)
    IF (VSMC_THREAD_FOUND)
        SET (VSMC_THREAD_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT}
            CACHE STRING "Link to thread")
        SET (VSMC_THREAD_FOUND TRUE CACHE BOOL "Found C++11 thread")
    ENDIF (VSMC_THREAD_FOUND)
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRAREIS})
ENDIF (NOT VSMC_THREAD_FOUND)
