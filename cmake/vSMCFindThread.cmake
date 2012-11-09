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
    UNSET (VSMC_THREAD_FOUND       CACHE)
    UNSET (VSMC_THREAD_STD_FOUND   CACHE)
    UNSET (VSMC_THREAD_BOOST_FOUND CACHE)
    IF (NOT Boost_FOUND)
        INCLUDE (FindBoost)
    ENDIF (NOT Boost_FOUND)
    INCLUDE (FindThreads)
    INCLUDE (CheckCXXSourceRuns)
    FIND_PACKAGE (Boost OPTIONAL_COMPONENTS chrono date_time thread system)
    SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRAREIS})
    SET (CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES}
        ${Boost_INCLUDE_DIRS})
    SET (CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT})
    SET (CMAKE_REQUIRED_DEFINITIONS -DVSMC_HAS_CXX11LIB_THREAD=1)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_THREAD_TEST_SOURCE}"
        VSMC_THREAD_STD_FOUND)
    IF (Boost_FOUND)
        SET (CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
            ${Boost_CHRONO_LIBRARY_DEBUG} ${Boost_DATE_TIME_LIBRARY_DEBUG}
            ${Boost_SYSTEM_LIBRARY_DEBUG} ${Boost_THREAD_LIBRARY_DEBUG}
            ${CMAKE_THREAD_LIBS_INIT})
        SET (CMAKE_REQUIRED_DEFINITIONS -DVSMC_HAS_CXX11LIB_THREAD=0)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_THREAD_TEST_SOURCE}"
            VSMC_THREAD_BOOST_FOUND)
    ENDIF (Boost_FOUND)
    IF (VSMC_THREAD_STD_FOUND OR VSMC_THREAD_BOOST_FOUND)
        SET (VSMC_THREAD_FOUND TRUE CACHE BOOL "Found thread")
    ENDIF (VSMC_THREAD_STD_FOUND OR VSMC_THREAD_BOOST_FOUND)
    IF (VSMC_THREAD_STD_FOUND)
        SET (VSMC_THREAD_STD_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT}
            CACHE STRING "Link to STD thread")
    ENDIF (VSMC_THREAD_STD_FOUND)
    IF (VSMC_THREAD_BOOST_FOUND)
        SET (VSMC_THREAD_BOOST_LINK_LIBRARIES
            ${Boost_CHRONO_LIBRARY} ${Boost_DATE_TIME_LIBRARY}
            ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY}
            ${CMAKE_THREAD_LIBS_INIT}
            CACHE STRING "Link to Boost thread")
    ENDIF (VSMC_THREAD_BOOST_FOUND)
    IF (VSMC_THREAD_STD_FOUND)
        SET (VSMC_THREAD_LINK_LIBRARIES ${VSMC_THREAD_STD_LINK_LIBRARIES}
            CACHE STRING "Link to thread")
    ELSEIF (VSMC_THREAD_BOOST_FOUND)
        SET (VSMC_THREAD_LINK_LIBRARIES ${VSMC_THREAD_BOOST_LINK_LIBRARIES}
            CACHE STRING "Link to thread")
    ENDIF (VSMC_THREAD_STD_FOUND)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRAREIS})
ENDIF (NOT VSMC_THREAD_FOUND)
