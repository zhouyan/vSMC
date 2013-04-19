SET (VSMC_STDTBB_TEST_SOURCE "
#include <vsmc/utility/stdtbb.hpp>
#include <mutex>
#include <set>

#include <iostream>
#include <cassert>

std::mutex mtx;
std::set<std::thread::id> unique_id;

void id_set (const vsmc::BlockedRange<int> &)
{
    std::lock_guard<std::mutex> lock(mtx);
    unique_id.insert(std::this_thread::get_id());
}

class square
{
    public :

    square (std::vector<int> &N) : N_(N) {}

    void operator() (const vsmc::BlockedRange<int> &block)
    {
        for (int i = block.begin(); i != block.end(); ++i)
            N_[i] = N_[i] * N_[i];
    }

    private :

    std::vector<int> &N_;
};

void sum (const vsmc::BlockedRange<int> &block, int &res)
{
    res = 0;
    for (int i = block.begin(); i != block.end(); ++i)
        res += i;
}

class prod
{
    public :

    template <typename IntType>
    void operator() (const vsmc::BlockedRange<IntType> &block,
            int &res)
    {
        res = 1;
        for (IntType i = block.begin(); i != block.end(); ++i)
            res *= i;
    }
};

int mul (int a, int b)
{
    return a * b;
}

int main ()
{

    std::vector<int> L, M;
    for (int i = 0; i != 100; ++i) {
        L.push_back(i);
        M.push_back(i * i);
    }

    for (int i = 0; i != 1000; ++i) {
        vsmc::parallel_for(vsmc::BlockedRange<int>(0, 100), id_set);

        std::vector<int> N(L);
        vsmc::parallel_for(vsmc::BlockedRange<int>(0, 100), square(N));
        for (int i = 0; i != 100; ++i)
            assert(N[i] == M[i]);

        int S = vsmc::parallel_accumulate(vsmc::BlockedRange<int>(0, 100),
                sum, 0);
        assert(S == 4950);

        int P = vsmc::parallel_accumulate(vsmc::BlockedRange<int>(1, 10),
                prod(), 1, mul);
        assert(P == 362880);
    }

    for (std::set<std::thread::id>::iterator
            i = unique_id.begin(); i != unique_id.end(); ++i) {
        std::cout << *i << std::endl;
    }

    return 0;
}
")

IF (NOT DEFINED VSMC_STDTBB_FOUND)
    INCLUDE (FindThreads)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRAREIS})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT})
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        -DVSMC_HAS_CXX11LIB_FUTURE=0)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_STDTBB_TEST_SOURCE}" VSMC_THREAD_FOUND)
    IF (VSMC_THREAD_FOUND)
        SET (VSMC_THREAD_FOUND TRUE CACHE BOOL "Found C++11 thread")
    ENDIF (VSMC_THREAD_FOUND)
    IF (VSMC_ENABLE_CXX11LIB_FUTURE)
        SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
            -DVSMC_HAS_CXX11LIB_FUTURE=1)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_STDTBB_TEST_SOURCE}" VSMC_FUTURE_FOUND)
        IF (VSMC_FUTURE_FOUND)
            SET (VSMC_FUTURE_FOUND TRUE CACHE BOOL "Found C++11 future")
        ENDIF (VSMC_FUTURE_FOUND)
    ENDIF (VSMC_ENABLE_CXX11LIB_FUTURE)
    IF (VSMC_THREAD_FOUND OR VSMC_FUTURE_FOUND)
        SET (VSMC_STDTBB_FOUND TRUE CACHE BOOL "Found vSMC STDTBB")
        SET (VSMC_THREAD_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT}
            CACHE STRING "Link to thread")
    ELSE (VSMC_THREAD_FOUND OR VSMC_FUTURE_FOUND)
        SET (VSMC_STDTBB_FOUND FALSE CACHE BOOL "NOT Found vSMC STDTBB")
    ENDIF (VSMC_THREAD_FOUND OR VSMC_FUTURE_FOUND)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRAREIS})
ENDIF (NOT DEFINED VSMC_STDTBB_FOUND)
