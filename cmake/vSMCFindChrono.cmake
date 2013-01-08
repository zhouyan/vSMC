SET (VSMC_CHRONO_TEST_SOURCE "
#include <chrono>
#include <iostream>

int main () {
    std::chrono::time_point<std::chrono::system_clock> bt =
        std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> et =
        std::chrono::system_clock::now();
    std::cout <<
        std::chrono::duration_cast<std::chrono::microseconds>(et - bt)
        .count() << std::endl;

    return 0;
}
")

IF (NOT VSMC_CHRONO_FOUND)
    UNSET (VSMC_CHRONO_FOUND CACHE)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_CHRONO_TEST_SOURCE}" VSMC_CHRONO_FOUND)
    IF (VSMC_CHRONO_FOUND)
        SET (VSMC_CHRONO_FOUND TRUE CACHE BOOL "Found C++11 chrono")
    ENDIF (VSMC_CHRONO_FOUND)
ENDIF (NOT VSMC_CHRONO_FOUND)
