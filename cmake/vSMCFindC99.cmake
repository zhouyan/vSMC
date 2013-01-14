IF (NOT VSMC_C99_FOUND)
    UNSET (VSMC_C99_FOUND CACHE)
    INCLUDE (CheckCSourceRuns)
    SET (C99_TEST_SOURCE "
    #include <math.h>
    int main ()
    {
        double y = 0;
        for (int i = 0; i != 100; ++i) {
            double x = i;
            x = x * x;
            y += log(sqrt(exp(x)));
        }
        return y == 0;
    }")
    CHECK_C_SOURCE_RUNS ("${C99_TEST_SOURCE}" VSMC_C99_FOUND)
    IF (NOT VSMC_C99_FOUND)
        UNSET (VSMC_C99_FOUND CACHE)
        SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES} -lm)
        CHECK_C_SOURCE_RUNS ("${C99_TEST_SOURCE}" VSMC_C99_FOUND)
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
        IF (VSMC_C99_FOUND)
            SET (VSMC_C_REQUIRED_LIBRARIES -lm CACHE STRING
                "C Required Libraries")
        ENDIF (VSMC_C99_FOUND)
    ENDIF (NOT VSMC_C99_FOUND)
ENDIF (NOT VSMC_C99_FOUND)
