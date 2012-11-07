# Find Intel Cilk Plus support
#
# The following variable is set
#
# CILK_FOUND - TRUE if Cilk Plus is found and work correctly. Currently only
#              Intel compiler has this feature

SET (CILK_TEST_SOURCE "
#include <cilk/cilk.h>
#include <cilk/reducer_opadd.h>

int main ()
{
    unsigned N = 1000;
    unsigned C = (N * (N - 1)) / 2;
    cilk::reducer_opadd<unsigned> T;
    cilk_for(unsigned i = 0; i != N; ++i) T += i;

    return T.get_value() != C;
}")

IF (NOT CILK_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    UNSET (CILK_FOUND CACHE)
    CHECK_CXX_SOURCE_RUNS ("${CILK_TEST_SOURCE}" CILK_TEST_SOURCE_RUNS)
    IF (CILK_TEST_SOURCE_RUNS)
        SET (CILK_FOUND TRUE)
        MESSAGE (STATUS "Find Cilk Plus support")
    ELSE (CILK_TEST_SOURCE_RUNS)
        SET (CILK_FOUND FALSE)
        MESSAGE (STATUS "NOT Find Cilk Plus support")
    ENDIF (CILK_TEST_SOURCE_RUNS)
ENDIF (NOT CILK_FOUND)
