INCLUDE (CheckCXXSourceRuns)

SET (TRY_CILK_CPP "
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
    UNSET (CILK_FOUND CACHE)
    MESSAGE (STATUS "Try Cilk Plus extension")
    CHECK_CXX_SOURCE_RUNS ("${TRY_CILK_CPP}" CILK_FOUND)
    IF (CILK_FOUND)
        MESSAGE (STATUS "Cilk Plus supported")
    ELSE (CILK_FOUND)
        MESSAGE (STATUS "Cilk Plus NOT supported")
    ENDIF (CILK_FOUND)
ENDIF (NOT CILK_FOUND)
