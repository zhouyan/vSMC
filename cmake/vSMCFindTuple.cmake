SET (VSMC_TUPLE_TEST_SOURCE "
#include <vsmc/internal/compiler.hpp>
#include <tuple>
#include <cassert>
int main () {assert(VSMC_HAS_CXX11_VARIADIC_TEMPLATES);}")

IF (NOT VSMC_TUPLE_FOUND)
    UNSET (VSMC_TUPLE_FOUND CACHE)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_TUPLE_TEST_SOURCE}" VSMC_TUPLE_FOUND)
ENDIF (NOT VSMC_TUPLE_FOUND)
