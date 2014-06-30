# ============================================================================
#  cmake/FindCXX11LibTuple.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find C++11 <tuple> support
#
# The following variable is set
#
# CXX11LIB_TUPLE_FOUND - TRUE if C++11 <tuple> is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCXX11LibTuple.cpp
    CXX11LIB_TUPLE_TEST_SOURCE)

IF (NOT DEFINED CXX11LIB_TUPLE_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${CXX11LIB_TUPLE_TEST_SOURCE}"
        CXX11LIB_TUPLE_FOUND)
ENDIF (NOT DEFINED CXX11LIB_TUPLE_FOUND)
