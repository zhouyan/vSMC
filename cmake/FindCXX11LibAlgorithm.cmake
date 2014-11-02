# ============================================================================
#  cmake/FindCXX11LibAlgorithm.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find C++11 <algorithm> support
#
# The following variable is set
#
# CXX11LIB_ALGORITHM_FOUND - TRUE if C++11 <algorithm> is found and work
#                            correctly

IF (DEFINED CXX11LIB_ALGORITHM_FOUND)
    RETURN ()
ENDIF (DEFINED CXX11LIB_ALGORITHM_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCXX11LibAlgorithm.cpp
    CXX11LIB_ALGORITHM_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${CXX11LIB_ALGORITHM_TEST_SOURCE}"
    CXX11LIB_ALGORITHM_FOUND)
