# ============================================================================
#  cmake/FindRdRand.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find RdRand support
#
# The following variable is set
#
# RDRAND_FOUND - TRUE if RdRand is found and work correctly

IF (DEFINED RDRAND_FOUND)
    RETURN ()
ENDIF (DEFINED RDRAND_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRdRand.cpp RDRAND_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${RDRAND_TEST_SOURCE}" RDRAND_FOUND)
IF (RDRAND_FOUND)
    MESSAGE (STATUS "Found RdRand support")
ELSE (RDRAND_FOUND)
    MESSAGE (STATUS "NOT Found RdRand support")
ENDIF (RDRAND_FOUND)
