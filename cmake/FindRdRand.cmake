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
# RD_RAND_FOUND - TRUE if RdRand is found and work correctly

IF (DEFINED RD_RAND_FOUND)
    RETURN ()
ENDIF (DEFINED RD_RAND_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRdRand.cpp RD_RAND_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${RD_RAND_TEST_SOURCE}" RD_RAND_FOUND)
IF (RD_RAND_FOUND)
    MESSAGE (STATUS "Found RdRand support")
ELSE (RD_RAND_FOUND)
    MESSAGE (STATUS "NOT Found RdRand support")
ENDIF (RD_RAND_FOUND)
