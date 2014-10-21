# ============================================================================
#  cmake/FindRdSeed.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find RdSeed support
#
# The following variable is set
#
# RDSEED_FOUND - TRUE if RdSeed is found and work correctly

IF (DEFINED RDSEED_FOUND)
    RETURN ()
ENDIF (DEFINED RDSEED_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRdSeed.cpp RDSEED_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${RDSEED_TEST_SOURCE}" RDSEED_FOUND)
IF (RDSEED_FOUND)
    MESSAGE (STATUS "Found RdSeed support")
ELSE (RDSEED_FOUND)
    MESSAGE (STATUS "NOT Found RdSeed support")
ENDIF (RDSEED_FOUND)
