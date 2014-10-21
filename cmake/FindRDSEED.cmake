# ============================================================================
#  cmake/FindRDSEED.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find RDSEED support
#
# The following variable is set
#
# RDSEED_FOUND - TRUE if RDSEED is found and work correctly

IF (DEFINED RDSEED_FOUND)
    RETURN ()
ENDIF (DEFINED RDSEED_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRDSEED.cpp RDSEED_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${RDSEED_TEST_SOURCE}" RDSEED_FOUND)
IF (RDSEED_FOUND)
    MESSAGE (STATUS "Found RDSEED support")
ELSE (RDSEED_FOUND)
    MESSAGE (STATUS "NOT Found RDSEED support")
ENDIF (RDSEED_FOUND)
