# ============================================================================
#  cmake/FindRDTSCP.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distributed under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find RDTSCP support
#
# The following variable is set
#
# RDTSCP_FOUND - TRUE if RDTSCP instruction is found and work correctly

IF (DEFINED RDTSCP_FOUND)
    RETURN ()
ENDIF (DEFINED RDTSCP_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindRDTSCP.cpp RDTSCP_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${RDTSCP_TEST_SOURCE}" RDTSCP_FOUND)
IF (RDTSCP_FOUND)
    MESSAGE (STATUS "Found RDTSCP support")
ELSE (RDTSCP_FOUND)
    MESSAGE (STATUS "NOT Found RDTSCP support")
ENDIF (RDTSCP_FOUND)

