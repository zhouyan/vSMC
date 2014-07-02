# ============================================================================
#  cmake/FindCPUID.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find CPUID support
#
# The following variable is set
#
# CPUID_FOUND - TRUE if CPUID instruction is found and work correctly

IF (DEFINED CPUID_FOUND)
    RETURN ()
ENDIF (DEFINED CPUID_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindCPUID.cpp CPUID_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
CHECK_CXX_SOURCE_RUNS ("${CPUID_TEST_SOURCE}" CPUID_FOUND)
IF (CPUID_FOUND)
    MESSAGE (STATUS "Found CPUID support")
ELSE (CPUID_FOUND)
    MESSAGE (STATUS "NOT Found CPUID support")
ENDIF (CPUID_FOUND)
