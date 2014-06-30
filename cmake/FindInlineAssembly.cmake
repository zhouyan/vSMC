# ============================================================================
#  cmake/FindInlineAssembly.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo

#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find GNU style inline assembly support
#
# The following variable is set
#
# INLINE_ASSEMBLY_FOUND - TRUE if GNU style inline assembly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindInlineAssembly.cpp
    INLINE_ASSEMBLY_TEST_SOURCE)

IF (NOT DEFINED INLINE_ASSEMBLY_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${INLINE_ASSEMBLY_TEST_SOURCE}"
        INLINE_ASSEMBLY_FOUND)
    IF (INLINE_ASSEMBLY_FOUND)
        MESSAGE (STATUS "Found GNU style inline assembly support")
    ELSE (INLINE_ASSEMBLY_FOUND)
        MESSAGE (STATUS "NOT Found GNU style inline assembly support")
    ENDIF (INLINE_ASSEMBLY_FOUND)
ENDIF (NOT DEFINED INLINE_ASSEMBLY_FOUND)
