# ============================================================================
#  cmake/FindAESNI.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find AES-NI support
#
# The following variable is set
#
# AESNI_FOUND - TRUE if AES-NI is found and work correctly

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindAESNI.cpp AESNI_TEST_SOURCE)

IF (NOT DEFINED AESNI_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    CHECK_CXX_SOURCE_RUNS ("${AESNI_TEST_SOURCE}" AESNI_FOUND)
    IF (AESNI_FOUND)
        MESSAGE (STATUS "Found AES-NI support")
    ELSE (AESNI_FOUND)
        MESSAGE (STATUS "NOT Found AES-NI support")
    ENDIF (AESNI_FOUND)
ENDIF (NOT DEFINED AESNI_FOUND)
