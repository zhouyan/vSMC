# ============================================================================
#  cmake/FindRandom123.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find Random123
#
# This module can be used to find Random123 headers
#
# The following variables are set
#
# RANDOM123_FOUND       - TRUE if Random123 headers are found
# Random123_INCLUDE_DIR - The directory containing OpenCL headers
#
# The following variables affect the behavior of this module
#
# Random123_INC_PATH - The path CMake shall try to find headers first

IF (DEFINED RANDOM123_FOUND)
    RETURN ()
ENDIF (DEFINED RANDOM123_FOUND)

IF (NOT DEFINED Random123_INCLUDE_DIR)
    FIND_PATH (Random123_INCLUDE_DIR Random123/threefry.h
        PATHS ${Random123_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (Random123_INCLUDE_DIR Random123/threefry.h)
    IF (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "Found Random123 headers: ${Random123_INCLUDE_DIR}")
        SET (RANDOM123_FOUND TRUE CACHE BOOL "Found Random123")
    ELSE (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found Random123 headers")
        SET (RANDOM123_FOUND FALSE CACHE BOOL "Not Found Random123")
    ENDIF (Random123_INCLUDE_DIR)
ENDIF (NOT DEFINED Random123_INCLUDE_DIR)
