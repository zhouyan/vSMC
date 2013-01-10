# Find Random123
#
# This module can be used to find Random123 headers and libraries
#
# The following variables are set
#
# RANDOM123_FOUND          - TRUE if Random123 headers and libraries are found
#                            But it is untested by real Random123 programs
# Random123_INCLUDE_DIR    - The directory containing OpenCL headers
#
# The following variables affect the behavior of this module
#
# Random123_INC_PATH - The path CMake shall try to find headers first
# Random123_LIB_PATH - The path CMake shall try to find libraries first

IF (NOT RANDOM123_FOUND OR NOT Random123_INCLUDE_DIR)
    UNSET (RANDOM123_FOUND CACHE)
    UNSET (Random123_INCLUDE_DIR CACHE)
    FIND_PATH (Random123_INCLUDE_DIR Random123/threefry.h
        PATHS ${Random123_INC_PATH} ENV CPATH)
    IF (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "Found Random123 headers: ${Random123_INCLUDE_DIR}")
        SET (RANDOM123_FOUND TRUE CACHE BOOL "Found Random123")
    ELSE (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found Random123 headers")
        SET (RANDOM123_FOUND FALSE CACHE BOOL "Not Found Random123")
    ENDIF (Random123_INCLUDE_DIR)
ENDIF (NOT RANDOM123_FOUND OR NOT Random123_INCLUDE_DIR)
