# Find TestU01
#
# This module can be used to find TestU01 headers and libraries
#
# The following variables are set
#
# TestU01_FOUND          - TRUE if TestU01 headers and libraries are found
#                          But it is untested by real OpenCL programs
# TestU01_INCLUDE_DIR    - The directory containing OpenCL headers
# TestU01_LINK_LIBRARIES - TBB libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# TestU01_INC_PATH - The path CMake shall try to find headers first
# TestU01_LIB_PATH - The path CMake shall try to find libraries first

IF (NOT DEFINED TestU01_FOUND)
    IF (NOT DEFINED TestU01_LINK_LIBRARIES)
        FIND_LIBRARY (TestU01_LINK_LIBRARIES testu01
            PATHS ${TestU01_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
        FIND_LIBRARY (TestU01_LINK_LIBRARIES testu01)
        IF (TestU01_LINK_LIBRARIES)
            MESSAGE (STATUS "Found TestU01 libraries: ${TestU01_LINK_LIBRARIES}")
        ELSE (TestU01_LINK_LIBRARIES)
            MESSAGE (STATUS "NOT Found TestU01 libraries")
        ENDIF (TestU01_LINK_LIBRARIES)
    ENDIF (NOT DEFINED TestU01_LINK_LIBRARIES)

    IF (NOT DEFINED TestU01_INCLUDE_DIR)
        FIND_PATH (TestU01_INCLUDE_DIR bbattery.h
            PATHS ${TestU01_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
        FIND_PATH (TestU01_INCLUDE_DIR bbattery.h)
        IF (TestU01_INCLUDE_DIR)
            MESSAGE (STATUS "Found TestU01 headers: ${TestU01_INCLUDE_DIR}")
        ELSE (TestU01_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found TestU01 headers")
            SET (TestU01_FOUND FALSE)
        ENDIF (TestU01_INCLUDE_DIR)
    ENDIF (NOT DEFINED TestU01_INCLUDE_DIR)

    IF (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
        MESSAGE (STATUS "Found TestU01")
        SET (TestU01_FOUND TRUE CACHE BOOL "Found TestU01")
    ELSE (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found TestU01")
        SET (TestU01_FOUND FALSE CACHE BOOL "Not Found TestU01")
    ENDIF (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
ENDIF (NOT DEFINED TestU01_FOUND)
