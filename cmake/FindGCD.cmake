# ============================================================================
#  cmake/FindGCD.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find Apple GCD support
#
# The following variable is set
#
# GCD_FOUND          - TRUE if Apple GCD is found and work correctly.
#                      But it is untested by real GCD programs
# GCD_INCLUDE_DIR    - The directory containing GCD headers
# GCD_LINK_LIBRARIES - GCD libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# GCD_INC_PATH - The path CMake shall try to find headers first
# GCD_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED GCD_FOUND)
    RETURN ()
ENDIF (DEFINED GCD_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindGCD.cpp GCD_TEST_SOURCE)

IF (NOT DEFINED GCD_LINK_LIBRARIES)
    IF (APPLE)
        MESSAGE (STATUS "GCD link libraries not required (Mac OS X)")
        SET (GCD_LINK_LIBRARIES "" CACHE STRING "GCD link libraries")
    ELSE (APPLE)
        FIND_LIBRARY (GCD_LINK_LIBRARIES dispatch
            PATHS ${GCD_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
        FIND_LIBRARY (GCD_LINK_LIBRARIES dispatch)
        IF (GCD_LINK_LIBRARIES)
            MESSAGE (STATUS "Found GCD libraries: ${GCD_LINK_LIBRARIES}")
        ELSE (GCD_LINK_LIBRARIES)
            MESSAGE (STATUS "NOT Found GCD libraries")
        ENDIF (GCD_LINK_LIBRARIES)
    ENDIF (APPLE)
ENDIF (NOT DEFINED GCD_LINK_LIBRARIES)

IF (NOT DEFINED GCD_INCLUDE_DIR)
    FIND_PATH (GCD_INCLUDE_DIR dispatch/dispatch.h
        PATHS ${GCD_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (GCD_INCLUDE_DIR dispatch/dispatch.h)
    IF (GCD_INCLUDE_DIR)
        MESSAGE (STATUS "Found GCD headers: ${GCD_INCLUDE_DIR}")
    ELSE (GCD_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found GCD headers")
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_INCLUDE_DIR)
ENDIF (NOT DEFINED GCD_INCLUDE_DIR)

IF (APPLE)
    IF (GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND TRUE)
    ELSE (GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_INCLUDE_DIR)
ELSE (APPLE)
    IF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND TRUE)
    ELSE (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
ENDIF (APPLE)

IF (GCD_BASIC_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${GCD_INCLUDE_DIR})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${GCD_LINK_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS ("${GCD_TEST_SOURCE}" GCD_FOUND)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
    IF (GCD_FOUND)
        MESSAGE (STATUS "Found GCD")
    ELSE (GCD_FOUND)
        MESSAGE (STATUS "NOT Found GCD")
    ENDIF (GCD_FOUND)
ENDIF (GCD_BASIC_FOUND)
