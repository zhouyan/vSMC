# ============================================================================
#  cmake/FindOpenCL.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find OpenCL
#
# This module can be used to find OpenCL headers and libraries
#
# The following variables are set
#
# OPENCL_FOUND          - TRUE if OpenCL headers and libraries are found
#                         But it is untested by real OpenCL programs
# OpenCL_INCLUDE_DIR    - The directory containing OpenCL headers
# OpenCL_LINK_LIBRARIES - OpenCL libraries that shall be linked to
# OpenCL_DEFINITIONS    - OpenCL compile time definitions
#
# The following variables affect the behavior of this module
#
# OpenCL_INC_PATH - The path CMake shall try to find headers first
# OpenCL_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED OPENCL_FOUND)
    RETURN ()
ENDIF (DEFINED OPENCL_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindOpenCL.cpp OpenCL_TEST_SOURCE)

IF (NOT DEFINED OpenCL_LINK_LIBRARIES)
    FIND_LIBRARY (OpenCL_LINK_LIBRARIES OpenCL
        PATHS ${OpenCL_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
    FIND_LIBRARY (OpenCL_LINK_LIBRARIES OpenCL)
    IF (OpenCL_LINK_LIBRARIES)
        MESSAGE (STATUS "Found OpenCL libraries: ${OpenCL_LINK_LIBRARIES}")
    ELSE (OpenCL_LINK_LIBRARIES)
        MESSAGE (STATUS "NOT Found OpenCL libraries")
    ENDIF (OpenCL_LINK_LIBRARIES)
ENDIF (NOT DEFINED OpenCL_LINK_LIBRARIES)

IF (NOT DEFINED OpenCL_INCLUDE_DIR)
    IF (APPLE)
        FIND_PATH (OpenCL_INCLUDE_DIR OpenCL/opencl.h
            PATHS ${OpenCL_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
        FIND_PATH (OpenCL_INCLUDE_DIR OpenCL/opencl.h)
    ELSE (APPLE)
        FIND_PATH (OpenCL_INCLUDE_DIR CL/opencl.h
            PATHS ${OpenCL_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
        FIND_PATH (OpenCL_INCLUDE_DIR CL/opencl.h)
    ENDIF (APPLE)
    IF (OpenCL_INCLUDE_DIR)
        MESSAGE (STATUS "Found OpenCL headers: ${OpenCL_INCLUDE_DIR}")
    ELSE (OpenCL_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found OpenCL headers")
        SET (OPENCL_FOUND FALSE)
    ENDIF (OpenCL_INCLUDE_DIR)
ENDIF (NOT DEFINED OpenCL_INCLUDE_DIR)

SET (OpenCL_DEFINITIONS -D__CL_ENABLE_EXCEPTIONS
    CACHE STRING "OpenCL compile time definitions")

IF (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)
    SET (OpenCL_BASIC_FOUND TRUE)
ELSE (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)
    SET (OpenCL_BASIC_FOUND FALSE)
ENDIF (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)

IF (OpenCL_BASIC_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
    SET (SAFE_CMAKE_REQUIRED_INCLUDES  ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        ${OpenCL_DEFINITIONS})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${OpenCL_INCLUDE_DIR})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${OpenCL_LINK_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS ("${OpenCL_TEST_SOURCE}" OpenCL_TEST_SOURCE_RUNS)
    IF (OpenCL_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "Found OpenCL")
        SET (OPENCL_FOUND TRUE CACHE BOOL "Found OpenCL")
    ELSE (OpenCL_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "NOT Found OpenCL")
        SET (OPENCL_FOUND FALSE CACHE BOOL "NOT Found OpenCL")
    ENDIF (OpenCL_TEST_SOURCE_RUNS)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ELSE (OpenCL_BASIC_FOUND)
    MESSAGE (STATUS "NOT Found OpenCL")
    SET (OPENCL_FOUND FALSE CACHE BOOL "NOT Found OpenCL")
ENDIF (OpenCL_BASIC_FOUND)
