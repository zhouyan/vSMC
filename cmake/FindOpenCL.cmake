# Find OpenCL
#
# This module can be used to find OpenCL headers and libraries
#
# The following variables are set
#
# OPENCL_FOUND          - TRUE if OpenCL headers and libraries are found
#                         But it is untested by real OpenCL programs
# OpenCL_INCLUDE_DIR    - The directory containing OpenCL headers
# OpenCL_LINK_LIBRARIES - TBB libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# OpenCL_INC_PATH - The path CMake shall try to find headers first
# OpenCL_LIB_PATH - The path CMake shall try to find libraries first

IF (NOT OPENCL_FOUND)
    UNSET (OPENCL_FOUND CACHE)
    IF (NOT OpenCL_LINK_LIBRARIES)
        UNSET (OpenCL_LINK_LIBRARIES CACHE)
        FIND_LIBRARY (OpenCL_LINK_LIBRARIES OpenCL
            PATHS ${OpenCL_LIB_PATH} ENV LIBRARY_PATH)
        IF (OpenCL_LINK_LIBRARIES)
            SET (OpenCL_LINK_LIBRARIES_RELEASE ${OpenCL_LINK_LIBRARIES})
            SET (OpenCL_LINK_LIBRARIES_DEBUG ${OpenCL_LINK_LIBRARIES})
            MESSAGE (STATUS "Found OpenCL libraries: ${OpenCL_LINK_LIBRARIES}")
        ELSE (OpenCL_LINK_LIBRARIES)
            MESSAGE (STATUS "NOT Found OpenCL libraries")
        ENDIF (OpenCL_LINK_LIBRARIES)
    ENDIF (NOT OpenCL_LINK_LIBRARIES)

    IF (NOT OpenCL_INCLUDE_DIR)
        UNSET (OpenCL_INCLUDE_DIR CACHE)
        IF (APPLE)
            FIND_PATH (OpenCL_INCLUDE_DIR OpenCL/opencl.h
                PATHS ${OpenCL_INC_PATH} ENV CPATH)
        ELSE (APPLE)
            FIND_PATH (OpenCL_INCLUDE_DIR CL/opencl.h
                PATHS ${OpenCL_INC_PATH} ENV CPATH)
        ENDIF (APPLE)
        IF (OpenCL_INCLUDE_DIR)
            MESSAGE (STATUS "Found OpenCL headers: ${OpenCL_INCLUDE_DIR}")
        ELSE (OpenCL_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found OpenCL headers")
            SET (OPENCL_FOUND FALSE)
        ENDIF (OpenCL_INCLUDE_DIR)
    ENDIF (NOT OpenCL_INCLUDE_DIR)

    IF (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)
        MESSAGE (STATUS "Found OpenCL")
        SET (OPENCL_FOUND TRUE CACHE BOOL "Found OpenCL")
    ELSE (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found OpenCL")
        SET (OPENCL_FOUND FALSE CACHE BOOL "Not Found OpenCL")
    ENDIF (OpenCL_LINK_LIBRARIES AND OpenCL_INCLUDE_DIR)
ENDIF (NOT OPENCL_FOUND)
