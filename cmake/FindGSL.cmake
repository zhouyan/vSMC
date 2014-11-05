# ============================================================================
#  cmake/FindGSL.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distributed under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find GSL
#
# This module can be used to find GSL headers and libraries
#
# The following variables are set
#
# GSL_FOUND                - TRUE if GSL headers and libraries are found
#                            But it is untested by real OpenCL programs
# GSL_INCLUDE_DIR          - The directory containing OpenCL headers
# GSL_LINK_LIBRARIES       - GSL libraries that shall be linked to
# GSL_CBLAS_LINK_LIBRARIES - GSL BLAS libraries that shall be linked if other
#                            BLAS implementation is not linked
#
# The following variables affect the behavior of this module
#
# GSL_INC_PATH - The path CMake shall try to find headers first
# GSL_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED GSL_FOUND)
    RETURN ()
ENDIF (DEFINED GSL_FOUND)

IF (NOT DEFINED GSL_LINK_LIBRARIES)
    FIND_LIBRARY (GSL_LINK_LIBRARIES gsl
        PATHS ${GSL_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
    FIND_LIBRARY (GSL_LINK_LIBRARIES gsl)
    IF (GSL_LINK_LIBRARIES)
        MESSAGE (STATUS "Found GSL libraries: ${GSL_LINK_LIBRARIES}")
    ELSE (GSL_LINK_LIBRARIES)
        MESSAGE (STATUS "NOT Found GSL libraries")
    ENDIF (GSL_LINK_LIBRARIES)
ENDIF (NOT DEFINED GSL_LINK_LIBRARIES)

IF (NOT DEFINED GSL_CBLAS_LINK_LIBRARIES)
    FIND_LIBRARY (GSL_CBLAS_LINK_LIBRARIES gslcblas
        PATHS ${GSL_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
    FIND_LIBRARY (GSL_CBLAS_LINK_LIBRARIES gslcblas)
    IF (GSL_CBLAS_LINK_LIBRARIES)
        MESSAGE (STATUS
            "Found GSL CBLAS libraries: ${GSL_CBLAS_LINK_LIBRARIES}")
    ELSE (GSL_CBLAS_LINK_LIBRARIES)
        MESSAGE (STATUS "NOT Found GSL CBLAS libraries")
    ENDIF (GSL_CBLAS_LINK_LIBRARIES)
ENDIF (NOT DEFINED GSL_CBLAS_LINK_LIBRARIES)

IF (NOT DEFINED GSL_INCLUDE_DIR)
    FIND_PATH (GSL_INCLUDE_DIR gsl/gsl_rng.h
        PATHS ${GSL_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (GSL_INCLUDE_DIR gsl/gsl_rng.h)
    IF (GSL_INCLUDE_DIR)
        MESSAGE (STATUS "Found GSL headers: ${GSL_INCLUDE_DIR}")
    ELSE (GSL_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found GSL headers")
        SET (GSL_FOUND FALSE)
    ENDIF (GSL_INCLUDE_DIR)
ENDIF (NOT DEFINED GSL_INCLUDE_DIR)

IF (GSL_LINK_LIBRARIES AND GSL_INCLUDE_DIR)
    MESSAGE (STATUS "Found GSL")
    SET (GSL_FOUND TRUE CACHE BOOL "Found GSL")
ELSE (GSL_LINK_LIBRARIES AND GSL_INCLUDE_DIR)
    MESSAGE (STATUS "NOT Found GSL")
    SET (GSL_FOUND FALSE CACHE BOOL "Not Found GSL")
ENDIF (GSL_LINK_LIBRARIES AND GSL_INCLUDE_DIR)
