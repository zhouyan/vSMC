# ============================================================================
#  vSMC/cmake/FindGSL.cmake
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013,2014, Yan Zhou
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
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
