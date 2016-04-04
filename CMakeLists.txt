# ============================================================================
#  vSMC/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2016, Yan Zhou
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
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

# Disable in source build
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "In-source builds not allowed")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

PROJECT(vSMC C CXX)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Options
OPTION(VSMC_ENABLE_LIBRARY "Enable building of library" ON)
OPTION(VSMC_ENABLE_EXAMPLE "Enable building of example" ON)

# User configurations
IF(EXISTS ${PROJECT_SOURCE_DIR}/user_options.cmake)
    INCLUDE(${PROJECT_SOURCE_DIR}/user_options.cmake)
ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/user_options.cmake)

# Configuration files
FILE(READ ${PROJECT_SOURCE_DIR}/README.md README)
FILE(READ ${PROJECT_SOURCE_DIR}/NEWS.md NEWS)
SET(DOC_SOURCE  ${PROJECT_SOURCE_DIR}/doc)
SET(DOC_INCLUDE ${PROJECT_SOURCE_DIR}/include)
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config/Doxyfile.in
    ${PROJECT_BINARY_DIR}/Doxyfile)
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config/main.md.in
    ${PROJECT_SOURCE_DIR}/doc/main.md)
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config/news.md.in
    ${PROJECT_SOURCE_DIR}/doc/news.md)

# Installation
IF(NOT VSMC_INSTALL_INC_DIR)
    SET(VSMC_INSTALL_INC_DIR include)
ENDIF(NOT VSMC_INSTALL_INC_DIR)
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/vsmc
    DESTINATION ${VSMC_INSTALL_INC_DIR}/vsmc
    PATTERN ".DS_Store" EXCLUDE PATTERN "*.swp" EXCLUDE)

# Documentations
INCLUDE(FindDoxygen)
FILE(GLOB_RECURSE DOC_SOURCE_FILES      ${PROJECT_SOURCE_DIR}/doc/*.*)
FILE(GLOB_RECURSE DOC_INCLUDE_FILES_C   ${PROJECT_SOURCE_DIR}/include/*.h)
FILE(GLOB_RECURSE DOC_INCLUDE_FILES_CPP ${PROJECT_SOURCE_DIR}/include/*.hpp)
IF(DOXYGEN_FOUND)
    ADD_CUSTOM_COMMAND(
        OUTPUT  ${PROJECT_BINARY_DIR}/doc
        DEPENDS ${PROJECT_BINARY_DIR}/Doxyfile
        ${DOC_SOURCE_FILES} ${DOC_INCLUDE_FILES_C} ${DOC_INCLUDE_FILES_CPP}
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMAND ${DOXYGEN_EXECUTABLE} ARGS Doxyfile)
    ADD_CUSTOM_TARGET(docs DEPENDS ${PROJECT_BINARY_DIR}/doc)
ENDIF(DOXYGEN_FOUND)

IF(NOT VSMC_ENABLE_LIBRARY AND NOT VSMC_ENABLE_EXAMPLE)
    RETURN()
ENDIF(NOT VSMC_ENABLE_LIBRARY AND NOT VSMC_ENABLE_EXAMPLE)

# Check features
INCLUDE(vSMCFunction)

##############################################################################
# Enable backends
##############################################################################

MESSAGE(STATUS "=================== Enable backends ===================")
FOREACH(backend ${BACKENDS})
    MESSAGE(STATUS ${backend})
ENDFOREACH(backend ${BACKENDS})

##############################################################################
# Enable features
##############################################################################

MESSAGE(STATUS "=================== Enable features ===================")
FOREACH(feature ${FEATURES})
    MESSAGE(STATUS ${feature})
ENDFOREACH(feature ${FEATURES})

##############################################################################
# Library
##############################################################################

IF(VSMC_ENABLE_LIBRARY AND MKL_FOUND)
    ADD_CUSTOM_TARGET(lib)
    ADD_SUBDIRECTORY(lib EXCLUDE_FROM_ALL)
ENDIF(VSMC_ENABLE_LIBRARY AND MKL_FOUND)

##############################################################################
# Example
##############################################################################

IF(VSMC_ENABLE_EXAMPLE AND (MKL_FOUND OR BLAS_FOUND))
    ADD_CUSTOM_TARGET(example)
    ADD_CUSTOM_TARGET(check)
    ADD_CUSTOM_TARGET(pdf)
    ADD_SUBDIRECTORY(example EXCLUDE_FROM_ALL)
ENDIF(VSMC_ENABLE_EXAMPLE AND (MKL_FOUND OR BLAS_FOUND))
