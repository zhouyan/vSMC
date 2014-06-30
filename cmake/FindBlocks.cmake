# ============================================================================
#  cmake/FindBlocks.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
#  See LICENSE for details.
# ============================================================================

# Find Blocks support
#
# The following variable is set
#
# BLOCKS_FOUND          - TRUE if Blocks is found and work correctly
# BLOCKS_COMPILE_FLAGS  - Comiple time flags for enable blocks support
# BLOCKS_LINK_LIBRARIES - Runtime libraries that should be linked to
#
# The following variables affect the behavior of this module
#
# BLOCKS_LIB_PATH - The path CMake shall try to find libraries first

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindBlocks.cpp BLOCKS_TEST_SOURCE)

IF (NOT DEFINED BLOCKS_LINK_LIBRARIES)
    IF (APPLE)
        MESSAGE (STATUS "Blocks link libraries not required (Mac OS X)")
        SET (BLOCKS_LINK_LIBRARIES "" CACHE STRING "Blocks link libraries")
    ELSE (APPLE)
        FIND_LIBRARY (BLOCKS_LINK_LIBRARIES BlocksRuntime
            PATH ${BLOCKS_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
        FIND_LIBRARY (BLOCKS_LINK_LIBRARIES BlocksRuntime)
        IF (BLOCKS_LINK_LIBRARIES)
            MESSAGE (STATUS "Found Blocks libraries: ${BLOCKS_LINK_LIBRARIES}")
        ELSE (BLOCKS_LINK_LIBRARIES)
            MESSAGE (STATUS "NOT Found Blocks libraries")
        ENDIF (BLOCKS_LINK_LIBRARIES)
    ENDIF (APPLE)
ENDIF (NOT DEFINED BLOCKS_LINK_LIBRARIES)

IF (APPLE)
    SET (BLOCKS_BASIC_FOUND TRUE)
ELSE (APPLE)
    IF (BLOCKS_LINK_LIBRARIES)
        SET (BLOCKS_BASIC_FOUND TRUE)
    ELSE (BLOCKS_LINK_LIBRARIES)
        SET (BLOCKS_BASIC_FOUND FALSE)
    ENDIF (BLOCKS_LINK_LIBRARIES)
ENDIF (APPLE)

IF (BLOCKS_BASIC_FOUND AND NOT DEFINED BLOCKS_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (SAFE_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
    SET (CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES}
        ${BLOCKS_LINK_LIBRARIES})

    MESSAGE (STATUS "Try blocks compile flags = [-fblocks]")
    SET (CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS} "-fblocks")
    CHECK_CXX_SOURCE_RUNS ("${BLOCKS_TEST_SOURCE}" BLOCKS_FOUND)
    SET (CMAKE_REQUIRED_FLAGS ${SAFE_CMAKE_REQUIRED_FLAGS})

    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
    IF (BLOCKS_FOUND)
        MESSAGE (STATUS "Found Blocks support")
    ELSE (BLOCKS_FOUND)
        MESSAGE (STATUS "NOT Found Blocks")
    ENDIF (BLOCKS_FOUND)
ENDIF (BLOCKS_BASIC_FOUND AND NOT DEFINED BLOCKS_FOUND)
