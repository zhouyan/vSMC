# Find Google Test
#
# The following variable is set
#
# GTEST_FOUND          - TRUE if Google Test  is found and work correctly.
#                        But it is untested by real GTEST programs
# GTEST_INCLUDE_DIR    - The directory containing GTEST headers
# GTEST_LINK_LIBRARIES - Google Test libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# GTEST_INC_PATH - The path CMake shall try to find headers first
# GTEST_LIB_PATH - The path CMake shall try to find libraries first

IF (NOT DEFINED GTEST_FOUND)
    IF (NOT DEFINED GTEST_LINK_LIBRARIES)
        INCLUDE (FindThreads)
        FIND_LIBRARY (GTEST_LINK_LIBRARIES_FOUND gtest
            PATH ${GTEST_LIB_PATH} ENV LIBRARY_PATH)
        IF (GTEST_LINK_LIBRARIES_FOUND)
            SET (GTEST_LINK_LIBRARIES ${GTEST_LINK_LIBRARIES_FOUND}
                ${CMAKE_THREAD_LIBS_INIT} CACHE STRING "Link to Google Test")
            MESSAGE (STATUS ${CMAKE_THREAD_LIBS_INIT})
            MESSAGE (STATUS
                "Found Google Test libraries: ${GTEST_LINK_LIBRARIES}")
        ELSE (GTEST_LINK_LIBRARIES_FOUND)
            MESSAGE (STATUS "NOT Found Google Test libraries")
        ENDIF (GTEST_LINK_LIBRARIES_FOUND)
    ENDIF (NOT DEFINED GTEST_LINK_LIBRARIES)

    IF (NOT DEFINED GTEST_INCLUDE_DIR)
        FIND_PATH (GTEST_INCLUDE_DIR dispatch/dispatch.h
            PATHS ${GTEST_INC_PATH} ENV CPATH)
        IF (GTEST_INCLUDE_DIR)
            MESSAGE (STATUS "Found Google Test headers: ${GTEST_INCLUDE_DIR}")
        ELSE (GTEST_INCLUDE_DIR)
            MESSAGE (STATUS "NOT Found Google Test headers")
            SET (GTEST_FOUND FALSE)
        ENDIF (GTEST_INCLUDE_DIR)
    ENDIF (NOT DEFINED GTEST_INCLUDE_DIR)

    IF (GTEST_LINK_LIBRARIES AND GTEST_INCLUDE_DIR)
        MESSAGE (STATUS "Found Google Test")
        SET (GTEST_FOUND TRUE CACHE BOOL "Found Google Test")
    ELSE (GTEST_LINK_LIBRARIES AND GTEST_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found Google Test")
        SET (GTEST_FOUND FALSE CACHE BOOL "Not Found Google Test")
    ENDIF (GTEST_LINK_LIBRARIES AND GTEST_INCLUDE_DIR)
ENDIF (NOT DEFINED GTEST_FOUND)
